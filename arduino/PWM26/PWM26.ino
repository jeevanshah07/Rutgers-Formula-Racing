/*
 * Radiator Fan + Pump PWM Controller
 * Target: ATmega328P (Arduino Mini)
 *
 * ── Timer allocation ────────────────────────────────────────────────────────
 *   Timer 0 → D5, D6   untouched — reserved for millis() / delay()
 *   Timer 1 → D9       pump PWM, Phase Correct PWM, 500 Hz
 *   Timer 2 → D3       fan PWM,  Fast PWM,          25 kHz
 *                       D11 unavailable as PWM, but remains SPI MOSI
 *
 * ── PWM API ─────────────────────────────────────────────────────────────────
 *   setPumpDuty(uint8_t percent)   0–100
 *   setFanDuty(uint8_t percent)    0–100
 *
 * ── DAQ usage ───────────────────────────────────────────────────────────────
 *   Temperature, fan duty, and pump duty are appended to PWM26.CSV on an
 *   SPI SD card once per second. D10 is the SD card chip-select pin.
 */

#include <math.h>
#include <SPI.h>
#include <SD.h>

// ── Build-time config ────────────────────────────────────────────────────────
#define SD_CS_PIN     10
#define LOG_INTERVAL  1000UL  // ms between SD-card samples
#define LOG_FILENAME  "PWM26.CSV"

// ── Pin assignments ──────────────────────────────────────────────────────────
#define PUMP_PIN  9           // OC1A — Timer 1 Phase Correct PWM
#define FAN_PIN   3           // OC2B — Timer 2 Fast PWM
#define TEMP_PIN  A2          // Coolant Temp 3 (NTC voltage divider)

// ── PWM frequencies ──────────────────────────────────────────────────────────
#define PUMP_FREQ 500.0       // Hz  — audible range is fine for pump
#define FAN_FREQ  25000.0     // Hz  — fan datasheet preferred frequency

// ── Thermistor constants ─────────────────────────────────────────────────────
const float R0       = 10000.0; // Nominal resistance at T0 (ohms)
const float Beta     = 3950.0;  // Beta coefficient (K) — verify against your part
const float T0       = 298.15;  // Nominal temperature (25 C in Kelvin)
const float R_fixed  = 10000.0; // R3110 pull-up resistor (ohms)
const float tempTune = 0.25;    // Fine-tune offset (C)
const bool SERIAL_OUTPUT = false;

// ── Temperature thresholds ───────────────────────────────────────────────────
const float TEMP_LOW  = 30.0;   // Below → fans off
const float TEMP_HIGH = 45.0;   // Above → fans full

// ── Runtime state ────────────────────────────────────────────────────────────
uint8_t pumpDuty = 20;
uint8_t fanDuty  = 0;
float   tC1      = 0.0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500; // ms
unsigned long lastLogTime = 0;
bool sdReady = false;
File logFile;

// ════════════════════════════════════════════════════════════════════════════
// Timer setup
// ════════════════════════════════════════════════════════════════════════════

/*
 * Timer 1 — Phase Correct PWM on D9 (OC1A)
 *
 * Mode 10: Phase Correct PWM, TOP = ICR1
 *   WGM13=1, WGM12=0, WGM11=1, WGM10=0
 *   COM1A1=1 → non-inverting on OC1A (D9)
 *   CS10=1   → prescaler /1
 *
 * Phase Correct counts 0→TOP→0, so:
 *   freq = F_CPU / (2 × prescaler × TOP)
 *   TOP  = F_CPU / (2 × prescaler × freq)
 *
 * Duty: OCR1A / ICR1  (0 = 0%, ICR1 = 100%)
 *
 * Phase Correct is preferred over Fast PWM for motor control — the symmetric
 * waveform produces less electrical noise and lower torque ripple.
 */
void setupTimer1(float freq) {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13)  | (1 << CS10);  // prescaler /1, mode 10
  ICR1   = (uint16_t)(F_CPU / (2.0 * freq));
  OCR1A  = 0;
}

/*
 * Timer 2 — Fast PWM on D3 (OC2B)
 *
 * Mode 7: Fast PWM, TOP = OCR2A
 *   WGM22=1, WGM21=1, WGM20=1
 *   COM2B1=1 → non-inverting on OC2B (D3)
 *   COM2A    → left clear so OC2A (D11) is not driven; OCR2A is free as TOP
 *   CS21=1   → prescaler /8
 *
 * freq = F_CPU / (prescaler × (TOP + 1))
 * TOP  = F_CPU / (prescaler × freq) - 1
 *      = 16,000,000 / (8 × 25,000) - 1 = 79
 *
 * Duty: OCR2B / OCR2A  (0 = 0%, OCR2A = 100%)
 *
 * ⚠ D11 must not be used as an output while Timer 2 is active — it shares
 *   OC2A which is now repurposed as the TOP register.
 *
 * TIMSK2/TIFR2 cleared first to ensure no stale interrupt state from the
 * bootloader interferes with the timer.
 */
void setupTimer2(float freq) {
  TIMSK2 = 0;
  TIFR2  = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22)  | (1 << CS21);
  OCR2A  = (uint8_t)(F_CPU / (8.0 * freq) - 1);
  OCR2B  = 0;
}

// ════════════════════════════════════════════════════════════════════════════
// PWM duty cycle setters
// ════════════════════════════════════════════════════════════════════════════

/*
 * Pump duty 0–100%
 * OCR1A ranges 0–ICR1; multiply first to avoid integer truncation.
 */
void setPumpDuty(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  OCR1A = (uint16_t)((uint32_t)ICR1 * (100-percent) / 100); // account for inversion
}

/*
 * Fan duty 0–100%
 * OCR2B ranges 0–OCR2A; multiply first to avoid integer truncation.
 */
void setFanDuty(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  OCR2B = (uint8_t)((uint16_t)OCR2A * (100-percent) / 100); // account for inversion
}

// ════════════════════════════════════════════════════════════════════════════
// SD-card logging
// ════════════════════════════════════════════════════════════════════════════

void logSample(unsigned long timestamp, float temperature,
               uint8_t fanPercent, uint8_t pumpPercent) {
  if (!sdReady) return;

  logFile.print(timestamp);
  logFile.print(',');
  logFile.print(temperature, 2);
  logFile.print(',');
  logFile.print(fanPercent);
  logFile.print(',');
  logFile.println(pumpPercent);
  logFile.flush();
}

// ════════════════════════════════════════════════════════════════════════════
// Thermistor reading
// ════════════════════════════════════════════════════════════════════════════

/*
 * Voltage divider: +5V → R_fixed → TEMP_PIN → NTC → GND
 *
 * R_therm = R_fixed × V / (Vcc - V)
 *
 * Steinhart-Hart (Beta approximation):
 *   1/T = 1/T0 + (1/Beta) × ln(R / R0)
 */
float adcToTemp(uint8_t pin) {
  int adc = analogRead(pin);

  if (adc <= 0 || adc >= 1023) return -999.0; // open or shorted thermistor

  float voltage = adc * (5.0f / 1023.0f);
  float R_therm = R_fixed * (voltage / (5.0f - voltage));
  float tK      = 1.0f / ((1.0f / T0) + (1.0f / Beta) * log(R_therm / R0));

  return tK - 273.15f + tempTune;
}

// ════════════════════════════════════════════════════════════════════════════
// Setup
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN,  OUTPUT);

  Serial.begin(115200);

  // pump start up sequence 
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(PUMP_PIN, HIGH);
  delayMicroseconds(3000);
  setPumpDuty(95);
  delay(100);



  // ── Configure timers ──────────────────────────────────────────────────
  setupTimer1(PUMP_FREQ);   // D9  — pump,  500 Hz, Phase Correct PWM
  setupTimer2(FAN_FREQ);    // D3  — fan,  25 kHz,  Fast PWM

  // Apply initial duty cycles
  setFanDuty(fanDuty);

  // ── Initialize the SD-card logger ─────────────────────────────────────
  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD card initialization failed; logging disabled."));
  } else {
    const bool newFile = !SD.exists(LOG_FILENAME);
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);

    if (!logFile) {
      Serial.println(F("Could not open PWM26.CSV; logging disabled."));
    } else {
      sdReady = true;
      if (newFile) {
        logFile.println(F("time_ms,coolant_temp_c,fan_duty_percent,pump_duty_percent"));
        logFile.flush();
      }
      Serial.println(F("SD-card logging ready at 1 Hz."));
    }
  }

  Serial.println(F("Fan/pump controller ready."));
}

// ════════════════════════════════════════════════════════════════════════════
// Loop
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  pumpDuty = 70;
  setPumpDuty(pumpDuty);

  // ── Read temperature ──────────────────────────────────────────────────
  tC1 = adcToTemp(TEMP_PIN);

  // ── Fan control ───────────────────────────────────────────────────────
  if (tC1 < TEMP_LOW) {
    fanDuty = 0;
  } else if (tC1 > TEMP_HIGH) {
    fanDuty = 100;
  } else {
    fanDuty = 60;
  }

  fanDuty = 15; 

  setFanDuty(fanDuty);

  // ── Append one SD-card sample per second ──────────────────────────────
  unsigned long now = millis();
  if (now - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = now;
    logSample(now, tC1, fanDuty, pumpDuty);
  }

  // ── Serial output every 500 ms ────────────────────────────────────────
  if (now - lastPrintTime >= printInterval && SERIAL_OUTPUT) {
    lastPrintTime = now;

    Serial.print(F("Coolant Temp 3: "));
    Serial.print(tC1);
    Serial.print(F(" C  |  Fan: "));
    Serial.print(fanDuty);
    Serial.print(F("%  |  Pump: "));
    Serial.print(pumpDuty);
    Serial.println(F("%"));
  }
}
