/* USER CODE BEGIN Header */
/**
 * Temperature Monitoring Board 2026 MK1
 * FLOAT-FREE VERSION (Fixed-point 0.1°C)
 */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

/* ------------------------------------------------------------------ */
/*                          DATA TYPES                                */
/* ------------------------------------------------------------------ */

typedef struct {
  int16_t min_temp; // 0.1°C
  int16_t max_temp; // 0.1°C
  int16_t avg_temp; // 0.1°C
  uint8_t min_channel;
  uint8_t max_channel;
  uint8_t num_enabled;
} TempStatistics_t;

/* ------------------------------------------------------------------ */
/*                         DEFINES                                    */
/* ------------------------------------------------------------------ */

#define MUX1_SYNC_PIN GPIO_PIN_0
#define MUX2_SYNC_PIN GPIO_PIN_1
#define MUX3_SYNC_PIN GPIO_PIN_2
#define MUX_SYNC_PORT GPIOB

#define NUM_MULTIPLEXERS 3
#define NUM_MUX_CHANNELS 32
#define TOTAL_SENSORS (NUM_MULTIPLEXERS * NUM_MUX_CHANNELS)

#define ADC_MAX_VALUE 4095u
#define ADC_VREF_mV 3300u

#define MUX_SYNC_DELAY_US 10u
#define MUX_SETTLE_US 1400u

/* ------------------------------------------------------------------ */
/*                     HAL HANDLES                                    */
/* ------------------------------------------------------------------ */

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;

static CAN_TxHeaderTypeDef TxHeader;
static uint8_t TxData[8];
static uint32_t TxMailbox;

/* ------------------------------------------------------------------ */
/*                     FUNCTION PROTOTYPES                            */
/* ------------------------------------------------------------------ */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);

static void delay_us(uint32_t us);
static uint16_t ADC_to_mV(uint16_t adc_value);
static int16_t mV_to_Temperature(uint16_t mV);

static void MUX_Init(void);
static void MUX_Enable(uint8_t mux_number, uint8_t enable);
static void MUX_SetChannel(uint8_t mux_number, uint8_t channel);

static int16_t Calculate_Channel_Mean(uint8_t mux, uint8_t chan,
                                      TempStatistics_t *stats);

static void Calculate_Module_Mean(TempStatistics_t *stats);

static void CAN_Init_Filter(void);
static HAL_StatusTypeDef CAN_SendTemperatureStatistics(TempStatistics_t *stats);

static const uint8_t mux_channel_count[NUM_MULTIPLEXERS] = {32, 32, 26};

/* ------------------------------------------------------------------ */
/*                       FIXED-POINT LUT                              */
/* ------------------------------------------------------------------ */

typedef struct {
  uint16_t mV;
  int16_t temp_dC; // 0.1°C
} LUT_t;

/* Replace values with real sensor table */
static const LUT_t temp_lut[] = {{500, -400}, {1000, 0},   {1500, 250},
                                 {2000, 500}, {2500, 750}, {3000, 1000}};

#define LUT_SIZE (sizeof(temp_lut) / sizeof(LUT_t))

/* ------------------------------------------------------------------ */
/*                          UTILITY                                   */
/* ------------------------------------------------------------------ */

static void delay_us(uint32_t us) {
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  uint32_t ticks = us * (SystemCoreClock / 1000000u);
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < ticks)
    ;
}

static uint16_t ADC_to_mV(uint16_t adc_value) {
  return (uint32_t)adc_value * ADC_VREF_mV / ADC_MAX_VALUE;
}

static int16_t mV_to_Temperature(uint16_t mV) {
  for (uint8_t i = 0; i < LUT_SIZE - 1; i++) {

    if (mV >= temp_lut[i].mV && mV <= temp_lut[i + 1].mV) {

      int32_t delta_mV = temp_lut[i + 1].mV - temp_lut[i].mV;
      int32_t delta_T = temp_lut[i + 1].temp_dC - temp_lut[i].temp_dC;
      int32_t offset = mV - temp_lut[i].mV;

      return temp_lut[i].temp_dC + (offset * delta_T) / delta_mV;
    }
  }

  return -9999; // fault
}

/* ------------------------------------------------------------------ */
/*                       MUX CONTROL                                  */
/* ------------------------------------------------------------------ */

static void MUX_Init(void) {
  HAL_GPIO_WritePin(MUX_SYNC_PORT,
                    MUX1_SYNC_PIN | MUX2_SYNC_PIN | MUX3_SYNC_PIN,
                    GPIO_PIN_SET);
}

static void MUX_Enable(uint8_t mux_number, uint8_t enable) {
  GPIO_PinState state = enable ? GPIO_PIN_RESET : GPIO_PIN_SET;

  switch (mux_number) {
  case 0:
    HAL_GPIO_WritePin(MUX_SYNC_PORT, MUX1_SYNC_PIN, state);
    break;
  case 1:
    HAL_GPIO_WritePin(MUX_SYNC_PORT, MUX2_SYNC_PIN, state);
    break;
  case 2:
    HAL_GPIO_WritePin(MUX_SYNC_PORT, MUX3_SYNC_PIN, state);
    break;
  default:
    break;
  }
}

static void MUX_SetChannel(uint8_t mux_number, uint8_t channel) {
  if (mux_number >= NUM_MULTIPLEXERS || channel >= NUM_MUX_CHANNELS)
    return;

  uint8_t spi_data = channel & 0x07u;

  MUX_Enable(mux_number, 1);
  delay_us(MUX_SYNC_DELAY_US);
  HAL_SPI_Transmit(&hspi1, &spi_data, 1, 10);
  delay_us(MUX_SYNC_DELAY_US);
  MUX_Enable(mux_number, 0);
}

/* ------------------------------------------------------------------ */
/*                      TEMPERATURE SCAN                              */
/* ------------------------------------------------------------------ */

static int16_t Calculate_Channel_Mean(uint8_t mux, uint8_t chan,
                                      TempStatistics_t *stats) {
  if (mux >= NUM_MULTIPLEXERS || chan >= NUM_MUX_CHANNELS)
    return -9999;

  uint32_t adc_sum = 0;

  HAL_GPIO_WritePin(MUX_SYNC_PORT,
                    MUX1_SYNC_PIN | MUX2_SYNC_PIN | MUX3_SYNC_PIN,
                    GPIO_PIN_SET);

  MUX_SetChannel(mux, chan);
  MUX_Enable(mux, 1);

  delay_us(MUX_SETTLE_US);

  HAL_ADC_Start(&hadc1);

  for (uint16_t i = 0; i < 500; i++) {
    HAL_ADC_PollForConversion(&hadc1, 2u);
    adc_sum += HAL_ADC_GetValue(&hadc1);
  }

  HAL_ADC_Stop(&hadc1);
  MUX_Enable(mux, 0);

  uint16_t adc_avg = adc_sum / 500u;
  uint16_t mV = ADC_to_mV(adc_avg);
  int16_t temp = mV_to_Temperature(mV);

  if (temp > -9980)
    stats->num_enabled++;

  return temp;
}

static void Calculate_Module_Mean(TempStatistics_t *stats) {
  int32_t sum = 0;
  uint16_t count = 0;

  for (uint8_t mux = 0; mux < NUM_MULTIPLEXERS; mux++) {

    for (uint8_t ch = 0; ch < mux_channel_count[mux]; ch++) {

      int16_t temp = Calculate_Channel_Mean(mux, ch, stats);

      sum += temp;
      count++;

      if (temp > stats->max_temp) {
        stats->max_temp = temp;
        stats->max_channel = ch;
      }

      if (temp < stats->min_temp) {
        stats->min_temp = temp;
        stats->min_channel = ch;
      }
    }
  }

  if (count > 0)
    stats->avg_temp = sum / count;
}

/* ------------------------------------------------------------------ */
/*                          CAN                                       */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef
CAN_SendTemperatureStatistics(TempStatistics_t *stats) {
  TxHeader.ExtId = 0x1839F380;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8u;
  TxHeader.TransmitGlobalTime = DISABLE;

  int8_t lowest = stats->min_temp / 10;
  int8_t highest = stats->max_temp / 10;
  int8_t average = stats->avg_temp / 10;

  if (lowest > 127)
    lowest = 127;
  if (lowest < -128)
    lowest = -128;
  if (highest > 127)
    highest = 127;
  if (highest < -128)
    highest = -128;
  if (average > 127)
    average = 127;
  if (average < -128)
    average = -128;

  TxData[0] = 1;
  TxData[1] = (uint8_t)lowest;
  TxData[2] = (uint8_t)highest;
  TxData[3] = (uint8_t)average;
  TxData[4] = stats->num_enabled;
  TxData[5] = stats->max_channel;
  TxData[6] = stats->min_channel;

  uint8_t checksum = 0x39 + 8u;
  for (uint8_t i = 0; i < 7; i++)
    checksum += TxData[i];

  TxData[7] = checksum;

  return HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

/* ---------------------------------------------------------------------------
 * CAN_Init_Filter()
 * Configures CAN hardware filter to accept all incoming messages.
 * ---------------------------------------------------------------------------*/
static void CAN_Init_Filter(void) {
  CAN_FilterTypeDef f = {0};
  f.FilterActivation = CAN_FILTER_ENABLE;
  f.FilterBank = 0u;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterIdHigh = 0x0000u;
  f.FilterIdLow = 0x0000u;
  f.FilterMaskIdHigh = 0x0000u;
  f.FilterMaskIdLow = 0x0000u;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.SlaveStartFilterBank = 14u;

  if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK)
    Error_Handler();
}


/* ------------------------------------------------------------------ */
/*                            MAIN                                    */
/* ------------------------------------------------------------------ */

int main(void) {
  initialise_monitor_handles();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();

  MUX_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);

  CAN_Init_Filter();
  HAL_CAN_Start(&hcan);



  TempStatistics_t stats;

  while (1) {

	printf("hello world\n");

    stats.max_temp = -32768;
    stats.min_temp = 32767;
    stats.num_enabled = 0;

//    Calculate_Module_Mean(&stats);
//    CAN_SendTemperatureStatistics(&stats);
  }
}
/* ---------------------------------------------------------------------------
 * SystemClock_Config()
 * HSI @ 8 MHz, no PLL. ADC clock = APB2 / 2 = 4 MHz.
 * ---------------------------------------------------------------------------*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2; /* 4 MHz ADC clock */
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();
}

/* ---------------------------------------------------------------------------
 * MX_ADC1_Init()
 * Channel 2 (PA2), single-conversion, software trigger, right-aligned.
 * Sampling time kept at minimum (1.5 cycles) since the op-amp filter
 * already conditions the signal before it reaches the ADC pin.
 * ---------------------------------------------------------------------------*/
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  sConfig.Channel = ADC_CHANNEL_2; /* PA2 = MUX_FILT_OUT */
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime =
      ADC_SAMPLETIME_1CYCLE_5; /* Signal pre-filtered by U302 */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

/* ---------------------------------------------------------------------------
 * MX_CAN_Init()
 * ---------------------------------------------------------------------------*/
static void MX_CAN_Init(void) {
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
    Error_Handler();
}

/* ---------------------------------------------------------------------------
 * MX_SPI1_Init()
 * PA5 = SCLK, PA7 = MOSI (DIN to ADG731 multiplexers).
 * 1-line (transmit only) since ADG731 has no MISO.
 * ---------------------------------------------------------------------------*/
static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
    Error_Handler();
}

/* ---------------------------------------------------------------------------
 * MX_GPIO_Init()
 * PB0/PB1/PB2 configured as push-pull outputs, initially HIGH (mux disabled).
 * ---------------------------------------------------------------------------*/
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Start with all SYNC pins HIGH (all muxes disabled) */
  HAL_GPIO_WritePin(MUX_SYNC_PORT,
                    MUX1_SYNC_PIN | MUX2_SYNC_PIN | MUX3_SYNC_PIN,
                    GPIO_PIN_SET);

  GPIO_InitStruct.Pin = MUX1_SYNC_PIN | MUX2_SYNC_PIN | MUX3_SYNC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUX_SYNC_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
