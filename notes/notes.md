# Powertrain Control Module (PCM) Notes
- Controls the inverter, turns the lights on the roll hoop on/off, can trigger shutdown events, etc.
- PCB is 5x5in

## Topology: 
Every componenet starts with a letter that refers to it type and the first digit corresponds to the sheet number. Second digit refers to the chip it goes to.

> Eg. R12XX is a resistor on sheet 1 that goes into chip U102

Power inputs begin at the top and it flows downwards. Signal inputs come from the left and exit on the right.

## Definitions:
- ECU: Engine control unit
- Bulkhead: special type of wire that allows for connections to pass through while being sealed (think rain/water protection)
- PWM: Pulse Width Module (fan and pump)
- RTD: Ready to drive
- BJT: Bipolar junction transistor
- Leakage: Current moving when it isn't desired

---

PCM **does not** talk to the MoTeC. In fact, it was designed with the idea of MoTeC failure in mind. Despite this, the connector we use is one that MoTeC uses because of how readily and commercially avialable is it. Also it is designed with a 'fool proof' assembly idea in mind. The PCB has some spare space for 'future proofing' in case extra components are added in the future.

---

- incremental cost of an arduino is high
- logic gates come in groups so as a result certain logic gates are unused
- arduinos can be hard to diagnose because it combines hardware with software
this is why we prefer to not use arduinos

---

## Sheet 1
- Contains power supplies as well as inputs, RTD pulse generator (replaced by arduino in 26 car), Reset pulse generator
- RTD pulse generator didn't work because of leakage through internals of 555 timer

# Week 2
- 555 times can either setup a single pulse or a repetative one
  - can change the duration between pulses
  - will run on any power source (3-24v)
  - U102 creates a singular pulse (useful for creating a single sound - car horn)
  - output pulse begins when trigger pulse goes low (towards 0v)
  - car horn works on start up due to a small capacitor that slowly charges

- reset pulse is a 556 chip which means there are two 555 timers sharing a power source (U101)
  - each 555 has unique characteristics (such as pusle length and function)
  - timer #2 flashes IMD red light
  - originally the first resistor was non-existant but this causes an issue because all the current runs into the discharge pin which breaks the timer, so it was fixed by adding in the first resistor and modifying the values of the second resistor and the capacitor

## Sheet 2
**Brake system plausability device** - determines whether there is heavy breaking in the car (for us - standing on the break) and if the power drawn from the moter (5 kilo watts) in which case the system engages which activates the brakes and deactives the motor. How it works:
- Three functions: detecting whether signal is within bounds (4.5-0.5v range), fault detection (with 500ms delay), shutdown (latch and relay)
- window comparator (manual one because pre made ones didn't fit our range) - if the inverting input drop below the non-inverting input the output is high (and vice versa)
  - stays on until the signal falls below a certain threshold - allows for variation (due to noise and such)
- inputs are all in parallel with an RC low pass filter
- reference is 5.1v to ensure that the voltage can always be brought down to 5v (with the use of a potentiometer)
- voltages dividers can be created using two resistors to divide a single voltage into two smaller ones

# Week 3
- zener diode that are reverse biased are used to generate the 5.1 reference voltage
  - 5.1 is choosen because its above the required threshold which leaves a margin to be trimmed down
- potentiometers referes to resistors that create variable voltage divide
- fault detectors work the same way as the over limit dectors - only difference is the feedback networks (fault is more sensitive)
  - brake fault triggers around 500-700psi
  - current fault triggers around 5kW
  - when both hard breaking and high current occurs, an RC circuit begans to charge (creates delay of 500ms)
- both the window comparators and fault dectors run into an OR gate so either comparator tripping will result in a shutdown event
- the fault latch gets a reset pulse of around 5seconds when the car is turned on in order to charge all the power supplies and reset all the gates/sensors

## Sheet 3
- pump pwm, pump speed is based on temperature 
- 4 different triggers with 5 states (all off is the 5th state) 
