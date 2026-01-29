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