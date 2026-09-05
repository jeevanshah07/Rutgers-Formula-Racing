# RFR27 Precharge Firmware

Bench-safe STM32F103 precharge controller written in Zig with MicroZig.

The firmware keeps PA7 low until it has a fresh, plausible BMS voltage and
three consecutive inverter-voltage readings at or above 90% of that voltage.
Missing, stale, malformed, or implausible data latches a fault and keeps PA7
low. After successful precharge, PA7 remains high until reset.

## Protocol interlock

`src/root.zig` contains `placeholder_config`. Its `ready` field intentionally
defaults to `false`, so the current firmware always latches a configuration
fault and cannot assert PA7.

Before hardware precharge testing, confirm and update all of the following:

- BMS and inverter CAN identifiers and whether each uses standard or extended frames
- CAN bus bitrate (the current bxCAN timing supports 500 kbit/s only)
- voltage field offset, width, byte order, multiplier, and divisor
- plausible minimum and maximum decoded voltages

Only then set `ready = true`. Do not enable it while any value remains a
placeholder.

## Build and test

Use the repository's pinned Zig compiler:

```sh
zig build test
zig build
```

The test step exercises decoding, frame validation, threshold qualification,
timeouts, plausibility checks, and fault latching on the host. The normal build
produces the STM32 ELF and binary images.

> This project uses *microzig* version 0.15.2 which __requires__ zig 0.17.0-dev.1257+67b05e521. See `build.zig.zon` for dependancy information.


## Bench checklist

1. With `ready = false`, reset the board and verify PA7 remains low.
2. Confirm the configured CAN bitrate and decoded voltage values using a current-limited bench setup.
3. Verify unrelated, remote, short, stale, and implausible frames cannot assert PA7.
4. Verify readings below 90% reset the qualification sequence.
5. Verify exactly three consecutive fresh readings at or above 90% assert PA7.
6. Verify reset returns PA7 low before communications initialization.
