#!/usr/bin/env python3
"""Simulate BMS and inverter CAN traffic for the precharge controller."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import serial


DEFAULT_PORT = "/dev/cu.usbserial-DN8FHRI7"
SERIAL_BAUD = 115200
CAN_SPEED = 6  # CANDapter/SLCAN speed code for 500 kbit/s

BMS_ID = 0x6B1
INVERTER_ID = 0x0A7
MAX_VOLTAGE = 390
FRESHNESS_TIMEOUT_SECONDS = 30.0

BELL = b"\x07"


@dataclass(frozen=True)
class Step:
    delay: float
    can_id: int | None
    extended: bool = False
    data: bytes = b""
    description: str = ""


def bms_data(voltage: int, *, on_cart: bool = False) -> bytes:
    """Encode 1 V/bit BMS voltage at bytes 2-3 and cart flag at byte 5."""
    data = bytearray(6)
    data[2:4] = voltage.to_bytes(2, "big")
    data[5] = 0 if on_cart else 1
    return bytes(data)


def inverter_data(voltage: int) -> bytes:
    """Encode 0.1 V/bit inverter voltage at bytes 0-1, little-endian."""
    return (voltage * 10).to_bytes(2, "little")


def candapter_frame(can_id: int, extended: bool, data: bytes) -> str:
    """Build a standard or extended CANdapter transmit command."""
    maximum_id = 0x1FFFFFFF if extended else 0x7FF
    if not 0 <= can_id <= maximum_id:
        raise ValueError("CAN ID is out of range for its frame type")
    if len(data) > 8:
        raise ValueError("CAN payload cannot exceed 8 bytes")
    prefix = "X" if extended else "T"
    identifier = f"{can_id:08X}" if extended else f"{can_id:03X}"
    return f"{prefix}{identifier}{len(data):X}{data.hex().upper()}"


def send_cmd(ser: Any, command: str, delay: float = 0.1) -> bytes:
    ser.write((command + "\r").encode("ascii"))
    ser.flush()
    time.sleep(delay)
    response = ser.read(ser.in_waiting or 1)
    if response and response[-1:] == BELL:
        raise RuntimeError(f"CANDapter rejected command {command!r}")
    return response


def scenarios(interval: float, stale_delay: float) -> dict[str, list[Step]]:
    def bms(delay: float, voltage: int, *, on_cart: bool = False) -> Step:
        location = "on cart" if on_cart else "off cart"
        return Step(
            delay,
            BMS_ID,
            False,
            bms_data(voltage, on_cart=on_cart),
            f"BMS {voltage} V, {location}",
        )

    def inverter(delay: float, voltage: int) -> Step:
        return Step(
            delay,
            INVERTER_ID,
            True,
            inverter_data(voltage),
            f"inverter {voltage} V",
        )

    def wait(delay: float, description: str) -> Step:
        return Step(delay, None, description=description)

    return {
        "success": [
            bms(0, 300),
            inverter(interval, 260),
            inverter(interval, 270),
            inverter(interval, 280),
            inverter(interval, 290),
            inverter(interval, 300),
        ],
        "cart-bypass": [
            bms(0, 300, on_cart=True),
        ],
        "slow-ramp": [
            bms(0, 350),
            inverter(interval, 50),
            inverter(interval, 100),
            inverter(interval, 150),
            inverter(interval, 200),
            inverter(interval, 250),
            inverter(interval, 300),
            inverter(interval, 320),
            inverter(interval, 320),
            inverter(interval, 320),
        ],
        "below-threshold": [
            bms(0, 350),
            inverter(interval, 100),
            inverter(interval, 200),
            inverter(interval, 300),
            inverter(interval, 310),  # 88.6%; below the 90% threshold
        ],
        "missing-bms": [
            inverter(0, 50),
            inverter(interval, 150),
            inverter(interval, 270),
            wait(stale_delay, "allow waiting-for-BMS freshness fault"),
        ],
        "missing-inverter": [
            bms(0, 300),
            bms(interval, 300),
            bms(interval, 300),
            bms(interval, 300),
            bms(interval, 300),
            bms(interval, 300),
            wait(stale_delay, "allow missing-inverter freshness fault"),
        ],
        "stale-bms": [
            bms(0, 300),
            inverter(stale_delay, 270),
        ],
        "invalid-bms": [
            bms(0, 0),
        ],
        "invalid-inverter": [
            bms(0, 300),
            inverter(interval, MAX_VOLTAGE + 10),
        ],
        "malformed-bms": [
            Step(0, BMS_ID, False, b"\x00\x00\x01\x2c\x00", "BMS DLC 5; cart byte missing"),
        ],
        "malformed-inverter": [
            bms(0, 300),
            Step(interval, INVERTER_ID, True, b"\x1e", "inverter DLC 1; voltage incomplete"),
        ],
    }


SCENARIO_NAMES = (
    "success",
    "cart-bypass",
    "slow-ramp",
    "below-threshold",
    "missing-bms",
    "missing-inverter",
    "stale-bms",
    "invalid-bms",
    "invalid-inverter",
    "malformed-bms",
    "malformed-inverter",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=SCENARIO_NAMES)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument(
        "--interval", type=float, default=0.5, help="seconds between normal frames"
    )
    parser.add_argument(
        "--stale-delay",
        type=float,
        default=FRESHNESS_TIMEOUT_SECONDS + 0.1,
        help="delay used for stale-data scenarios (firmware default is 30 seconds)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print frames without opening serial"
    )
    args = parser.parse_args()
    if args.interval < 0 or args.stale_delay < 0:
        parser.error("delays must be non-negative")
    return args


def run_steps(steps: list[Step], ser: Any | None) -> None:
    started = time.monotonic()
    for step in steps:
        if step.delay:
            time.sleep(step.delay)
        elapsed = time.monotonic() - started
        if step.can_id is None:
            print(f"{elapsed:7.2f}s  wait complete ({step.description})")
            continue

        command = candapter_frame(step.can_id, step.extended, step.data)
        frame_type = "extended" if step.extended else "standard"
        print(
            f"{elapsed:7.2f}s  id={step.can_id:X}, length={len(step.data)}, "
            f"message={step.data.hex(' ').upper()}  "
            f"({frame_type}; {step.description}; CANdapter={command})"
        )
        if ser is not None:
            send_cmd(ser, command)


def main() -> None:
    args = parse_args()
    steps = scenarios(args.interval, args.stale_delay)[args.scenario]

    print(f"Scenario: {args.scenario}")
    if args.dry_run:
        run_steps(steps, None)
        return

    import serial

    with serial.Serial(args.port, SERIAL_BAUD, timeout=1) as ser:
        channel_open = False
        try:
            send_cmd(ser, f"S{CAN_SPEED}")
            send_cmd(ser, "O")
            channel_open = True
            print(f"CAN open on {args.port} at 500 kbit/s")
            run_steps(steps, ser)
        finally:
            if channel_open:
                try:
                    send_cmd(ser, "C")
                    print("CAN channel closed")
                except RuntimeError as error:
                    print(f"Warning: {error}", file=sys.stderr)


if __name__ == "__main__":
    main()
