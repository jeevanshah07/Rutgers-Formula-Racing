#!/usr/bin/env python3
"""Simulate BMS and inverter CAN traffic for the precharge controller."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import serial

DEFAULT_PORT = "/dev/cu.usbserial-DN7BORLC"
SERIAL_BAUD = 115200
CAN_SPEED = 6  # CANDapter/SLCAN speed code for 500 kbit/s

BMS_ID = 0x01
INVERTER_ID = 0x02
MAX_VOLTAGE = 390

ACK = b"\x06"
BELL = b"\x07"


@dataclass(frozen=True)
class Step:
    delay: float
    can_id: int
    data: bytes
    description: str


def voltage_data(voltage: int) -> bytes:
    """Encode voltage exactly as src/root.zig's placeholder_config expects."""
    if voltage % 10:
        raise ValueError("voltage must be a multiple of 10 V")
    raw = voltage // 10
    if not 0 <= raw <= 0xFFFF:
        raise ValueError("voltage cannot be represented in two bytes")
    return raw.to_bytes(2, "little")


def slcan_frame(can_id: int, data: bytes) -> str:
    """Build an extended (29-bit) SLCAN transmit command."""
    if not 0 <= can_id <= 0x1FFFFFFF:
        raise ValueError("extended CAN ID is out of range")
    if len(data) > 8:
        raise ValueError("CAN payload cannot exceed 8 bytes")
    return f"T{can_id:08X}{len(data):X}{data.hex().upper()}"


def send_cmd(ser: serial.Serial, command: str, delay: float = 0.1) -> bytes:
    ser.write((command + "\r").encode("ascii"))
    ser.flush()
    time.sleep(delay)
    response = ser.read(ser.in_waiting or 1)
    if response and response[-1:] == BELL:
        raise RuntimeError(f"CANDapter rejected command {command!r}")
    return response


def scenarios(interval: float, stale_delay: float) -> dict[str, list[Step]]:
    def bms(delay: float, voltage: int) -> Step:
        return Step(delay, BMS_ID, voltage_data(voltage), f"BMS {voltage} V")

    def inverter(delay: float, voltage: int) -> Step:
        return Step(
            delay,
            INVERTER_ID,
            voltage_data(voltage),
            f"inverter {voltage} V",
        )

    return {
        "success": [
            bms(0, 300),
            inverter(interval, 260),
            inverter(interval, 270),
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
        ],
        "below-threshold": [
            bms(0, 350),
            inverter(interval, 100),
            inverter(interval, 200),
            inverter(interval, 300),
            inverter(interval, 310),  # 88.6%; never reaches the 90% threshold
        ],
        "missing-bms": [
            inverter(0, 50),
            inverter(interval, 150),
            inverter(interval, 270),
        ],
        "missing-inverter": [
            bms(0, 300),
            bms(interval, 300),
            bms(interval, 300),
        ],
        "stale-bms": [
            bms(0, 300),
            inverter(stale_delay, 270),
        ],
        "invalid-bms": [
            Step(0, BMS_ID, voltage_data(0), "BMS 0 V (implausible)"),
        ],
        "invalid-inverter": [
            bms(0, 300),
            Step(
                interval,
                INVERTER_ID,
                voltage_data(MAX_VOLTAGE + 10),
                "inverter 400 V (implausible)",
            ),
        ],
        "malformed-bms": [
            Step(0, BMS_ID, b"\x1e", "BMS payload is only one byte"),
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario",
        choices=[
            "success",
            "slow-ramp",
            "below-threshold",
            "missing-bms",
            "missing-inverter",
            "stale-bms",
            "invalid-bms",
            "invalid-inverter",
            "malformed-bms",
        ],
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument(
        "--interval", type=float, default=1.0, help="seconds between normal frames"
    )
    parser.add_argument(
        "--stale-delay",
        type=float,
        default=100.1,
        help="delay used by stale-bms (firmware default is 100 seconds)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print frames without opening serial"
    )
    args = parser.parse_args()
    if args.interval < 0 or args.stale_delay < 0:
        parser.error("delays must be non-negative")
    return args


def run_steps(steps: list[Step], ser: serial.Serial | None) -> None:
    started = time.monotonic()
    for step in steps:
        if step.delay:
            time.sleep(step.delay)
        command = slcan_frame(step.can_id, step.data)
        elapsed = time.monotonic() - started
        print(
            f"{elapsed:7.2f}s  id={step.can_id:08X}, length={len(step.data)}, "
            f"message={step.data.hex(' ').upper()}  ({step.description})"
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

    with serial.Serial(args.port, SERIAL_BAUD, timeout=1) as ser:
        try:
            # Closing first makes rerunning the script safe if the adapter was open.
            send_cmd(ser, "C")
            send_cmd(ser, f"S{CAN_SPEED}")
            send_cmd(ser, "O")
            print(f"CAN open on {args.port} at 500 kbit/s")
            run_steps(steps, ser)
        finally:
            send_cmd(ser, "C")
            print("CAN channel closed")


if __name__ == "__main__":
    main()
