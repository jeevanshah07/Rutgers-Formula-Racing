#!/usr/bin/env python3
"""Combine a MoTeC CSV export with a VectorNav logger CSV by wall-clock time."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import tempfile
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

import pandas as pd


DEFAULT_TIMEZONE = "America/New_York"
DEFAULT_TOLERANCE_SECONDS = 0.2

VN_UNITS = {
    "startup_ns": "ns",
    "gps_utc_ns": "ns",
    "yaw": "deg",
    "pitch": "deg",
    "roll": "deg",
    "ax": "m/s^2",
    "ay": "m/s^2",
    "az": "m/s^2",
    "gx": "rad/s",
    "gy": "rad/s",
    "gz": "rad/s",
    "dvx": "m/s",
    "dvy": "m/s",
    "dvz": "m/s",
    "dtx": "deg",
    "dty": "deg",
    "dtz": "deg",
    "dt": "s",
    "temp_c": "C",
    "pressure_pa": "Pa",
    "gnss_lat": "deg",
    "gnss_lon": "deg",
    "gnss_alt": "m",
    "gnss_vn": "m/s",
    "gnss_ve": "m/s",
    "gnss_vd": "m/s",
    "gnss_speed": "m/s",
    "gnss_pos_u_n": "m",
    "gnss_pos_u_e": "m",
    "gnss_pos_u_d": "m",
    "ins_lat": "deg",
    "ins_lon": "deg",
    "ins_alt": "m",
    "ins_vn": "m/s",
    "ins_ve": "m/s",
    "ins_vd": "m/s",
    "ins_pos_u": "m",
    "ins_vel_u": "m/s",
}


class CombineError(ValueError):
    """Raised when the inputs cannot be safely combined."""


@dataclass
class MotecData:
    metadata: list[list[str]]
    columns: list[str]
    units: list[str]
    frame: pd.DataFrame
    log_start: pd.Timestamp


@dataclass
class CombineSummary:
    standard_start: pd.Timestamp
    standard_end: pd.Timestamp
    vectornav_start: pd.Timestamp
    vectornav_end: pd.Timestamp
    output_rows: int
    matched_rows: int
    max_abs_delta_seconds: float


def _read_rows(path: Path) -> list[list[str]]:
    if not path.is_file():
        raise CombineError(f"input file does not exist: {path}")
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.reader(handle))
    if not rows:
        raise CombineError(f"input file is empty: {path}")
    return rows


def _metadata_value(rows: list[list[str]], key: str) -> str:
    for row in rows:
        if row and row[0].strip() == key and len(row) > 1 and row[1].strip():
            return row[1].strip()
    raise CombineError(f"MoTeC metadata is missing {key!r}")


def read_motec(path: Path, timezone: ZoneInfo) -> MotecData:
    rows = _read_rows(path)
    if not rows[0] or rows[0][0].strip() != "Format":
        raise CombineError("standard input is not a MoTeC CSV (missing Format row)")

    header_index = next(
        (index for index, row in enumerate(rows) if row and row[0].strip() == "Time"),
        None,
    )
    if header_index is None:
        raise CombineError("MoTeC CSV has no Time header row")
    if header_index + 1 >= len(rows):
        raise CombineError("MoTeC CSV has no units row")

    columns = [value.strip() for value in rows[header_index]]
    if not columns or any(not column for column in columns):
        raise CombineError("MoTeC CSV contains an empty column name")
    if len(set(columns)) != len(columns):
        raise CombineError("MoTeC CSV contains duplicate column names")
    units = (rows[header_index + 1] + [""] * len(columns))[: len(columns)]

    data_rows: list[list[str]] = []
    elapsed: list[float] = []
    for row in rows[header_index + 2 :]:
        if not row or not any(value.strip() for value in row):
            continue
        try:
            time_value = float(row[0])
        except (ValueError, IndexError):
            continue
        data_rows.append((row + [""] * len(columns))[: len(columns)])
        elapsed.append(time_value)
    if not data_rows:
        raise CombineError("MoTeC CSV contains no numeric data rows")
    if any(later < earlier for earlier, later in zip(elapsed, elapsed[1:])):
        raise CombineError("MoTeC Time values must be in ascending order")

    date_value = _metadata_value(rows[:header_index], "Log Date")
    time_value = _metadata_value(rows[:header_index], "Log Time")
    try:
        log_start = pd.Timestamp(f"{date_value} {time_value}").tz_localize(timezone)
    except (ValueError, TypeError) as exc:
        raise CombineError(f"invalid MoTeC Log Date/Log Time: {exc}") from exc

    frame = pd.DataFrame(data_rows, columns=columns)
    frame["_standard_time"] = (
        log_start.tz_convert("UTC") + pd.to_timedelta(elapsed, unit="s")
    ).astype("datetime64[ns, UTC]")
    return MotecData(rows[:header_index], columns, units, frame, log_start)


def _parse_vectornav_time(value: object, timezone: ZoneInfo) -> pd.Timestamp:
    try:
        parsed = pd.Timestamp(value)
        if pd.isna(parsed):
            raise ValueError("empty timestamp")
        if parsed.tzinfo is None:
            parsed = parsed.tz_localize(timezone)
        return parsed.tz_convert("UTC")
    except (ValueError, TypeError) as exc:
        raise CombineError(f"invalid VectorNav timestamp {value!r}: {exc}") from exc


def read_vectornav(
    path: Path, timezone: ZoneInfo, offset_seconds: float
) -> tuple[pd.DataFrame, list[str]]:
    rows = _read_rows(path)
    columns = [value.strip() for value in rows[0]]
    if "timestamp" not in columns:
        raise CombineError("VectorNav CSV is missing the timestamp column")
    if any(not column for column in columns) or len(set(columns)) != len(columns):
        raise CombineError("VectorNav CSV contains empty or duplicate column names")

    try:
        frame = pd.read_csv(path, dtype=str, keep_default_na=False)
    except Exception as exc:
        raise CombineError(f"unable to parse VectorNav CSV: {exc}") from exc
    if frame.empty:
        raise CombineError("VectorNav CSV contains no data rows")

    parsed = [_parse_vectornav_time(value, timezone) for value in frame["timestamp"]]
    frame["_vn_merge_time"] = pd.DatetimeIndex(parsed) + pd.to_timedelta(
        offset_seconds, unit="s"
    )
    frame["_vn_merge_time"] = frame["_vn_merge_time"].astype("datetime64[ns, UTC]")
    frame = frame.sort_values("_vn_merge_time", kind="stable").reset_index(drop=True)
    renamed = {column: f"VN {column}" for column in columns}
    return frame.rename(columns=renamed), columns


def combine_frames(
    motec: MotecData,
    vectornav: pd.DataFrame,
    vectornav_columns: list[str],
    tolerance_seconds: float,
) -> tuple[pd.DataFrame, CombineSummary]:
    if tolerance_seconds <= 0:
        raise CombineError("tolerance must be greater than zero")

    merged = pd.merge_asof(
        motec.frame,
        vectornav,
        left_on="_standard_time",
        right_on="_vn_merge_time",
        direction="nearest",
        tolerance=pd.Timedelta(seconds=tolerance_seconds),
    )
    matched = merged["_vn_merge_time"].notna()
    matched_count = int(matched.sum())
    if matched_count == 0:
        raise CombineError(
            "the recordings have no samples within the alignment tolerance; "
            "check their clocks or use --vectornav-offset-seconds"
        )

    deltas = (merged["_vn_merge_time"] - merged["_standard_time"]).dt.total_seconds()
    merged["VN Alignment Delta (s)"] = deltas
    output_columns = [
        *motec.columns,
        *(f"VN {column}" for column in vectornav_columns),
        "VN Alignment Delta (s)",
    ]
    summary = CombineSummary(
        standard_start=motec.frame["_standard_time"].iloc[0],
        standard_end=motec.frame["_standard_time"].iloc[-1],
        vectornav_start=vectornav["_vn_merge_time"].iloc[0],
        vectornav_end=vectornav["_vn_merge_time"].iloc[-1],
        output_rows=len(merged),
        matched_rows=matched_count,
        max_abs_delta_seconds=float(deltas[matched].abs().max()),
    )
    return merged[output_columns], summary


def _csv_value(value: object) -> object:
    if pd.isna(value):
        return ""
    if isinstance(value, pd.Timestamp):
        return value.isoformat()
    return value


def write_motec_output(
    output: Path,
    motec: MotecData,
    combined: pd.DataFrame,
    vectornav_columns: list[str],
) -> None:
    if output.exists():
        raise CombineError(f"output file already exists: {output}")
    if not output.parent.is_dir():
        raise CombineError(f"output directory does not exist: {output.parent}")

    units = [
        *motec.units,
        *(VN_UNITS.get(column, "") for column in vectornav_columns),
        "s",
    ]
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            newline="",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_name = handle.name
            writer = csv.writer(handle)
            writer.writerows(motec.metadata)
            writer.writerow(combined.columns)
            writer.writerow(units)
            writer.writerow([])
            writer.writerow([])
            for row in combined.itertuples(index=False, name=None):
                writer.writerow(_csv_value(value) for value in row)
        os.replace(temporary_name, output)
    except Exception:
        if temporary_name:
            Path(temporary_name).unlink(missing_ok=True)
        raise


def combine_files(
    standard_path: Path,
    vectornav_path: Path,
    output_path: Path,
    *,
    timezone_name: str = DEFAULT_TIMEZONE,
    vectornav_offset_seconds: float = 0.0,
    tolerance_seconds: float = DEFAULT_TOLERANCE_SECONDS,
) -> CombineSummary:
    if output_path.exists():
        raise CombineError(f"output file already exists: {output_path}")
    try:
        timezone = ZoneInfo(timezone_name)
    except ZoneInfoNotFoundError as exc:
        raise CombineError(f"unknown timezone: {timezone_name}") from exc
    if tolerance_seconds <= 0:
        raise CombineError("tolerance must be greater than zero")

    motec = read_motec(standard_path, timezone)
    vectornav, vectornav_columns = read_vectornav(
        vectornav_path, timezone, vectornav_offset_seconds
    )
    combined, summary = combine_frames(
        motec, vectornav, vectornav_columns, tolerance_seconds
    )
    write_motec_output(output_path, motec, combined, vectornav_columns)
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Combine MoTeC and VectorNav CSV files using nearest timestamps."
    )
    parser.add_argument("standard_csv", type=Path, help="MoTeC-format CSV")
    parser.add_argument("vectornav_csv", type=Path, help="VectorNav logger CSV")
    parser.add_argument("--output", "-o", required=True, type=Path)
    parser.add_argument("--timezone", default=DEFAULT_TIMEZONE)
    parser.add_argument(
        "--vectornav-offset-seconds",
        type=float,
        default=0.0,
        help="seconds added to VectorNav timestamps (positive moves them later)",
    )
    parser.add_argument(
        "--tolerance-seconds",
        type=float,
        default=DEFAULT_TOLERANCE_SECONDS,
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        summary = combine_files(
            args.standard_csv,
            args.vectornav_csv,
            args.output,
            timezone_name=args.timezone,
            vectornav_offset_seconds=args.vectornav_offset_seconds,
            tolerance_seconds=args.tolerance_seconds,
        )
    except CombineError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    unmatched = summary.output_rows - summary.matched_rows
    percentage = 100 * summary.matched_rows / summary.output_rows
    print(f"Wrote {args.output}")
    print(f"MoTeC range:    {summary.standard_start.isoformat()} to {summary.standard_end.isoformat()}")
    print(f"VectorNav range: {summary.vectornav_start.isoformat()} to {summary.vectornav_end.isoformat()}")
    print(
        f"Rows: {summary.output_rows:,}; matched: {summary.matched_rows:,} "
        f"({percentage:.1f}%); unmatched: {unmatched:,}"
    )
    print(f"Maximum absolute alignment delta: {summary.max_abs_delta_seconds:.6f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
