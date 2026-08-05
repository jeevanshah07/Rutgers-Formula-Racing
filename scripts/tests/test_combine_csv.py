from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest

from zoneinfo import ZoneInfo

import pandas as pd

from scripts.combine_csv import CombineError, combine_files, read_motec

BACKEND_DIR = Path(__file__).resolve().parents[2] / "daq-website" / "backend"
sys.path.insert(0, str(BACKEND_DIR))
from app.services.csv_reader import _read_motec_csv  # noqa: E402


MOTEC_METADATA = """Format,MoTeC CSV File,,,,Workbook,
Venue,Test Track,,,,Worksheet,
Log Date,6/16/2026,,,,Origin Time,0.000,s
Log Time,11:46:17 AM,,,,Start Time,0.000,s
Sample Rate,100.000,Hz,,,End Time,0.030,s

Time,Motor Speed,Battery Volts
s,rpm,V


0.000,1000,36.1
0.010,1001,36.2
0.020,1002,36.3
0.030,1003,36.4
"""


class CombineCsvTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.motec = self.directory / "standard.csv"
        self.vn = self.directory / "vectornav.csv"
        self.output = self.directory / "combined.csv"
        self.motec.write_text(MOTEC_METADATA, encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def write_vn(self, rows):
        with self.vn.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["timestamp", "yaw", "gnss_speed", "event_type"])
            writer.writerows(rows)

    def output_rows(self):
        with self.output.open(newline="", encoding="utf-8") as handle:
            return list(csv.reader(handle))

    def test_combines_nearest_samples_and_preserves_motec_shape(self):
        self.write_vn(
            [
                ["2026-06-16T11:46:17.000-04:00", "1", "2.0", ""],
                ["2026-06-16T11:46:17.020-04:00", "3", "4.0", "Cone"],
            ]
        )
        summary = combine_files(self.motec, self.vn, self.output)

        self.assertEqual(summary.output_rows, 4)
        self.assertEqual(summary.matched_rows, 4)
        rows = self.output_rows()
        header_index = next(i for i, row in enumerate(rows) if row and row[0] == "Time")
        header, units = rows[header_index], rows[header_index + 1]
        self.assertEqual(header[:3], ["Time", "Motor Speed", "Battery Volts"])
        self.assertIn("VN timestamp", header)
        self.assertIn("VN Alignment Delta (s)", header)
        self.assertEqual(units[header.index("VN yaw")], "deg")
        self.assertEqual(units[header.index("VN Alignment Delta (s)")], "s")
        data = [row for row in rows[header_index + 2 :] if row]
        self.assertEqual(len(data), 4)
        self.assertEqual(data[0][header.index("VN yaw")], "1")
        self.assertEqual(data[2][header.index("VN event_type")], "Cone")

        frame, parsed_units = _read_motec_csv(self.output)
        self.assertEqual(len(frame), 4)
        self.assertIn("VN gnss_speed", frame.columns)
        self.assertEqual(parsed_units["VN gnss_speed"], "m/s")

    def test_offset_moves_vectornav_timestamps_later(self):
        self.write_vn([["2026-06-16T11:46:16.000-04:00", "7", "1", ""]])
        summary = combine_files(
            self.motec,
            self.vn,
            self.output,
            vectornav_offset_seconds=1.0,
            tolerance_seconds=0.005,
        )
        self.assertEqual(summary.matched_rows, 1)
        self.assertAlmostEqual(summary.max_abs_delta_seconds, 0.0)

    def test_out_of_tolerance_values_are_blank(self):
        self.write_vn([["2026-06-16T11:46:17.000-04:00", "7", "1", ""]])
        summary = combine_files(
            self.motec, self.vn, self.output, tolerance_seconds=0.005
        )
        self.assertEqual(summary.matched_rows, 1)
        rows = self.output_rows()
        header_index = next(i for i, row in enumerate(rows) if row and row[0] == "Time")
        header = rows[header_index]
        data = [row for row in rows[header_index + 2 :] if row]
        self.assertEqual(data[-1][header.index("VN yaw")], "")

    def test_naive_vectornav_timestamp_uses_selected_timezone(self):
        self.write_vn([["2026-06-16 11:46:17", "7", "1", ""]])
        summary = combine_files(self.motec, self.vn, self.output)
        self.assertGreater(summary.matched_rows, 0)
        self.assertEqual(summary.vectornav_start.utcoffset().total_seconds(), 0)

    def test_rejects_no_overlap_and_does_not_create_output(self):
        self.write_vn([["2026-06-17T11:46:17-04:00", "7", "1", ""]])
        with self.assertRaisesRegex(CombineError, "no samples"):
            combine_files(self.motec, self.vn, self.output)
        self.assertFalse(self.output.exists())

    def test_rejects_existing_output(self):
        self.write_vn([["2026-06-16T11:46:17-04:00", "7", "1", ""]])
        self.output.write_text("keep", encoding="utf-8")
        with self.assertRaisesRegex(CombineError, "already exists"):
            combine_files(self.motec, self.vn, self.output)
        self.assertEqual(self.output.read_text(encoding="utf-8"), "keep")

    def test_rejects_invalid_tolerance_and_missing_timestamp(self):
        self.write_vn([["2026-06-16T11:46:17-04:00", "7", "1", ""]])
        with self.assertRaisesRegex(CombineError, "greater than zero"):
            combine_files(self.motec, self.vn, self.output, tolerance_seconds=0)

        self.vn.write_text("yaw\n1\n", encoding="utf-8")
        with self.assertRaisesRegex(CombineError, "missing the timestamp"):
            combine_files(self.motec, self.vn, self.output)

    def test_reads_quoted_motec_export(self):
        quoted = self.directory / "quoted.csv"
        # Use csv.writer to create a genuinely quoted equivalent.
        rows = list(csv.reader(MOTEC_METADATA.splitlines()))
        with quoted.open("w", newline="", encoding="utf-8") as handle:
            csv.writer(handle, quoting=csv.QUOTE_ALL).writerows(rows)
        parsed = read_motec(quoted, ZoneInfo("America/New_York"))
        self.assertEqual(len(parsed.frame), 4)


if __name__ == "__main__":
    unittest.main()
