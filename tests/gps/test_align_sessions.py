"""
Session alignment: GPS elapsed_time_ms vs fin IMU (phase B/C).
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

from _paths import FAKE_GPS_CSV, FIXTURE_SFDAT, REPO_ROOT, TOOLS_DIR
from sfdat_reader import load_quat_imu_elapsed_ms

MIN_OVERLAP_FRACTION = 0.80
T0_UNIX_MS = 1780324200 * 1000
ALIGN_TOLERANCE_MS = 50


def load_generator_module():
    spec = importlib.util.spec_from_file_location(
        "generate_fake_gps", TOOLS_DIR / "generate_fake_gps.py"
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _read_gps_rows(path: Path) -> list[dict[str, str]]:
    import csv

    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def overlap_fraction(gps_start: int, gps_end: int, imu_start: int, imu_end: int) -> float:
    overlap_start = max(gps_start, imu_start)
    overlap_end = min(gps_end, imu_end)
    overlap = max(0, overlap_end - overlap_start)
    union_start = min(gps_start, imu_start)
    union_end = max(gps_end, imu_end)
    union = max(0, union_end - union_start)
    return 1.0 if union == 0 else overlap / union


def align_gps_elapsed_ms(rows: list[dict[str, str]], t0_unix_ms: int) -> list[int]:
    out: list[int] = []
    for row in rows:
        unix_ms = round(float(row["locationTimestamp_since1970(s)"]) * 1000.0)
        elapsed = max(0, int(unix_ms - t0_unix_ms))
        out.append(elapsed)
    return out


class TestAlignSessionsPython(unittest.TestCase):
    def test_paired_fixtures_high_overlap(self) -> None:
        rows = _read_gps_rows(FAKE_GPS_CSV)
        imu_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)
        gps_ms = align_gps_elapsed_ms(rows, T0_UNIX_MS)

        gps_start, gps_end = gps_ms[0], gps_ms[-1]
        imu_start, imu_end = imu_ms[0], imu_ms[-1]

        self.assertLessEqual(abs(gps_start - imu_start), ALIGN_TOLERANCE_MS)
        self.assertGreaterEqual(gps_start, imu_start)
        self.assertLessEqual(gps_end, imu_end)

        frac = overlap_fraction(gps_start, gps_end, imu_start, imu_end)
        self.assertGreaterEqual(frac, MIN_OVERLAP_FRACTION)

    def test_gps_elapsed_maps_to_nearest_imu_sample(self) -> None:
        rows = _read_gps_rows(FAKE_GPS_CSV)
        imu_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)
        gps_ms = align_gps_elapsed_ms(rows, T0_UNIX_MS)

        for gps_elapsed in gps_ms[::10]:
            nearest = min(imu_ms, key=lambda ms: abs(ms - gps_elapsed))
            self.assertLessEqual(
                abs(nearest - gps_elapsed),
                ALIGN_TOLERANCE_MS,
                msg=f"GPS {gps_elapsed} ms vs IMU {nearest} ms",
            )

    def test_low_overlap_detected(self) -> None:
        # GPS 0–100 s, IMU 0–3 s → small intersection vs large union.
        gps_start, gps_end = 0, 100_000
        imu_start, imu_end = 0, 3_000
        frac = overlap_fraction(gps_start, gps_end, imu_start, imu_end)
        self.assertLess(frac, MIN_OVERLAP_FRACTION)


if __name__ == "__main__":
    unittest.main()
