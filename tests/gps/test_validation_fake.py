"""
Phase G — fake-data validation checklist.

  - Parser row count on committed fake_gps.csv
  - ENU ↔ lat/lon round-trip (Python reference == C++ geo_utils contract)
  - fake GPS + synthetic .sfdat IMU → non-empty fused trajectory
  - Fused horizontal position error vs GPS (small on paddle_pause fixture)
"""

from __future__ import annotations

import csv
import importlib.util
import subprocess
import sys
import unittest
from pathlib import Path

from _paths import FAKE_GPS_CSV, FIXTURE_SFDAT, REPO_ROOT, TOOLS_DIR

T0_UNIX = 1780324200
# paddle_pause default: 180 s @ 1 Hz → 181 fixes (inclusive endpoints).
EXPECTED_FAKE_GPS_ROWS = 181
BUILD_DIRS = [REPO_ROOT / "build", REPO_ROOT / "cmake-build-debug"]


def _load_generator():
    path = TOOLS_DIR / "generate_fake_gps.py"
    spec = importlib.util.spec_from_file_location("generate_fake_gps", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _find_fusion_export() -> Path:
    for d in BUILD_DIRS:
        p = d / "fusion_export"
        if p.exists():
            return p
    raise unittest.SkipTest("fusion_export not built")


class TestValidationFake(unittest.TestCase):
    def test_parser_reads_fake_csv_row_count(self) -> None:
        with FAKE_GPS_CSV.open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), EXPECTED_FAKE_GPS_ROWS)

    def test_enu_round_trip_lat_lon(self) -> None:
        gen = _load_generator()
        lat0, lon0, alt0 = gen.ORIGIN_LAT, gen.ORIGIN_LON, gen.ORIGIN_ALT_M
        lat, lon, alt = lat0 + 1e-5, lon0 - 2e-5, alt0 + 0.5
        east, north, up = gen.latlon_to_enu(lat0, lon0, alt0, lat, lon, alt)
        lat2, lon2 = gen.enu_to_latlon(lat0, lon0, east, north)
        self.assertAlmostEqual(lat, lat2, places=9)
        self.assertAlmostEqual(lon, lon2, places=9)
        self.assertAlmostEqual(alt, alt0 + up, places=6)

    def test_fake_gps_and_synthetic_imu_nonempty_trajectory(self) -> None:
        exe = _find_fusion_export()
        out = REPO_ROOT / "build" / "test_validation_g_csv"
        out.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                str(exe),
                str(FIXTURE_SFDAT),
                str(FAKE_GPS_CSV),
                "--t0",
                str(T0_UNIX),
                "--out",
                str(out),
            ],
            check=True,
        )
        fused_path = out / "trajectory_fused.csv"
        self.assertTrue(fused_path.is_file())
        with fused_path.open(newline="", encoding="utf-8") as f:
            fused_rows = list(csv.DictReader(f))
        self.assertGreater(len(fused_rows), 0)
        self.assertGreater(len(fused_rows), 1000)

    def test_fused_position_error_small_vs_gps(self) -> None:
        exe = _find_fusion_export()
        out = REPO_ROOT / "build" / "test_validation_g_csv"
        if not (out / "trajectory_fused.csv").is_file():
            out.mkdir(parents=True, exist_ok=True)
            subprocess.run(
                [
                    str(exe),
                    str(FIXTURE_SFDAT),
                    str(FAKE_GPS_CSV),
                    "--t0",
                    str(T0_UNIX),
                    "--out",
                    str(out),
                ],
                check=True,
            )

        with (out / "trajectory_gps.csv").open(newline="", encoding="utf-8") as f:
            gps = list(csv.DictReader(f))
        with (out / "trajectory_fused.csv").open(newline="", encoding="utf-8") as f:
            fused = list(csv.DictReader(f))

        errors: list[float] = []
        for g in gps:
            t_g = float(g["t_elapsed_s"])
            nearest = min(
                fused,
                key=lambda r: abs(float(r["t_elapsed_s"]) - t_g),
            )
            if abs(float(nearest["t_elapsed_s"]) - t_g) > 0.1:
                continue
            de = float(nearest["east_m"]) - float(g["east_m"])
            dn = float(nearest["north_m"]) - float(g["north_m"])
            errors.append((de * de + dn * dn) ** 0.5)

        self.assertGreater(len(errors), 20)
        self.assertLess(max(errors), 6.0)
        self.assertLess(sum(errors) / len(errors), 2.0)


if __name__ == "__main__":
    unittest.main()
