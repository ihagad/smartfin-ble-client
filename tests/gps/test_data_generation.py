"""
Phase A — fake GPS / time-align fixture tests.

Covers:
  - SensorLog-style CSV schema
  - tools/generate_fake_gps.py trajectories
  - fixed T0 documented next to paired .sfdat
  - unprocessed/fake_gps.csv (~1 Hz, 1–5 min)
  - unprocessed/ride_*.sfdat in the same nominal session window
"""

from __future__ import annotations

import csv
import importlib.util
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from _paths import (
    FAKE_GPS_CSV,
    FAKE_SESSION_README,
    FIXTURE_SFDAT,
    REPO_ROOT,
    SENSORLOG_SCHEMA_DOC,
    TOOLS_DIR,
)
from sfdat_reader import load_quat_imu_elapsed_ms, read_header


def load_generator_module():
    path = TOOLS_DIR / "generate_fake_gps.py"
    spec = importlib.util.spec_from_file_location("generate_fake_gps", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def load_fixture_config_module():
    path = TOOLS_DIR / "fixture_config.py"
    spec = importlib.util.spec_from_file_location("fixture_config", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


gen = load_generator_module()

MIN_DURATION_S = 60.0
MAX_DURATION_S = 300.0
NOMINAL_RATE_HZ = 1.0
IMU_HZ = 55.0
T0_TOLERANCE_S = 0.001
ALIGN_TOLERANCE_MS = 50


class TestSensorLogSchema(unittest.TestCase):
    def test_gps_columns_listed_in_schema_doc(self) -> None:
        doc = SENSORLOG_SCHEMA_DOC.read_text(encoding="utf-8")
        for col in gen.GPS_COLUMNS:
            with self.subTest(column=col):
                self.assertIn(f"`{col}`", doc)

    def test_schema_doc_example_header_matches_generator(self) -> None:
        doc = SENSORLOG_SCHEMA_DOC.read_text(encoding="utf-8")
        header = ",".join(gen.GPS_COLUMNS)
        self.assertIn(header, doc)


class TestCommittedFixtures(unittest.TestCase):
    def test_fake_gps_csv_exists_with_sensorlog_header(self) -> None:
        self.assertTrue(FAKE_GPS_CSV.is_file(), "run: python3 tools/generate_fake_gps.py")
        with FAKE_GPS_CSV.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            self.assertEqual(reader.fieldnames, gen.GPS_COLUMNS)

    def test_fake_gps_csv_rate_and_duration_in_range(self) -> None:
        rows = _read_gps_rows(FAKE_GPS_CSV)
        self.assertGreaterEqual(len(rows), int(MIN_DURATION_S * NOMINAL_RATE_HZ) + 1)
        self.assertLessEqual(len(rows), int(MAX_DURATION_S * NOMINAL_RATE_HZ) + 1)

        duration_s = _gps_duration_s(rows)
        self.assertGreaterEqual(duration_s, MIN_DURATION_S)
        self.assertLessEqual(duration_s, MAX_DURATION_S)

        dt = _median_gps_dt_s(rows)
        self.assertAlmostEqual(dt, 1.0 / NOMINAL_RATE_HZ, delta=0.05)

    def test_fake_gps_timestamps_anchor_at_fixed_t0(self) -> None:
        rows = _read_gps_rows(FAKE_GPS_CSV)
        first_ts = float(rows[0]["locationTimestamp_since1970(s)"])
        self.assertAlmostEqual(first_ts, gen.FAKE_SESSION_T0_UNIX, delta=T0_TOLERANCE_S)

        for row in rows:
            ts = float(row["locationTimestamp_since1970(s)"])
            elapsed = ts - gen.FAKE_SESSION_T0_UNIX
            self.assertGreaterEqual(elapsed, -T0_TOLERANCE_S)
            self.assertLessEqual(elapsed, MAX_DURATION_S + 1.0)

    def test_readme_documents_t0_next_to_fixture_sfdat(self) -> None:
        text = FAKE_SESSION_README.read_text(encoding="utf-8")
        self.assertIn(str(gen.FAKE_SESSION_T0_UNIX), text)
        self.assertIn(FIXTURE_SFDAT.name, text)
        self.assertRegex(text, r"T0.*Unix", re.IGNORECASE)

    def test_fixture_sfdat_exists_with_valid_header(self) -> None:
        self.assertTrue(FIXTURE_SFDAT.is_file(), "run: python3 tools/generate_fake_gps.py")
        hdr = read_header(FIXTURE_SFDAT.read_bytes())
        self.assertEqual(hdr.magic, gen.RIDE_FILE_MAGIC)
        self.assertEqual(hdr.version, gen.RIDE_FILE_VERSION)

    def test_fixture_sfdat_covers_same_session_window_as_gps(self) -> None:
        rows = _read_gps_rows(FAKE_GPS_CSV)
        gps_duration_s = _gps_duration_s(rows)
        elapsed_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)

        self.assertGreater(len(elapsed_ms), 0)
        self.assertAlmostEqual(elapsed_ms[0], 0, delta=ALIGN_TOLERANCE_MS)
        self.assertAlmostEqual(
            elapsed_ms[-1] / 1000.0, gps_duration_s, delta=1.0 / IMU_HZ + 0.05
        )

    def test_time_alignment_gps_elapsed_maps_to_sfdat(self) -> None:
        rows = _read_gps_rows(FAKE_GPS_CSV)
        elapsed_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)

        for row in rows[::10]:
            ts = float(row["locationTimestamp_since1970(s)"])
            gps_elapsed_ms = int(round((ts - gen.FAKE_SESSION_T0_UNIX) * 1000.0))
            nearest = min(elapsed_ms, key=lambda ms: abs(ms - gps_elapsed_ms))
            self.assertLessEqual(
                abs(nearest - gps_elapsed_ms),
                ALIGN_TOLERANCE_MS,
                msg=f"GPS t={gps_elapsed_ms} ms, nearest IMU={nearest} ms",
            )


class TestGenerateFakeGpsTool(unittest.TestCase):
    def test_fixture_config_reads_trajectory_from_env(self) -> None:
        fc = load_fixture_config_module()
        with _temporary_env(SMARTFIN_TRAJECTORY="circle", SMARTFIN_FIXTURE_SPEED="2.0"):
            cfg = fc.FixtureConfig.from_env()
            self.assertEqual(cfg.trajectory, "circle")
            self.assertAlmostEqual(cfg.speed_mps, 2.0)

    def test_imu_error_demo_enables_drift_knobs(self) -> None:
        fc = load_fixture_config_module()
        with _temporary_env(SMARTFIN_IMU_ERROR_DEMO="1"):
            cfg = fc.FixtureConfig.from_env()
            self.assertTrue(cfg.imu_error_enabled())
            self.assertAlmostEqual(cfg.imu_yaw_drift_dps, 1.0)
            self.assertAlmostEqual(cfg.imu_accel_bias_x_mps2, 0.015)

    def test_imu_yaw_drift_changes_stored_quaternion(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "drift.sfdat"
            _run_generator(
                "--trajectory",
                "circle",
                "--duration",
                "10",
                "--output",
                str(Path(tmp) / "gps.csv"),
                "--sfdat",
                str(out),
                "--imu-yaw-drift-dps",
                "2.0",
            )
            elapsed, quats = _read_first_last_quat(out)
            self.assertEqual(elapsed[0], 0)
            self.assertGreater(elapsed[-1], 0)
            self.assertAlmostEqual(quats[0][0], 1.0, places=4)
            self.assertLess(abs(quats[-1][0]), 0.999)

    def test_cli_generates_csv_and_sfdat(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_out = tmp_path / "out.csv"
            sfdat_out = tmp_path / "out.sfdat"
            _run_generator(
                "--trajectory",
                "paddle_pause",
                "--duration",
                "120",
                "--output",
                str(csv_out),
                "--sfdat",
                str(sfdat_out),
            )
            self.assertTrue(csv_out.is_file())
            self.assertTrue(sfdat_out.is_file())
            rows = _read_gps_rows(csv_out)
            self.assertEqual(len(rows), 121)
            self.assertAlmostEqual(_gps_duration_s(rows), 120.0, delta=0.01)

    def test_all_trajectories_produce_valid_csv(self) -> None:
        for name in gen.TRAJECTORIES:
            with self.subTest(trajectory=name):
                with tempfile.TemporaryDirectory() as tmp:
                    out = Path(tmp) / f"{name}.csv"
                    _run_generator(
                        "--trajectory",
                        name,
                        "--duration",
                        "60",
                        "--output",
                        str(out),
                        "--no-sfdat",
                    )
                    rows = _read_gps_rows(out)
                    self.assertEqual(len(rows), 61)
                    self._assert_rows_well_formed(rows)

    def test_straight_line_increases_longitude(self) -> None:
        fixes = gen.trajectory_straight_line(30.0, 1.0)
        self.assertGreater(fixes[-1].lon, fixes[0].lon)
        self.assertAlmostEqual(fixes[0].lat, fixes[-1].lat, places=5)

    def test_stationary_holds_position(self) -> None:
        fixes = gen.trajectory_stationary(10.0, 1.0)
        lats = {round(f.lat, 6) for f in fixes}
        lons = {round(f.lon, 6) for f in fixes}
        self.assertEqual(len(lats), 1)
        self.assertEqual(len(lons), 1)

    def test_paddle_pause_includes_moving_and_stopped_segments(self) -> None:
        fixes = gen.trajectory_paddle_pause(90.0, 1.0)
        speeds = [f.speed_mps for f in fixes]
        self.assertIn(1.5, speeds)
        self.assertIn(0.0, speeds)

    def test_circle_displaces_from_origin(self) -> None:
        fixes = gen.trajectory_circle(120.0, 1.0)
        moved = any(
            abs(f.lat - gen.ORIGIN_LAT) > 1e-6 or abs(f.lon - gen.ORIGIN_LON) > 1e-6
            for f in fixes
        )
        self.assertTrue(moved)

    def _assert_rows_well_formed(self, rows: list[dict[str, str]]) -> None:
        for row in rows:
            self.assertEqual(set(row.keys()), set(gen.GPS_COLUMNS))
            lat = float(row["locationLatitude(WGS84)"])
            lon = float(row["locationLongitude(WGS84)"])
            self.assertGreaterEqual(lat, -90.0)
            self.assertLessEqual(lat, 90.0)
            self.assertGreaterEqual(lon, -180.0)
            self.assertLessEqual(lon, 180.0)


def _read_gps_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _gps_duration_s(rows: list[dict[str, str]]) -> float:
    first = float(rows[0]["locationTimestamp_since1970(s)"])
    last = float(rows[-1]["locationTimestamp_since1970(s)"])
    return last - first


def _median_gps_dt_s(rows: list[dict[str, str]]) -> float:
    stamps = [float(r["locationTimestamp_since1970(s)"]) for r in rows]
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    dts.sort()
    return dts[len(dts) // 2]


def _read_first_last_quat(path: Path) -> tuple[list[int], list[tuple[float, float, float, float]]]:
    import struct

    data = path.read_bytes()
    offset = struct.calcsize("<QH4B")
    quat_fmt = "<I3f3f3f4ffB"
    quat_size = struct.calcsize(quat_fmt)
    elapsed: list[int] = []
    quats: list[tuple[float, float, float, float]] = []
    while offset + 1 + quat_size <= len(data):
        tag = data[offset]
        offset += 1
        if tag != gen.RECORD_TAG_QUAT_IMU:
            break
        (
            elapsed_ms,
            _ax,
            _ay,
            _az,
            _gx,
            _gy,
            _gz,
            _mx,
            _my,
            _mz,
            qw,
            qx,
            qy,
            qz,
            _acc,
            _valid,
        ) = struct.unpack(quat_fmt, data[offset : offset + quat_size])
        offset += quat_size
        elapsed.append(elapsed_ms)
        quats.append((qw, qx, qy, qz))
    return elapsed, quats


def _run_generator(*args: str) -> subprocess.CompletedProcess[str]:
    script = TOOLS_DIR / "generate_fake_gps.py"
    proc = subprocess.run(
        [sys.executable, str(script), *args],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return proc


class _temporary_env:
    def __init__(self, **values: str) -> None:
        self._values = values
        self._previous: dict[str, str | None] = {}

    def __enter__(self) -> None:
        import os

        for key, value in self._values.items():
            self._previous[key] = os.environ.get(key)
            os.environ[key] = value

    def __exit__(self, exc_type, exc, tb) -> None:
        import os

        for key, previous in self._previous.items():
            if previous is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = previous


if __name__ == "__main__":
    unittest.main()
