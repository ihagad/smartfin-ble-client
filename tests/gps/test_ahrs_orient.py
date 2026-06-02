"""
Phase D — load_ride → orient_ride expectations on synthetic .sfdat fixture.
"""

from __future__ import annotations

import unittest

from _paths import FIXTURE_SFDAT
from sfdat_reader import load_quat_imu_elapsed_ms

FIXTURE_DURATION_S = 180.0
NOMINAL_IMU_HZ = 55.0


class TestAhrsOrientFixture(unittest.TestCase):
    def test_quat_imu_count_matches_nominal_rate(self) -> None:
        elapsed_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)
        expected = int(FIXTURE_DURATION_S * NOMINAL_IMU_HZ) + 1
        self.assertEqual(len(elapsed_ms), expected)

    def test_quat_imu_median_interval_near_55_hz(self) -> None:
        elapsed_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)
        dts = [
            (elapsed_ms[i] - elapsed_ms[i - 1]) / 1000.0
            for i in range(1, len(elapsed_ms))
            if elapsed_ms[i] > elapsed_ms[i - 1]
        ]
        self.assertGreater(len(dts), 100)
        dts.sort()
        median_dt = dts[len(dts) // 2]
        hz = 1.0 / median_dt
        self.assertAlmostEqual(hz, NOMINAL_IMU_HZ, delta=1.5)

    def test_elapsed_time_monotonic(self) -> None:
        elapsed_ms = load_quat_imu_elapsed_ms(FIXTURE_SFDAT)
        for i in range(1, len(elapsed_ms)):
            self.assertGreaterEqual(elapsed_ms[i], elapsed_ms[i - 1])


if __name__ == "__main__":
    unittest.main()
