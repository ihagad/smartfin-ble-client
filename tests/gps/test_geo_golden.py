"""
Golden geo conversion tests: Python reference vs trajectory fixtures.

C++ parity is enforced by gps_test + geo_golden_vectors.hpp (CMake-generated).
"""

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_DIR = REPO_ROOT / "tools"

GOLDEN_DURATION_S = 12.0
GOLDEN_DT_S = 1.0
TOL_M = 1e-4
TOL_DEG = 1e-9


def load_generator_module():
    spec = importlib.util.spec_from_file_location(
        "generate_fake_gps", TOOLS_DIR / "generate_fake_gps.py"
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


gen = load_generator_module()


class TestGeoGoldenPython(unittest.TestCase):
    """Reference math used by generate_fake_gps and geo_utils.cpp."""

    def test_latlon_to_enu_matches_meters_to_latlon_inverse(self) -> None:
        east, north = 12.5, -7.25
        lat, lon = gen.meters_to_latlon(
            gen.ORIGIN_LAT, gen.ORIGIN_LON, east, north
        )
        e2, n2, u2 = gen.latlon_to_enu(
            gen.ORIGIN_LAT,
            gen.ORIGIN_LON,
            gen.ORIGIN_ALT_M,
            lat,
            lon,
            gen.ORIGIN_ALT_M + 0.5,
        )
        self.assertAlmostEqual(e2, east, places=6)
        self.assertAlmostEqual(n2, north, places=6)
        self.assertAlmostEqual(u2, 0.5, places=6)

    def test_each_trajectory_fix_matches_analytic_enu(self) -> None:
        for name in sorted(gen.TRAJECTORIES):
            with self.subTest(trajectory=name):
                fixes = gen.TRAJECTORIES[name](GOLDEN_DURATION_S, GOLDEN_DT_S)
                self.assertGreaterEqual(len(fixes), 2)
                for idx, fix in enumerate(fixes):
                    t = idx * GOLDEN_DT_S
                    east, north, up = gen.latlon_to_enu(
                        gen.ORIGIN_LAT,
                        gen.ORIGIN_LON,
                        gen.ORIGIN_ALT_M,
                        fix.lat,
                        fix.lon,
                        fix.alt_m,
                    )
                    if name == "straight_line":
                        self.assertAlmostEqual(
                            east, 1.5 * t, delta=TOL_M, msg=f"idx={idx}"
                        )
                        self.assertAlmostEqual(north, 0.0, delta=TOL_M)
                    elif name == "stationary":
                        self.assertAlmostEqual(east, 0.0, delta=TOL_M)
                        self.assertAlmostEqual(north, 0.0, delta=TOL_M)
                    elif name == "circle":
                        omega = 1.2 / 40.0
                        angle = omega * t
                        exp_e = 40.0 * math.sin(angle)
                        exp_n = 40.0 * (1.0 - math.cos(angle))
                        self.assertAlmostEqual(
                            east, exp_e, delta=TOL_M, msg=f"idx={idx}"
                        )
                        self.assertAlmostEqual(
                            north, exp_n, delta=TOL_M, msg=f"idx={idx}"
                        )
                    elif name == "paddle_pause":
                        # Monotonic north while paddling; ENU y non-decreasing.
                        if idx > 0:
                            prev = gen.latlon_to_enu(
                                gen.ORIGIN_LAT,
                                gen.ORIGIN_LON,
                                gen.ORIGIN_ALT_M,
                                fixes[idx - 1].lat,
                                fixes[idx - 1].lon,
                                fixes[idx - 1].alt_m,
                            )
                            self.assertGreaterEqual(north + TOL_M, prev[1])
                    self.assertAlmostEqual(up, 0.0, delta=TOL_M)

                    lat2, lon2 = gen.enu_to_latlon(
                        gen.ORIGIN_LAT, gen.ORIGIN_LON, east, north
                    )
                    self.assertAlmostEqual(lat2, fix.lat, delta=TOL_DEG)
                    self.assertAlmostEqual(lon2, fix.lon, delta=TOL_DEG)


if __name__ == "__main__":
    unittest.main()
