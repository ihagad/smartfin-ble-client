"""
Phase F — trajectory export sanity checks on fake paddle_pause fixtures.
"""

from __future__ import annotations

import csv
import subprocess
import unittest
from pathlib import Path

from _paths import FAKE_GPS_CSV, FIXTURE_SFDAT, REPO_ROOT, TOOLS_DIR

T0_UNIX = 1780324200
BUILD_DIRS = [REPO_ROOT / "build", REPO_ROOT / "cmake-build-debug"]


def find_fusion_export() -> Path:
    for d in BUILD_DIRS:
        p = d / "fusion_export"
        if p.exists():
            return p
    raise unittest.SkipTest("fusion_export not built")


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


class TestFusionTrajectoryExport(unittest.TestCase):
    def test_fused_track_near_fake_gps(self) -> None:
        exe = find_fusion_export()
        out = REPO_ROOT / "build" / "test_fusion_csv_out"
        out.mkdir(parents=True, exist_ok=True)

        subprocess.run(
            [str(exe), str(FIXTURE_SFDAT), str(FAKE_GPS_CSV),
             "--t0", str(T0_UNIX), "--out", str(out)],
            check=True,
        )

        gps = load_csv(out / "trajectory_gps.csv")
        fused = load_csv(out / "trajectory_fused.csv")
        self.assertGreater(len(fused), 1000)
        self.assertGreater(len(gps), 50)

        # Subsample fused to nearest GPS second.
        for g in gps[::30]:
            t_g = float(g["t_elapsed_s"])
            nearest = min(
                fused,
                key=lambda r: abs(float(r["t_elapsed_s"]) - t_g),
            )
            de = float(nearest["east_m"]) - float(g["east_m"])
            dn = float(nearest["north_m"]) - float(g["north_m"])
            err = (de * de + dn * dn) ** 0.5
            self.assertLess(err, 8.0, msg=f"t={t_g:.1f}s err={err:.2f}m")

        moving = [r for r in fused if float(r["speed_mps"]) > 0.3]
        self.assertGreater(len(moving), 100)
        speeds = [float(r["speed_mps"]) for r in moving]
        med = sorted(speeds)[len(speeds) // 2]
        self.assertGreater(med, 1.0)
        self.assertLess(med, 2.0)

    def test_plot_script_runs(self) -> None:
        find_fusion_export()
        plot = TOOLS_DIR / "plot_fusion_trajectory.py"
        out = REPO_ROOT / "build" / "test_fusion_plots"
        subprocess.run(
            [
                "python3",
                str(plot),
                "--no-export",
                "--csv-dir",
                str(REPO_ROOT / "build" / "test_fusion_csv_out"),
                "--out",
                str(out),
            ],
            check=True,
            cwd=str(REPO_ROOT),
        )
        for name in (
            "trajectory_gps.png",
            "trajectory_imu_dr.png",
            "trajectory_fused.png",
            "speed_comparison.png",
        ):
            self.assertTrue((out / name).exists(), name)


if __name__ == "__main__":
    unittest.main()
