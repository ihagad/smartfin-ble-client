#!/usr/bin/env python3
"""
Plot GPS ground truth, IMU dead-reckoning, and fused trajectories + speed comparison.

Requires trajectory CSVs from fusion_export (or this script runs it).

Usage:
    python3 tools/plot_fusion_trajectory.py
    python3 tools/plot_fusion_trajectory.py --out tools/demo/output/fusion_plots
"""

from __future__ import annotations

import argparse
import csv
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
UNPROCESSED = REPO_ROOT / "unprocessed"
T0_UNIX = 1780324200
G0 = 9.80665

DEFAULT_SFDAT = UNPROCESSED / "ride_20260601_143000.sfdat"
DEFAULT_GPS = UNPROCESSED / "fake_gps.csv"
BUILD_DIRS = [REPO_ROOT / "build", REPO_ROOT / "cmake-build-debug"]


def find_fusion_export() -> Path:
    for d in BUILD_DIRS:
        p = d / "fusion_export"
        if p.exists():
            return p
    raise FileNotFoundError(
        "fusion_export not found. Build: cmake --build build --target fusion_export"
    )


def run_export(out_dir: Path, sfdat: Path, gps_csv: Path, t0: float) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    exe = find_fusion_export()
    subprocess.run(
        [str(exe), str(sfdat), str(gps_csv), "--t0", str(t0), "--out", str(out_dir)],
        check=True,
    )


def load_trajectory_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty trajectory: {path}")

    def col(name: str, dtype=float) -> np.ndarray:
        return np.array([dtype(r[name]) for r in rows], dtype=np.float64)

    return {
        "t_s": col("t_elapsed_s"),
        "east_m": col("east_m"),
        "north_m": col("north_m"),
        "lat": col("lat_deg"),
        "lon": col("lon_deg"),
        "speed_mps": col("speed_mps"),
        "ve": col("ve"),
        "vn": col("vn"),
        "vu": col("vu"),
    }


def plot_path_enu(
    data: dict[str, np.ndarray],
    title: str,
    out_path: Path,
    *,
    expect_speed: float | None = None,
) -> None:
    """Bird's-eye path: East vs North (meters from session origin)."""
    east = data["east_m"]
    north = data["north_m"]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(east, north, lw=1.2, color="steelblue")
    ax.plot(east[0], north[0], "o", color="green", label="start")
    ax.plot(east[-1], north[-1], "s", color="crimson", label="end")

    # Pure north/south or east/west tracks have zero span on one axis; equal
    # aspect then collapses that axis and the path becomes invisible.
    e_span = float(np.ptp(east))
    n_span = float(np.ptp(north))
    if e_span < 1e-3:
        pad = max(n_span * 0.5, 5.0)
        cx = float(np.mean(east))
        ax.set_xlim(cx - pad, cx + pad)
    if n_span < 1e-3:
        pad = max(e_span * 0.5, 5.0)
        cy = float(np.mean(north))
        ax.set_ylim(cy - pad, cy + pad)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(title)
    ax.grid(True, lw=0.3)
    ax.legend(loc="best")
    if expect_speed is not None and "speed_mps" in data:
        moving = data["speed_mps"] > 0.1
        if np.any(moving):
            med = float(np.median(data["speed_mps"][moving]))
            ax.text(
                0.02,
                0.98,
                f"median speed ≈ {med:.2f} m/s",
                transform=ax.transAxes,
                va="top",
                fontsize=9,
            )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_speed_comparison(
    gps: dict[str, np.ndarray],
    fused: dict[str, np.ndarray],
    imu_accel_g: np.ndarray,
    imu_t_s: np.ndarray,
    out_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(gps["t_s"], gps["speed_mps"], "o-", ms=3, lw=1, color="darkorange", label="GPS speed")
    ax.plot(
        fused["t_s"],
        fused["speed_mps"],
        lw=0.7,
        color="royalblue",
        alpha=0.85,
        label="Fused speed",
    )
    ax2 = ax.twinx()
    ax2.plot(
        imu_t_s,
        imu_accel_g,
        lw=0.5,
        color="gray",
        alpha=0.6,
        label="|accel_global| (g)",
    )
    ax.set_xlabel("Time since session start (s)")
    ax.set_ylabel("Speed (m/s)")
    ax2.set_ylabel("|accel_global| (g)", color="gray")
    ax.set_title("Speed vs time — GPS vs fused vs IMU activity")
    ax.grid(True, lw=0.3)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def load_imu_accel(csv_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    path = csv_dir / "imu_accel.csv"
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    t_s = np.array([float(r["t_elapsed_s"]) for r in rows])
    mag = np.array([float(r["accel_mag_g"]) for r in rows])
    return t_s, mag


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sfdat", type=Path, default=DEFAULT_SFDAT)
    parser.add_argument("--gps", type=Path, default=DEFAULT_GPS)
    parser.add_argument("--t0", type=float, default=T0_UNIX)
    parser.add_argument(
        "--out",
        type=Path,
        default=REPO_ROOT / "tools" / "demo" / "output" / "fusion_plots",
    )
    parser.add_argument(
        "--csv-dir",
        type=Path,
        default=None,
        help="folder with trajectory_*.csv (default: <out>/csv after export)",
    )
    parser.add_argument("--no-export", action="store_true", help="skip fusion_export")
    args = parser.parse_args()

    if not args.no_export:
        csv_dir = args.csv_dir or (args.out / "csv")
        run_export(csv_dir, args.sfdat, args.gps, args.t0)
    else:
        csv_dir = args.csv_dir or args.out
        if not (csv_dir / "trajectory_gps.csv").exists() and (args.out / "csv" / "trajectory_gps.csv").exists():
            csv_dir = args.out / "csv"

    gps = load_trajectory_csv(csv_dir / "trajectory_gps.csv")
    fused = load_trajectory_csv(csv_dir / "trajectory_fused.csv")
    imu_dr = load_trajectory_csv(csv_dir / "trajectory_imu_dr.csv")

    args.out.mkdir(parents=True, exist_ok=True)

    plot_path_enu(
        gps,
        "GPS ground truth (SensorLog / fake fixture)",
        args.out / "trajectory_gps.png",
        expect_speed=1.5,
    )
    plot_path_enu(
        imu_dr,
        "IMU dead reckoning (accel integration, no GPS updates)",
        args.out / "trajectory_imu_dr.png",
    )
    plot_path_enu(
        fused,
        "Fused EKF trajectory",
        args.out / "trajectory_fused.png",
        expect_speed=1.5,
    )

    imu_t, imu_accel = load_imu_accel(csv_dir)
    plot_speed_comparison(
        gps, fused, imu_accel, imu_t, args.out / "speed_comparison.png"
    )

    print(f"Plots saved to {args.out}/")


if __name__ == "__main__":
    main()
