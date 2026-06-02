#!/usr/bin/env python3
"""
Generate SensorLog-style GPS CSV for offline IMU+GPS testing.

GPS path and synthetic IMU accelerations share one trajectory scenario.
Configure via CLI flags or SMARTFIN_* environment variables (see tools/fixture_config.py).

Usage:
    python3 tools/generate_fake_gps.py
    python3 tools/generate_fake_gps.py --trajectory circle --duration 300
    SMARTFIN_TRAJECTORY=straight_line python3 tools/generate_fake_gps.py
    python3 tools/generate_fake_gps.py --sfdat unprocessed/ride_20260601_143000.sfdat

Column schema: docs/SENSORLOG_CSV.md
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from fixture_config import (
    FAKE_SESSION_T0_UNIX,
    FixtureConfig,
    _env_bool,
)

# Fixed session start for reproducible test fixtures (2026-06-01 14:30:00 UTC).

# Scripps Pier, La Jolla — nominal surf test origin.
ORIGIN_LAT = 32.867512
ORIGIN_LON = -117.256843
ORIGIN_ALT_M = 2.3

EARTH_RADIUS_M = 6_371_000.0

GPS_COLUMNS = [
    "locationTimestamp_since1970(s)",
    "locationLatitude(WGS84)",
    "locationLongitude(WGS84)",
    "locationAltitude(m)",
    "locationSpeed(m/s)",
    "locationSpeedAccuracy(m/s)",
    "locationCourse(°)",
    "locationCourseAccuracy(°)",
    "locationVerticalAccuracy(m)",
    "locationHorizontalAccuracy(m)",
    "locationFloor(Z)",
]

# .sfdat on-disk layout (must match src/pipeline/file_sink.hpp).
RIDE_FILE_MAGIC = 0x5346444154  # "SFDAT"
RIDE_FILE_VERSION = 2
RECORD_TAG_QUAT_IMU = 0x02


@dataclass(frozen=True)
class GpsFix:
    unix_s: float
    lat: float
    lon: float
    alt_m: float
    speed_mps: float
    course_deg: float


def meters_to_latlon(
    lat0: float, lon0: float, east_m: float, north_m: float
) -> tuple[float, float]:
    """ENU horizontal offset (m) → WGS84 lat/lon (deg). Inverse of latlon_to_enu."""
    lat_rad = math.radians(lat0)
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon)


def latlon_to_enu(
    lat0: float,
    lon0: float,
    alt0_m: float,
    lat: float,
    lon: float,
    alt_m: float,
) -> tuple[float, float, float]:
    """WGS84 → local ENU (m). Must match src/proccessing/gps/geo_utils.cpp."""
    lat0_rad = math.radians(lat0)
    dlat_rad = math.radians(lat - lat0)
    dlon_rad = math.radians(lon - lon0)
    north_m = dlat_rad * EARTH_RADIUS_M
    east_m = dlon_rad * EARTH_RADIUS_M * math.cos(lat0_rad)
    up_m = alt_m - alt0_m
    return east_m, north_m, up_m


def enu_to_latlon(
    lat0: float, lon0: float, east_m: float, north_m: float
) -> tuple[float, float]:
    """ENU horizontal (m) → WGS84 lat/lon (deg)."""
    return meters_to_latlon(lat0, lon0, east_m, north_m)


def course_deg_from_velocity(east_mps: float, north_mps: float) -> float:
    if abs(east_mps) < 1e-6 and abs(north_mps) < 1e-6:
        return -1.0
    deg = math.degrees(math.atan2(east_mps, north_mps))
    return deg % 360.0


def trajectory_straight_line(
    duration_s: float, dt_s: float, speed_mps: float = 1.5
) -> list[GpsFix]:
    """Move east along a straight line at constant speed."""
    fixes: list[GpsFix] = []
    t = 0.0
    while t <= duration_s + 1e-9:
        east = speed_mps * t
        lat, lon = meters_to_latlon(ORIGIN_LAT, ORIGIN_LON, east, 0.0)
        fixes.append(
            GpsFix(
                unix_s=FAKE_SESSION_T0_UNIX + t,
                lat=lat,
                lon=lon,
                alt_m=ORIGIN_ALT_M,
                speed_mps=speed_mps,
                course_deg=course_deg_from_velocity(speed_mps, 0.0),
            )
        )
        t += dt_s
    return fixes


def trajectory_circle(
    duration_s: float,
    dt_s: float,
    radius_m: float = 40.0,
    speed_mps: float = 1.2,
) -> list[GpsFix]:
    """Orbit a fixed center at constant ground speed."""
    omega = speed_mps / radius_m
    fixes: list[GpsFix] = []
    t = 0.0
    while t <= duration_s + 1e-9:
        angle = omega * t
        east = radius_m * math.sin(angle)
        north = radius_m * (1.0 - math.cos(angle))
        lat, lon = meters_to_latlon(ORIGIN_LAT, ORIGIN_LON, east, north)
        east_vel = speed_mps * math.cos(angle)
        north_vel = speed_mps * math.sin(angle)
        fixes.append(
            GpsFix(
                unix_s=FAKE_SESSION_T0_UNIX + t,
                lat=lat,
                lon=lon,
                alt_m=ORIGIN_ALT_M,
                speed_mps=speed_mps,
                course_deg=course_deg_from_velocity(east_vel, north_vel),
            )
        )
        t += dt_s
    return fixes


def trajectory_paddle_pause(
    duration_s: float,
    dt_s: float,
    paddle_s: float = 30.0,
    pause_s: float = 15.0,
    speed_mps: float = 1.5,
) -> list[GpsFix]:
    """Paddle north, then hold position — repeats until duration elapses."""
    fixes: list[GpsFix] = []
    t = 0.0
    north_offset = 0.0
    while t <= duration_s + 1e-9:
        cycle = paddle_s + pause_s
        phase_t = t % cycle
        if phase_t < paddle_s:
            north_offset += speed_mps * dt_s
            speed = speed_mps
            course = 0.0
        else:
            speed = 0.0
            course = -1.0

        lat, lon = meters_to_latlon(ORIGIN_LAT, ORIGIN_LON, 0.0, north_offset)
        fixes.append(
            GpsFix(
                unix_s=FAKE_SESSION_T0_UNIX + t,
                lat=lat,
                lon=lon,
                alt_m=ORIGIN_ALT_M,
                speed_mps=speed,
                course_deg=course,
            )
        )
        t += dt_s
    return fixes


def trajectory_stationary(duration_s: float, dt_s: float) -> list[GpsFix]:
    """Fixed position — useful for drift / alignment sanity checks."""
    fixes: list[GpsFix] = []
    t = 0.0
    while t <= duration_s + 1e-9:
        fixes.append(
            GpsFix(
                unix_s=FAKE_SESSION_T0_UNIX + t,
                lat=ORIGIN_LAT,
                lon=ORIGIN_LON,
                alt_m=ORIGIN_ALT_M,
                speed_mps=0.0,
                course_deg=-1.0,
            )
        )
        t += dt_s
    return fixes


TRAJECTORIES = {
    "straight_line": trajectory_straight_line,
    "circle": trajectory_circle,
    "paddle_pause": trajectory_paddle_pause,
    "stationary": trajectory_stationary,
}


def fix_to_row(fix: GpsFix) -> dict[str, str | float | int]:
    speed = fix.speed_mps if fix.speed_mps > 0 else -1.0
    course = fix.course_deg if fix.course_deg >= 0 else -1.0
    return {
        "locationTimestamp_since1970(s)": f"{fix.unix_s:.3f}",
        "locationLatitude(WGS84)": f"{fix.lat:.6f}",
        "locationLongitude(WGS84)": f"{fix.lon:.6f}",
        "locationAltitude(m)": f"{fix.alt_m:.1f}",
        "locationSpeed(m/s)": f"{speed:.3f}",
        "locationSpeedAccuracy(m/s)": "0.300",
        "locationCourse(°)": f"{course:.1f}",
        "locationCourseAccuracy(°)": "5.0",
        "locationVerticalAccuracy(m)": "5.0",
        "locationHorizontalAccuracy(m)": "3.2",
        "locationFloor(Z)": 0,
    }


def write_gps_csv(path: Path, fixes: list[GpsFix]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=GPS_COLUMNS)
        writer.writeheader()
        for fix in fixes:
            writer.writerow(fix_to_row(fix))


G0_MPS2 = 9.80665


def yaw_quat_wxyz(yaw_rad: float) -> tuple[float, float, float, float]:
    """Rotation about +Up (ENU yaw). Matches math3d body-from-earth convention."""
    half = 0.5 * yaw_rad
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def synthetic_body_accel_mps2(
    t_s: float,
    cfg: FixtureConfig,
    rng: random.Random,
) -> tuple[float, float, float, float, float, float, float]:
    """
    Build one QuatImu sample with optional orientation/accel error.

    Returns (ax, ay, az, qw, qx, qy, qz) in the on-disk layout.

    Error models (combinable):
      - yaw drift: reported quaternion yaws vs true level frame → mis-projected accel
      - body accel bias: constant offset integrated into velocity/position drift
      - accel noise: zero-g white noise in body frame
    """
    ax_0g, ay_0g, az_0g = zero_g_enu_mps2(t_s, cfg)

    if cfg.imu_accel_noise_std_g > 0.0:
        ax_0g += rng.gauss(0.0, cfg.imu_accel_noise_std_g) * G0_MPS2
        ay_0g += rng.gauss(0.0, cfg.imu_accel_noise_std_g) * G0_MPS2
        az_0g += rng.gauss(0.0, cfg.imu_accel_noise_std_g) * G0_MPS2

    ax_0g += cfg.imu_accel_bias_x_mps2
    ay_0g += cfg.imu_accel_bias_y_mps2
    az_0g += cfg.imu_accel_bias_z_mps2

    # Physical sensor in a level board frame (= ENU when truth is identity).
    ax = ax_0g
    ay = ay_0g
    az = az_0g + G0_MPS2

    yaw_rad = math.radians(cfg.imu_yaw_drift_dps) * t_s
    qw, qx, qy, qz = yaw_quat_wxyz(yaw_rad)
    return ax, ay, az, qw, qx, qy, qz


def zero_g_enu_mps2(t_s: float, cfg: FixtureConfig) -> tuple[float, float, float]:
    """
    Kinematic zero-g acceleration in ENU (m/s²) consistent with GPS trajectories.

    Uses identity orientation in the synthetic .sfdat so body frame = ENU.
    Constant-speed legs → ~0; circle includes centripetal acceleration.
    """
    if cfg.trajectory in ("paddle_pause", "straight_line", "stationary"):
        return (0.0, 0.0, 0.0)
    if cfg.trajectory == "circle":
        radius_m = cfg.circle_radius_m
        speed_mps = cfg.circle_speed_mps
        omega = speed_mps / radius_m
        angle = omega * t_s
        ax = -radius_m * omega * omega * math.sin(angle)
        ay = radius_m * omega * omega * math.cos(angle)
        return (ax, ay, 0.0)
    return (0.0, 0.0, 0.0)


def write_synthetic_sfdat(
    path: Path,
    cfg: FixtureConfig,
) -> int:
    """
    Write a .sfdat with synthetic QuatImu @ cfg.imu_hz, matched to cfg.trajectory.

    Identity quaternion (level, ENU = body): accel_ms2 = zero_g_enu + gravity on Up.
    Returns number of records written.
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    # WireQuatImu packed layout (file_sink.hpp).
    quat_fmt = "<I3f3f3f4ffB"
    quat_size = struct.calcsize(quat_fmt)
    imu_size = struct.calcsize("<I3f3f3f")
    temp_size = struct.calcsize("<IfB")
    fwver_size = struct.calcsize("<I33s")
    header = struct.pack(
        "<QH4B",
        RIDE_FILE_MAGIC,
        RIDE_FILE_VERSION,
        imu_size,
        quat_size,
        temp_size,
        fwver_size,
    )

    n_samples = int(cfg.duration_s * cfg.imu_hz) + 1
    dt_ms = 1000.0 / cfg.imu_hz
    rng = random.Random(cfg.t0_unix)

    with path.open("wb") as f:
        f.write(header)
        for i in range(n_samples):
            elapsed_ms = int(round(i * dt_ms))
            t_s = elapsed_ms / 1000.0
            ax, ay, az, qw, qx, qy, qz = synthetic_body_accel_mps2(t_s, cfg, rng)
            payload = struct.pack(
                quat_fmt,
                elapsed_ms,
                ax,
                ay,
                az,
                0.0,
                0.0,
                0.0,
                25.0,
                0.0,
                0.0,
                qw,
                qx,
                qy,
                qz,
                5.0,
                1,
            )
            record = struct.pack("<B", RECORD_TAG_QUAT_IMU) + payload
            f.write(record)

    return n_samples


def parse_args(argv: list[str]) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parent.parent
    default_csv = repo_root / "unprocessed" / "fake_gps.csv"
    default_sfdat = repo_root / "unprocessed" / "ride_20260601_143000.sfdat"
    defaults = FixtureConfig.from_env()

    parser = argparse.ArgumentParser(
        description="Generate SensorLog-style fake GPS CSV (and optional .sfdat)."
    )
    parser.add_argument(
        "--trajectory",
        choices=sorted(TRAJECTORIES),
        default=defaults.trajectory,
        help="Path shape for GPS + synthetic IMU (default: %(default)s)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=defaults.duration_s,
        help="Session length in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=defaults.gps_rate_hz,
        help="GPS sample rate in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=defaults.speed_mps,
        help="Ground speed for straight_line / paddle_pause (default: %(default)s)",
    )
    parser.add_argument(
        "--circle-radius",
        type=float,
        default=defaults.circle_radius_m,
        help="Circle radius in meters (default: %(default)s)",
    )
    parser.add_argument(
        "--circle-speed",
        type=float,
        default=defaults.circle_speed_mps,
        help="Circle ground speed in m/s (default: %(default)s)",
    )
    parser.add_argument(
        "--paddle-s",
        type=float,
        default=defaults.paddle_s,
        help="Paddle segment length in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--pause-s",
        type=float,
        default=defaults.pause_s,
        help="Pause segment length in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--imu-yaw-drift-dps",
        type=float,
        default=defaults.imu_yaw_drift_dps,
        help="Reported quaternion yaw drift (deg/s); simulates gyro bias (default: %(default)s)",
    )
    parser.add_argument(
        "--imu-accel-bias-x",
        type=float,
        default=defaults.imu_accel_bias_x_mps2,
        help="Body-frame accel bias X in m/s² (default: %(default)s)",
    )
    parser.add_argument(
        "--imu-accel-bias-y",
        type=float,
        default=defaults.imu_accel_bias_y_mps2,
        help="Body-frame accel bias Y in m/s² (default: %(default)s)",
    )
    parser.add_argument(
        "--imu-accel-bias-z",
        type=float,
        default=defaults.imu_accel_bias_z_mps2,
        help="Body-frame accel bias Z in m/s² (default: %(default)s)",
    )
    parser.add_argument(
        "--imu-accel-noise-g",
        type=float,
        default=defaults.imu_accel_noise_std_g,
        help="Zero-g accel white noise σ in g (default: %(default)s)",
    )
    parser.add_argument(
        "--imu-error-demo",
        action="store_true",
        default=_env_bool("SMARTFIN_IMU_ERROR_DEMO", False),
        help="Preset IMU error (1°/s yaw drift + 0.015 m/s² X bias) for fusion demos",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_csv,
        help=f"Output CSV path (default: {default_csv.relative_to(repo_root)})",
    )
    parser.add_argument(
        "--sfdat",
        type=Path,
        default=default_sfdat,
        help="Also write synthetic IMU .sfdat aligned to t=0 (default: paired fixture)",
    )
    parser.add_argument(
        "--no-sfdat",
        action="store_true",
        help="Skip synthetic .sfdat generation",
    )
    parser.add_argument(
        "--t0",
        type=int,
        default=defaults.t0_unix,
        help=f"Unix session start in seconds (default: {defaults.t0_unix})",
    )
    return parser.parse_args(argv)


def args_to_config(args: argparse.Namespace) -> FixtureConfig:
    if args.imu_error_demo:
        return FixtureConfig(
            trajectory=args.trajectory,
            duration_s=args.duration,
            gps_rate_hz=args.rate,
            t0_unix=args.t0,
            speed_mps=args.speed,
            circle_radius_m=args.circle_radius,
            circle_speed_mps=args.circle_speed,
            paddle_s=args.paddle_s,
            pause_s=args.pause_s,
            imu_yaw_drift_dps=1.0,
            imu_accel_bias_x_mps2=0.015,
        )

    return FixtureConfig(
        trajectory=args.trajectory,
        duration_s=args.duration,
        gps_rate_hz=args.rate,
        t0_unix=args.t0,
        speed_mps=args.speed,
        circle_radius_m=args.circle_radius,
        circle_speed_mps=args.circle_speed,
        paddle_s=args.paddle_s,
        pause_s=args.pause_s,
        imu_yaw_drift_dps=args.imu_yaw_drift_dps,
        imu_accel_bias_x_mps2=args.imu_accel_bias_x,
        imu_accel_bias_y_mps2=args.imu_accel_bias_y,
        imu_accel_bias_z_mps2=args.imu_accel_bias_z,
        imu_accel_noise_std_g=args.imu_accel_noise_g,
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    cfg = args_to_config(args)

    global FAKE_SESSION_T0_UNIX
    FAKE_SESSION_T0_UNIX = cfg.t0_unix

    if cfg.duration_s <= 0 or cfg.gps_rate_hz <= 0:
        print("duration and rate must be positive", file=sys.stderr)
        return 1

    dt_s = 1.0 / cfg.gps_rate_hz
    gen = TRAJECTORIES[cfg.trajectory]
    fixes = gen(cfg.duration_s, dt_s, **cfg.trajectory_kwargs())

    write_gps_csv(args.output, fixes)
    print(f"Wrote {len(fixes)} GPS fixes -> {args.output}")
    print(f"  T0 = {cfg.t0_unix} (2026-06-01T14:30:00Z when using default)")
    print(
        f"  trajectory = {cfg.trajectory}, duration = {cfg.duration_s}s, "
        f"rate = {cfg.gps_rate_hz} Hz"
    )

    if not args.no_sfdat:
        n = write_synthetic_sfdat(args.sfdat, cfg)
        err_note = ""
        if cfg.imu_error_enabled():
            err_note = (
                f"  IMU error: yaw_drift={cfg.imu_yaw_drift_dps} deg/s, "
                f"bias=({cfg.imu_accel_bias_x_mps2}, {cfg.imu_accel_bias_y_mps2}, "
                f"{cfg.imu_accel_bias_z_mps2}) m/s², "
                f"noise σ={cfg.imu_accel_noise_std_g} g\n"
            )
        print(
            f"Wrote {n} QuatImu records @ {cfg.imu_hz:g} Hz -> {args.sfdat} "
            f"(kinematics matched to {cfg.trajectory})"
        )
        if err_note:
            print(err_note, end="")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
