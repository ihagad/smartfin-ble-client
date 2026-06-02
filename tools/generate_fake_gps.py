#!/usr/bin/env python3
"""
Generate SensorLog-style GPS CSV for offline IMU+GPS testing.

Usage:
    python3 tools/generate_fake_gps.py
    python3 tools/generate_fake_gps.py --trajectory circle --duration 300
    python3 tools/generate_fake_gps.py --sfdat unprocessed/ride_20260601_143000.sfdat

Column schema: docs/SENSORLOG_CSV.md
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

# Fixed session start for reproducible test fixtures (2026-06-01 14:30:00 UTC).
FAKE_SESSION_T0_UNIX = 1780324200

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


def zero_g_enu_mps2(t_s: float, trajectory: str) -> tuple[float, float, float]:
    """
    Kinematic zero-g acceleration in ENU (m/s²) consistent with GPS trajectories.

    Uses identity orientation in the synthetic .sfdat so body frame = ENU.
    Constant-speed legs → ~0; circle includes centripetal acceleration.
    """
    if trajectory == "paddle_pause":
        return (0.0, 0.0, 0.0)
    if trajectory == "straight_line":
        return (0.0, 0.0, 0.0)
    if trajectory == "stationary":
        return (0.0, 0.0, 0.0)
    if trajectory == "circle":
        radius_m = 40.0
        speed_mps = 1.2
        omega = speed_mps / radius_m
        angle = omega * t_s
        # Matches trajectory_circle position derivatives (east, north).
        ax = -radius_m * omega * omega * math.sin(angle)
        ay = radius_m * omega * omega * math.cos(angle)
        return (ax, ay, 0.0)
    return (0.0, 0.0, 0.0)


def write_synthetic_sfdat(
    path: Path,
    duration_s: float,
    trajectory: str = "paddle_pause",
    imu_hz: float = 55.0,
) -> int:
    """
    Write a .sfdat with synthetic QuatImu @ imu_hz, kinematically matched to @p trajectory.

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

    n_samples = int(duration_s * imu_hz) + 1
    dt_ms = 1000.0 / imu_hz

    with path.open("wb") as f:
        f.write(header)
        for i in range(n_samples):
            elapsed_ms = int(round(i * dt_ms))
            t_s = elapsed_ms / 1000.0
            ax_0g, ay_0g, az_0g = zero_g_enu_mps2(t_s, trajectory)
            # Body frame = ENU with identity quat; gravity along +Z (g-units z=1).
            ax = ax_0g
            ay = ay_0g
            az = az_0g + G0_MPS2
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
                1.0,
                0.0,
                0.0,
                0.0,
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

    parser = argparse.ArgumentParser(
        description="Generate SensorLog-style fake GPS CSV (and optional .sfdat)."
    )
    parser.add_argument(
        "--trajectory",
        choices=sorted(TRAJECTORIES),
        default="paddle_pause",
        help="Path shape (default: paddle_pause)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=180.0,
        help="Session length in seconds (default: 180 = 3 min)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="GPS sample rate in Hz (default: 1.0)",
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
        default=FAKE_SESSION_T0_UNIX,
        help=f"Unix session start in seconds (default: {FAKE_SESSION_T0_UNIX})",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    global FAKE_SESSION_T0_UNIX
    FAKE_SESSION_T0_UNIX = args.t0

    if args.duration <= 0 or args.rate <= 0:
        print("duration and rate must be positive", file=sys.stderr)
        return 1

    dt_s = 1.0 / args.rate
    gen = TRAJECTORIES[args.trajectory]
    fixes = gen(args.duration, dt_s)

    write_gps_csv(args.output, fixes)
    print(f"Wrote {len(fixes)} GPS fixes -> {args.output}")
    print(f"  T0 = {FAKE_SESSION_T0_UNIX} (2026-06-01T14:30:00Z when using default)")
    print(f"  trajectory = {args.trajectory}, duration = {args.duration}s, rate = {args.rate} Hz")

    if not args.no_sfdat:
        n = write_synthetic_sfdat(args.sfdat, args.duration, args.trajectory)
        print(
            f"Wrote {n} QuatImu records @ 55 Hz -> {args.sfdat} "
            f"(kinematics matched to {args.trajectory})"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
