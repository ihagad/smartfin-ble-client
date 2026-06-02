"""
Shared settings for synthetic GPS + IMU test fixtures.

GPS path shape and IMU zero-g kinematics both follow ``trajectory`` (one scenario).
Override via environment variables (Docker) or CLI flags on ``generate_fake_gps.py``.

Environment variables
---------------------
SMARTFIN_TRAJECTORY          straight_line | circle | paddle_pause | stationary
SMARTFIN_FIXTURE_DURATION    session length in seconds (default 180)
SMARTFIN_FIXTURE_GPS_RATE    GPS sample rate in Hz (default 1.0)
SMARTFIN_T0                  Unix session start (default 1780324200)
SMARTFIN_FIXTURE_SPEED       m/s for straight_line and paddle_pause (default 1.5)
SMARTFIN_CIRCLE_RADIUS       circle radius in meters (default 40)
SMARTFIN_CIRCLE_SPEED        circle ground speed in m/s (default 1.2)
SMARTFIN_PADDLE_S            paddle segment length in seconds (default 30)
SMARTFIN_PAUSE_S             pause segment length in seconds (default 15)
SMARTFIN_REGENERATE_FIXTURES set to 1 to rebuild CSV + .sfdat before pipeline run

Optional synthetic IMU error (shows IMU drift vs GPS-corrected fusion):
SMARTFIN_IMU_YAW_DRIFT_DPS   constant yaw-rate error in reported quaternion (default 0)
SMARTFIN_IMU_ACCEL_BIAS_X    body-frame accel bias in m/s² (default 0)
SMARTFIN_IMU_ACCEL_BIAS_Y    body-frame accel bias in m/s² (default 0)
SMARTFIN_IMU_ACCEL_BIAS_Z    body-frame accel bias in m/s² (default 0)
SMARTFIN_IMU_ACCEL_NOISE_G   zero-g white noise σ in g-units (default 0)
SMARTFIN_IMU_ERROR_DEMO      set to 1 for preset demo drift (yaw 1°/s + small bias)
"""

from __future__ import annotations

import os
from dataclasses import dataclass

FAKE_SESSION_T0_UNIX = 1780324200

TRAJECTORY_CHOICES = (
    "straight_line",
    "circle",
    "paddle_pause",
    "stationary",
)


def _env(name: str, default: str | None = None) -> str | None:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value


def _env_float(name: str, default: float) -> float:
    raw = _env(name)
    return float(raw) if raw is not None else default


def _env_int(name: str, default: int) -> int:
    raw = _env(name)
    return int(raw) if raw is not None else default


def _env_bool(name: str, default: bool = False) -> bool:
    raw = _env(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class FixtureConfig:
    """One synthetic session: GPS path + matched IMU accelerations."""

    trajectory: str = "paddle_pause"
    duration_s: float = 180.0
    gps_rate_hz: float = 1.0
    t0_unix: int = FAKE_SESSION_T0_UNIX
    speed_mps: float = 1.5
    circle_radius_m: float = 40.0
    circle_speed_mps: float = 1.2
    paddle_s: float = 30.0
    pause_s: float = 15.0
    imu_hz: float = 55.0
    imu_yaw_drift_dps: float = 0.0
    imu_accel_bias_x_mps2: float = 0.0
    imu_accel_bias_y_mps2: float = 0.0
    imu_accel_bias_z_mps2: float = 0.0
    imu_accel_noise_std_g: float = 0.0

    def __post_init__(self) -> None:
        if self.trajectory not in TRAJECTORY_CHOICES:
            choices = ", ".join(TRAJECTORY_CHOICES)
            raise ValueError(f"unknown trajectory {self.trajectory!r}; choose: {choices}")

    @classmethod
    def from_env(cls) -> FixtureConfig:
        demo = _env_bool("SMARTFIN_IMU_ERROR_DEMO", False)
        yaw_drift = _env_float("SMARTFIN_IMU_YAW_DRIFT_DPS", 1.0 if demo else 0.0)
        bias_x = _env_float("SMARTFIN_IMU_ACCEL_BIAS_X", 0.015 if demo else 0.0)
        bias_y = _env_float("SMARTFIN_IMU_ACCEL_BIAS_Y", 0.0)
        bias_z = _env_float("SMARTFIN_IMU_ACCEL_BIAS_Z", 0.0)
        noise_g = _env_float("SMARTFIN_IMU_ACCEL_NOISE_G", 0.0)

        return cls(
            trajectory=_env("SMARTFIN_TRAJECTORY", "paddle_pause") or "paddle_pause",
            duration_s=_env_float("SMARTFIN_FIXTURE_DURATION", 180.0),
            gps_rate_hz=_env_float("SMARTFIN_FIXTURE_GPS_RATE", 1.0),
            t0_unix=_env_int("SMARTFIN_T0", FAKE_SESSION_T0_UNIX),
            speed_mps=_env_float("SMARTFIN_FIXTURE_SPEED", 1.5),
            circle_radius_m=_env_float("SMARTFIN_CIRCLE_RADIUS", 40.0),
            circle_speed_mps=_env_float("SMARTFIN_CIRCLE_SPEED", 1.2),
            paddle_s=_env_float("SMARTFIN_PADDLE_S", 30.0),
            pause_s=_env_float("SMARTFIN_PAUSE_S", 15.0),
            imu_yaw_drift_dps=yaw_drift,
            imu_accel_bias_x_mps2=bias_x,
            imu_accel_bias_y_mps2=bias_y,
            imu_accel_bias_z_mps2=bias_z,
            imu_accel_noise_std_g=noise_g,
        )

    def imu_error_enabled(self) -> bool:
        return (
            abs(self.imu_yaw_drift_dps) > 1e-12
            or abs(self.imu_accel_bias_x_mps2) > 1e-12
            or abs(self.imu_accel_bias_y_mps2) > 1e-12
            or abs(self.imu_accel_bias_z_mps2) > 1e-12
            or abs(self.imu_accel_noise_std_g) > 1e-12
        )

    def trajectory_kwargs(self) -> dict[str, float]:
        if self.trajectory == "straight_line":
            return {"speed_mps": self.speed_mps}
        if self.trajectory == "circle":
            return {
                "radius_m": self.circle_radius_m,
                "speed_mps": self.circle_speed_mps,
            }
        if self.trajectory == "paddle_pause":
            return {
                "paddle_s": self.paddle_s,
                "pause_s": self.pause_s,
                "speed_mps": self.speed_mps,
            }
        return {}

    def regenerate_fixtures_from_env(self) -> bool:
        return _env_bool("SMARTFIN_REGENERATE_FIXTURES", False)
