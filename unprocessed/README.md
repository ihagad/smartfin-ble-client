# Unprocessed test fixtures

Offline session assets for GPS–IMU fusion development without a live Apple Watch or fin BLE session.

## Fixed session window

| Field | Value |
|-------|-------|
| **T0 (Unix s)** | `1780324200` |
| **T0 (ISO 8601)** | `2026-06-01T14:30:00Z` |
| **Nominal duration** | 180 s (3 min) |
| **GPS rate** | 1 Hz |
| **IMU rate** | 55 Hz (in `.sfdat`) |

GPS timestamps in `fake_gps.csv` are absolute Unix seconds. Fin IMU in `ride_20260601_143000.sfdat` uses **`elapsed_time_ms`** from session start (t = 0 at T0).

```
t_elapsed_s = locationTimestamp_since1970(s) − 1780324200
t_elapsed_ms = elapsed_time_ms   # in .sfdat
```

## Files

| File | Source |
|------|--------|
| `fake_gps.csv` | `python3 tools/generate_fake_gps.py` — paddle + pause trajectory |
| `ride_20260601_143000.sfdat` | Same script (`--sfdat`); synthetic QuatImu @ 55 Hz, zero-g matched to `paddle_pause` GPS |

Replace the `.sfdat` with a real recording when hardware is available:

```bash
# Start SensorLog on Watch and ride_recorder on Mac at the same wall-clock time.
./ride_recorder unprocessed/ride_20260601_143000.sfdat
```

Keep **T0** documented here when regenerating fixtures so GPS and IMU stay aligned.

## Regenerate

Trajectory and tuning knobs (GPS path + matched synthetic IMU):

| Variable | Default | Meaning |
|----------|---------|---------|
| `SMARTFIN_TRAJECTORY` | `paddle_pause` | `straight_line`, `circle`, `paddle_pause`, `stationary` |
| `SMARTFIN_FIXTURE_DURATION` | `180` | Session length (s) |
| `SMARTFIN_FIXTURE_GPS_RATE` | `1.0` | GPS Hz |
| `SMARTFIN_FIXTURE_SPEED` | `1.5` | m/s for straight_line / paddle_pause |
| `SMARTFIN_CIRCLE_RADIUS` | `40` | Circle radius (m) |
| `SMARTFIN_CIRCLE_SPEED` | `1.2` | Circle speed (m/s) |
| `SMARTFIN_PADDLE_S` | `30` | Paddle segment (s) |
| `SMARTFIN_PAUSE_S` | `15` | Pause segment (s) |
| `SMARTFIN_IMU_YAW_DRIFT_DPS` | `0` | Yaw drift in reported quaternion (deg/s) |
| `SMARTFIN_IMU_ACCEL_BIAS_X/Y/Z` | `0` | Body-frame accel bias (m/s²) |
| `SMARTFIN_IMU_ACCEL_NOISE_G` | `0` | Zero-g noise σ (g) |
| `SMARTFIN_IMU_ERROR_DEMO` | `0` | Set `1` for preset drift demo |

```bash
python3 tools/generate_fake_gps.py --trajectory paddle_pause --duration 180
SMARTFIN_TRAJECTORY=circle python3 tools/generate_fake_gps.py
python3 tools/generate_fake_gps.py --trajectory circle --imu-error-demo
```

Schema details: [docs/SENSORLOG_CSV.md](../docs/SENSORLOG_CSV.md)
