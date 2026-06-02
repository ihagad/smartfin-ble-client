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
| `ride_20260601_143000.sfdat` | Same script (`--sfdat`); synthetic QuatImu for loader tests |

Replace the `.sfdat` with a real recording when hardware is available:

```bash
# Start SensorLog on Watch and ride_recorder on Mac at the same wall-clock time.
./ride_recorder unprocessed/ride_20260601_143000.sfdat
```

Keep **T0** documented here when regenerating fixtures so GPS and IMU stay aligned.

## Regenerate

```bash
python3 tools/generate_fake_gps.py --trajectory paddle_pause --duration 180
python3 tools/generate_fake_gps.py --trajectory circle --duration 300 --output unprocessed/fake_gps_circle.csv
```

Schema details: [docs/SENSORLOG_CSV.md](../docs/SENSORLOG_CSV.md)
