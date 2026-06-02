# SensorLog GPS CSV Schema

SensorLog ([Bernd Thomas](http://sensorlog.berndthomas.net/)) exports Apple Watch / iPhone GPS as CSV. This repo uses that format for offline GPS–IMU alignment tests without a live watch session.

## GPS columns (Core Location)

These are the location fields documented for SensorLog. Column names and units match the app export exactly.

| Column | Type | Units | Notes |
|--------|------|-------|-------|
| `locationTimestamp_since1970(s)` | float | seconds | Unix epoch (UTC). Primary alignment key vs fin IMU. |
| `locationLatitude(WGS84)` | float | degrees | WGS84, −90 … 90 |
| `locationLongitude(WGS84)` | float | degrees | WGS84, −180 … 180 |
| `locationAltitude(m)` | float | meters | Height above WGS84 ellipsoid |
| `locationSpeed(m/s)` | float | m/s | Ground speed; **&lt; 0** = invalid |
| `locationSpeedAccuracy(m/s)` | float | m/s | **&lt; 0** = invalid |
| `locationCourse(°)` | float | degrees | 0 = north, clockwise; **&lt; 0** = invalid |
| `locationCourseAccuracy(°)` | float | degrees | **&lt; 0** = invalid |
| `locationVerticalAccuracy(m)` | float | meters | Radius of uncertainty; **&lt; 0** = invalid |
| `locationHorizontalAccuracy(m)` | float | meters | Radius of uncertainty; **&lt; 0** = invalid |
| `locationFloor(Z)` | int | — | Building floor; 0 when unknown |

On Apple Watch exports, SensorLog documents `locationSpeedAccuracy(°)` with a degree suffix — a documentation typo. Real Watch CSVs use m/s; fake fixtures in this repo use `locationSpeedAccuracy(m/s)` to match iPhone exports and Core Location semantics.

## Optional general columns

Full SensorLog sessions may also include:

| Column | Notes |
|--------|-------|
| `loggingTime(txt)` | Human-readable sample time |
| `loggingSample(N)` | Monotonic sample index |

GPS-only test fixtures (`unprocessed/fake_gps.csv`) omit these and export only the location block above.

## Example row

```csv
locationTimestamp_since1970(s),locationLatitude(WGS84),locationLongitude(WGS84),locationAltitude(m),locationSpeed(m/s),locationSpeedAccuracy(m/s),locationCourse(°),locationCourseAccuracy(°),locationVerticalAccuracy(m),locationHorizontalAccuracy(m),locationFloor(Z)
1780324200.000,32.867512,-117.256843,2.3,1.5,0.3,90.0,5.0,5.0,3.2,0
```

## Aligning with `.sfdat` IMU

Fin recordings store **`elapsed_time_ms`** from session start (t = 0 when BLE logging begins), not Unix time.

To fuse GPS with a `.sfdat` ride file:

1. Record both during the same wall-clock window, **or** use the paired fixtures in `unprocessed/` (fixed `T0`; see `unprocessed/README.md`).
2. Convert GPS timestamps to session-relative seconds:  
   `t_elapsed_s = locationTimestamp_since1970(s) − T0`
3. Interpolate or associate GPS fixes (~1 Hz) with IMU samples (~55 Hz) on `t_elapsed_s`.

## Generating fake GPS

```bash
python3 tools/generate_fake_gps.py --trajectory paddle_pause
```

See `tools/generate_fake_gps.py --help` for trajectory modes, duration, sample rate, and optional synthetic `.sfdat` output.
