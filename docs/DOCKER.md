# Docker build and GPS+IMU pipeline

Run CMake build and the offline fusion pipeline without installing toolchains locally.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) (Docker Desktop on macOS is fine)

## Build image

From the repository root:

```bash
docker build -t smartfin-ble-client .
```

This installs Ubuntu packages (CMake, Ninja, g++, Python), pip packages (`matplotlib`, `numpy`, `scipy`), and builds:

- `fusion_export`, `gps_inspect`, `orient_ride_inspect`
- GPS-related test binaries

SimpleBLE is **off** in the image (no BLE hardware in Docker).

## Run the full pipeline (fake fixtures)

```bash
mkdir -p docker-out
docker run --rm -v "$(pwd)/docker-out:/workspace/docker-out" smartfin-ble-client
```

Or with Compose:

```bash
docker compose build
docker compose run --rm pipeline
```

**Host outputs:**

| Path | Contents |
|------|----------|
| `docker-out/csv/` | `trajectory_fused.csv`, `trajectory_gps.csv`, … |
| `docker-out/plots/` | `trajectory_*.png`, `speed_comparison.png` |

Open plots on macOS: `open docker-out/plots`

## Synthetic path shapes (GPS + IMU)

Both the SensorLog CSV and synthetic `.sfdat` IMU come from `tools/generate_fake_gps.py`.
One **`SMARTFIN_TRAJECTORY`** value selects the path for **both** sensors.

| `SMARTFIN_TRAJECTORY` | Motion |
|-----------------------|--------|
| `straight_line` | Constant speed east |
| `circle` | Orbit at `SMARTFIN_CIRCLE_RADIUS` / `SMARTFIN_CIRCLE_SPEED` |
| `paddle_pause` | Paddle north, pause, repeat (default baked fixture) |
| `stationary` | Fixed position |

Optional tuning env vars: `SMARTFIN_FIXTURE_DURATION`, `SMARTFIN_FIXTURE_GPS_RATE`,
`SMARTFIN_FIXTURE_SPEED`, `SMARTFIN_PADDLE_S`, `SMARTFIN_PAUSE_S`.

Regenerate fixtures at container start (writes under `/workspace/fixtures/runtime`, not your host repo):

```bash
docker run --rm \
  -v "$(pwd)/docker-out:/workspace/docker-out" \
  -e SMARTFIN_REGENERATE_FIXTURES=1 \
  -e SMARTFIN_TRAJECTORY=circle \
  smartfin-ble-client
```

Local regeneration (updates `unprocessed/`):

```bash
SMARTFIN_TRAJECTORY=circle python3 tools/generate_fake_gps.py
python3 tools/generate_fake_gps.py --trajectory straight_line --duration 180
```

### Simulate IMU error (drift vs GPS fusion)

By default, synthetic IMU is **perfect** — `trajectory_imu_dr.png` nearly matches GPS. To demo **IMU dead-reckoning drift** while **fusion stays on GPS**, regenerate fixtures with IMU error enabled.

**Requirements:**

1. Run from the repo root (`smartfin-ble-client`).
2. Image must include the latest `docker/run_pipeline.sh` (rebuild after pulling):

```bash
docker build -t smartfin-ble-client .
```

**Quick demo** (circle path + preset IMU error: 1°/s yaw drift + 0.015 m/s² X accel bias):

```bash
docker run --rm \
  -v "$(pwd)/docker-out:/workspace/docker-out" \
  -e SMARTFIN_REGENERATE_FIXTURES=1 \
  -e SMARTFIN_TRAJECTORY=circle \
  -e SMARTFIN_IMU_ERROR_DEMO=1 \
  smartfin-ble-client
```

Each `-e` flag must be prefixed with `-e`. This is **wrong** (env var never reaches the container):

```bash
# WRONG: IMU_ERROR_DEMO=1 without -e SMARTFIN_
docker run ... -e SMARTFIN_TRAJECTORY=circle IMU_ERROR_DEMO=1 smartfin-ble-client
```

**Confirm it worked** — log output should include:

```
== Regenerate fixtures ==
  trajectory=circle
  IMU error demo preset enabled
```

and GPS inspect should show 2D motion, e.g. `east ±40.0 m  north ±79.9 m` (not `east ±0.0 m`).

**What gets overwritten each run:**

| Path | On host? | Effect |
|------|----------|--------|
| `docker-out/csv/`, `docker-out/plots/` | Yes | New fusion CSVs + PNGs |
| `/workspace/fixtures/runtime/` | No (inside container) | Regenerated fake GPS + IMU |
| `unprocessed/` in your repo | No | Unchanged unless you edit locally |

**Manual tuning** (instead of `SMARTFIN_IMU_ERROR_DEMO=1`):

```bash
docker run --rm \
  -v "$(pwd)/docker-out:/workspace/docker-out" \
  -e SMARTFIN_REGENERATE_FIXTURES=1 \
  -e SMARTFIN_TRAJECTORY=circle \
  -e SMARTFIN_IMU_YAW_DRIFT_DPS=1.0 \
  -e SMARTFIN_IMU_ACCEL_BIAS_X=0.015 \
  -e SMARTFIN_IMU_ACCEL_BIAS_Y=0.0 \
  -e SMARTFIN_IMU_ACCEL_BIAS_Z=0.0 \
  -e SMARTFIN_IMU_ACCEL_NOISE_G=0.0 \
  smartfin-ble-client
```

| Env var | Units | What it simulates |
|---------|-------|------------------|
| `SMARTFIN_IMU_YAW_DRIFT_DPS` | deg/s | Reported quaternion yaws over time → accel projected into wrong ENU direction (gyro-like error) |
| `SMARTFIN_IMU_ACCEL_BIAS_X` | m/s² | Constant body-frame accel offset on fin X axis |
| `SMARTFIN_IMU_ACCEL_BIAS_Y` | m/s² | Same on Y |
| `SMARTFIN_IMU_ACCEL_BIAS_Z` | m/s² | Same on Z |
| `SMARTFIN_IMU_ACCEL_NOISE_G` | g (σ) | White noise on zero-g each sample |
| `SMARTFIN_IMU_ERROR_DEMO=1` | preset | Sets yaw 1°/s + X bias 0.015 m/s² (tuned for ~180 s circle) |

GPS CSV stays perfect ground truth; only the synthetic `.sfdat` IMU is wrong. Expect **`trajectory_imu_dr.png`** to diverge from GPS while **`trajectory_fused.png`** stays near GPS.

**Local equivalent** (writes to `unprocessed/`):

```bash
python3 tools/generate_fake_gps.py --trajectory circle --imu-error-demo
python3 tools/generate_fake_gps.py --trajectory circle \
  --imu-yaw-drift-dps 1.0 --imu-accel-bias-x 0.015
```

See also `tools/fixture_config.py` for all env var names.

## Custom inputs

Mount your own `.sfdat` and SensorLog CSV and override env vars:

```bash
docker run --rm \
  -v "$(pwd)/docker-out:/workspace/docker-out" \
  -v "/path/to/my_ride.sfdat:/data/ride.sfdat:ro" \
  -v "/path/to/my_gps.csv:/data/gps.csv:ro" \
  -e SMARTFIN_SFDAT=/data/ride.sfdat \
  -e SMARTFIN_GPS_CSV=/data/gps.csv \
  -e SMARTFIN_T0=1780324200 \
  smartfin-ble-client
```

## Interactive shell

```bash
docker compose run --rm shell
# inside container:
./build/fusion_export unprocessed/ride_20260601_143000.sfdat unprocessed/fake_gps.csv --t0 1780324200 --out /tmp/out
```

## Run tests in Docker

```bash
docker compose run --rm test-gps
```

## Rebuild after code changes

```bash
docker build -t smartfin-ble-client . --no-cache
```

Or use `docker compose build --no-cache`.
