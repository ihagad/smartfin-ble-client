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
