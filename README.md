# smartfin-ble-client

Cross-platform **C++ backend** for [Smartfin](https://github.com/UCSD-E4E) ocean telemetry. Decodes BLE sensor packets, runs orientation and wave-band signal processing, and exposes results through a pure-C API for Apple Watch and desktop clients.

Part of UCSD Engineers for Exploration (E4E) — Smartfin embeds IMU sensors in a surfboard fin to capture ride dynamics and ocean conditions.

## Goals

- One shared processing core so firmware, the Watch app, and lab tools don’t drift on protocol or signal math
- Run the full pipeline on-device (Apple Watch) without platform-specific C++
- Keep BLE transport on the platform (Swift CoreBluetooth / desktop SimpleBLE); this library owns bytes → metrics

## What This Repo Owns

- BLE telemetry framing, ensemble decoding, and dequantization
- Protocol constants, field scaling, and unit conversion
- AHRS orientation (Madgwick) with world-frame rotation and gravity subtraction
- Butterworth bandpass filtering and SciPy-matching `filtfilt` (Gustafsson initial conditions)
- Wave-metric pipeline building blocks (decimation, wave-band filtering, Welch PSD)
- Pure-C bridge for Swift interop (`src/bridge/`)
- Optional SimpleBLE host transport for desktop testing
- GoogleTest suite with SciPy-generated golden vectors + GitHub Actions CI

## What It Does Not Own

- Firmware / embedded BLE stack ([`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3))
- Apple Watch UI and connection lifecycle ([`smartfin-watch`](https://github.com/charliekush/smartfin-watch))
- Platform presentation models

## Tech Stack

| Area | Tools |
|---|---|
| Language | C++20 |
| Build | CMake |
| Tests | GoogleTest, SciPy/NumPy reference vectors |
| Interop | Pure-C bridge for Swift / watchOS |
| Optional | SimpleBLE (desktop transport) |
| CI | GitHub Actions |

## Architecture

```
raw BLE bytes
  → protocol decode / dequantize
  → AHRS + world-frame processing
  → filter / decimate / spectral analysis
  → sinks (buffer, CSV, log) + C bridge API
```

See [ARCHITECTURE.md](ARCHITECTURE.md) and [Swift / Xcode Integration](docs/SWIFT_INTEGRATION.md).

## Build & Test

```bash
cmake -B build
cmake --build build
cd build && ctest
```

## Design Principles

- Share processing logic, not platform assumptions
- Keep the C bridge narrow and stable for Swift
- Prefer explicit binary layouts; lock compatibility with golden tests
- Make optional deps (SimpleBLE) truly optional

## Applications

- On-device surf session telemetry for Apple Watch
- Lab / desktop tools for validating firmware packets
- Shared library for any Smartfin client that needs consistent IMU → wave metrics
