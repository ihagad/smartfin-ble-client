# GPS / time-align tests (phase A + B)

Validates fake SensorLog GPS fixtures, geo conversion, and GPS–IMU session alignment.

## What is covered

| Item | Test |
|------|------|
| SensorLog CSV schema | `TestSensorLogSchema` |
| `generate_fake_gps.py` trajectories | `TestGenerateFakeGpsTool` |
| Fixed T0 + paired `.sfdat` | `TestCommittedFixtures` |
| Python/C++ geo golden (all trajectories) | `test_geo_golden.py`, `GpsGeoGolden.*` |
| `align_sessions` overlap ≥ 80% | `test_align_sessions.py`, `GpsSessionAlign.PairedFixturesOverlapAndSpan` |
| Low overlap warning | `GpsSessionAlign.LowOverlapEmitsWarning` |
| C++ parser + `load_sensorlog_csv` | `GpsSensorLogParser.LoadsFakeFixture` |
| `load_ride` → `orient_ride` → `accel_global` | `ahrs_orient_test`, `test_ahrs_orient.py` |
| Oriented sample rate ~55 Hz | `AhrsOrient.FixtureSampleCountAndRate` |
| EKF GPS–IMU fusion (`fuse_gps_imu`) | `ekf_fusion_test` |
| Trajectory CSV + fusion plots | `test_fusion_trajectory.py`, `tools/plot_fusion_trajectory.py` |
| Phase G fake validation checklist | `test_validation_fake.py`, `GpsValidationFake.*`, `EkfFusion.FakeGps*` |

## Run

From repo root:

```bash
python3 -m unittest discover -s tests/gps -v
```

Or via CTest after configuring CMake:

```bash
ctest --test-dir build -R gps_
```

Regenerate fixtures if generator tests fail on missing files:

```bash
python3 tools/generate_fake_gps.py
```
