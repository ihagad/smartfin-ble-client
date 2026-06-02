#!/usr/bin/env bash
# Run the offline GPS+IMU integration pipeline on fake fixtures.
set -euo pipefail

REPO=/workspace
BUILD="${SMARTFIN_BUILD_DIR:-${REPO}/build}"
OUT="${SMARTFIN_OUT_DIR:-${REPO}/docker-out}"
T0="${SMARTFIN_T0:-1780324200}"
GPS_CSV="${SMARTFIN_GPS_CSV:-${REPO}/unprocessed/fake_gps.csv}"
SFDAT="${SMARTFIN_SFDAT:-${REPO}/unprocessed/ride_20260601_143000.sfdat}"
FIXTURE_DIR="${SMARTFIN_FIXTURE_DIR:-${REPO}/fixtures/runtime}"

regenerate_fixtures() {
    local out_csv="${FIXTURE_DIR}/fake_gps.csv"
    local out_sfdat="${FIXTURE_DIR}/ride.sfdat"
    mkdir -p "${FIXTURE_DIR}"

    echo "== Regenerate fixtures =="
    echo "  trajectory=${SMARTFIN_TRAJECTORY:-paddle_pause}"
    echo "  output dir=${FIXTURE_DIR}"
    if [[ "${SMARTFIN_IMU_ERROR_DEMO:-0}" == "1" ]]; then
        echo "  IMU error demo preset enabled"
    fi

    local imu_demo_flag=()
    if [[ "${SMARTFIN_IMU_ERROR_DEMO:-0}" == "1" ]]; then
        imu_demo_flag=(--imu-error-demo)
    fi

    python3 "${REPO}/tools/generate_fake_gps.py" \
        --trajectory "${SMARTFIN_TRAJECTORY:-paddle_pause}" \
        --duration "${SMARTFIN_FIXTURE_DURATION:-180}" \
        --rate "${SMARTFIN_FIXTURE_GPS_RATE:-1.0}" \
        --speed "${SMARTFIN_FIXTURE_SPEED:-1.5}" \
        --circle-radius "${SMARTFIN_CIRCLE_RADIUS:-40}" \
        --circle-speed "${SMARTFIN_CIRCLE_SPEED:-1.2}" \
        --paddle-s "${SMARTFIN_PADDLE_S:-30}" \
        --pause-s "${SMARTFIN_PAUSE_S:-15}" \
        --imu-yaw-drift-dps "${SMARTFIN_IMU_YAW_DRIFT_DPS:-0}" \
        --imu-accel-bias-x "${SMARTFIN_IMU_ACCEL_BIAS_X:-0}" \
        --imu-accel-bias-y "${SMARTFIN_IMU_ACCEL_BIAS_Y:-0}" \
        --imu-accel-bias-z "${SMARTFIN_IMU_ACCEL_BIAS_Z:-0}" \
        --imu-accel-noise-g "${SMARTFIN_IMU_ACCEL_NOISE_G:-0}" \
        "${imu_demo_flag[@]}" \
        --t0 "${T0}" \
        --output "${out_csv}" \
        --sfdat "${out_sfdat}"

    GPS_CSV="${out_csv}"
    SFDAT="${out_sfdat}"
}

if [[ "${SMARTFIN_REGENERATE_FIXTURES:-0}" == "1" ]]; then
    regenerate_fixtures
fi

mkdir -p "${OUT}/csv" "${OUT}/plots"

echo "== GPS inspect =="
"${BUILD}/gps_inspect" "${GPS_CSV}" --t0 "${T0}"

echo ""
echo "== IMU orient inspect =="
"${BUILD}/orient_ride_inspect" "${SFDAT}"

echo ""
echo "== Fusion export =="
"${BUILD}/fusion_export" "${SFDAT}" "${GPS_CSV}" --t0 "${T0}" --out "${OUT}/csv"

echo ""
echo "== Plots =="
python3 "${REPO}/tools/plot_fusion_trajectory.py" \
    --no-export \
    --csv-dir "${OUT}/csv" \
    --out "${OUT}/plots" \
    --sfdat "${SFDAT}" \
    --gps "${GPS_CSV}" \
    --t0 "${T0}"

echo ""
echo "Done. Artifacts:"
echo "  CSV:   ${OUT}/csv/"
echo "  PNG:   ${OUT}/plots/"
ls -la "${OUT}/csv" "${OUT}/plots"
