#!/usr/bin/env bash
# Run the offline GPS+IMU integration pipeline on committed fake fixtures.
set -euo pipefail

REPO=/workspace
BUILD="${SMARTFIN_BUILD_DIR:-${REPO}/build}"
OUT="${SMARTFIN_OUT_DIR:-${REPO}/docker-out}"
T0="${SMARTFIN_T0:-1780324200}"
GPS_CSV="${SMARTFIN_GPS_CSV:-${REPO}/unprocessed/fake_gps.csv}"
SFDAT="${SMARTFIN_SFDAT:-${REPO}/unprocessed/ride_20260601_143000.sfdat}"

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
    --out "${OUT}/plots"

echo ""
echo "Done. Artifacts:"
echo "  CSV:   ${OUT}/csv/"
echo "  PNG:   ${OUT}/plots/"
ls -la "${OUT}/csv" "${OUT}/plots"
