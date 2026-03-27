#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPEATS="${GO2_JUMP_BATCH_REPEATS:-2}"
DISTANCES="${GO2_JUMP_BATCH_DISTANCES:-0.20 0.25 0.30 0.35}"
INTENT_MODES="${GO2_JUMP_BATCH_INTENT_MODES:-enabled}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="${ROOT_DIR}/reports/batches/${TIMESTAMP}"
MANIFEST_PATH="${BATCH_DIR}/summary_paths.txt"
AGGREGATE_JSON="${BATCH_DIR}/aggregate.json"
FAILURES_PATH="${BATCH_DIR}/failures.txt"

mkdir -p "${BATCH_DIR}"
: >"${MANIFEST_PATH}"
: >"${FAILURES_PATH}"

echo "Batch run"
echo "  repeats: ${REPEATS}"
echo "  distances: ${DISTANCES}"
echo "  intent modes: ${INTENT_MODES}"
echo "  manifest: ${MANIFEST_PATH}"

for intent_mode in ${INTENT_MODES}; do
  case "${intent_mode}" in
    enabled)
      run_enable_intent="true"
      ;;
    disabled)
      run_enable_intent="false"
      ;;
    *)
      echo "Unsupported intent mode: ${intent_mode}" >&2
      exit 1
      ;;
  esac
  for distance in ${DISTANCES}; do
    for repeat_idx in $(seq 1 "${REPEATS}"); do
      echo
      echo "[intent ${intent_mode}] [distance ${distance}] repeat ${repeat_idx}/${REPEATS}"
      before_latest="$(find "${ROOT_DIR}/reports/trials" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2- || true)"
      if ! GO2_JUMP_ENABLE_INTENT_PLANNER="${run_enable_intent}" \
        "${ROOT_DIR}/scripts/docker_run_single_jump_trial.sh" "${distance}"; then
        echo "intent=${intent_mode} distance=${distance} repeat=${repeat_idx} status=failed" | tee -a "${FAILURES_PATH}"
        continue
      fi
      after_latest="$(find "${ROOT_DIR}/reports/trials" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
      if [ -z "${after_latest}" ] || [ "${after_latest}" = "${before_latest}" ]; then
        echo "intent=${intent_mode} distance=${distance} repeat=${repeat_idx} status=missing_report" | tee -a "${FAILURES_PATH}"
        continue
      fi
      echo "${after_latest}/summary.json" >>"${MANIFEST_PATH}"
    done
  done
done

echo
echo "Aggregating batch results"
if [ ! -s "${MANIFEST_PATH}" ]; then
  echo "No successful trial summaries were collected." >&2
  exit 1
fi
xargs python3 "${ROOT_DIR}/scripts/summarize_jump_trials.py" \
  --output-json "${AGGREGATE_JSON}" <"${MANIFEST_PATH}"

echo
echo "Artifacts"
echo "  manifest: ${MANIFEST_PATH}"
echo "  aggregate: ${AGGREGATE_JSON}"
if [ -s "${FAILURES_PATH}" ]; then
  echo "  failures: ${FAILURES_PATH}"
fi
