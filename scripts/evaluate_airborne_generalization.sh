#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/airborne_priority_params.sh"

FIXED_DISTANCES_CSV="${1:-${GO2_JUMP_GENERALIZATION_FIXED_DISTANCES:-0.20,0.25,0.30}}"
RANDOM_TARGET_COUNT="${2:-${GO2_JUMP_GENERALIZATION_RANDOM_TARGET_COUNT:-2}}"
RANDOM_TARGET_SEED="${3:-${GO2_JUMP_GENERALIZATION_RANDOM_TARGET_SEED:-20260326}}"
TRIALS_PER_TARGET="${4:-${GO2_JUMP_GENERALIZATION_TRIALS_PER_TARGET:-1}}"
MAX_ATTEMPTS_PER_TRIAL="${GO2_JUMP_GENERALIZATION_MAX_ATTEMPTS:-2}"
RANDOM_MIN_DISTANCE_M="${GO2_JUMP_GENERALIZATION_RANDOM_MIN_DISTANCE_M:-0.18}"
RANDOM_MAX_DISTANCE_M="${GO2_JUMP_GENERALIZATION_RANDOM_MAX_DISTANCE_M:-0.32}"
OUT_DIR="${ROOT_DIR}/reports/calibration"
RAW_REPORT_PATH="${ROOT_DIR}/reports/jump_metrics/latest_report.txt"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RAW_CSV_PATH="${OUT_DIR}/airborne_generalization_${TIMESTAMP}.csv"
SUMMARY_PATH="${OUT_DIR}/airborne_generalization_${TIMESTAMP}_summary.txt"

mkdir -p "${OUT_DIR}"

extract_metric() {
  local key="$1"
  local report_path="$2"
  awk -v key="${key}" '$1 == key ":" {print $2; exit}' "${report_path}"
}

generate_random_distances_csv() {
  local count="$1"
  local seed="$2"
  local min_distance_m="$3"
  local max_distance_m="$4"

  if [ "${count}" -le 0 ]; then
    return
  fi

  awk -v count="${count}" -v seed="${seed}" -v min_v="${min_distance_m}" -v max_v="${max_distance_m}" '
    BEGIN {
      srand(seed)
      for (i = 1; i <= count; ++i) {
        value = min_v + rand() * (max_v - min_v)
        if (i > 1) {
          printf ","
        }
        printf "%.3f", value
      }
    }
  '
}

append_unique_targets() {
  local csv_string="$1"
  local target_csv="$2"
  local -n output_array="$3"
  local value

  if [ -n "${csv_string}" ]; then
    IFS=, read -r -a values <<< "${csv_string}"
    for value in "${values[@]}"; do
      if [ -z "${value}" ]; then
        continue
      fi
      output_array+=("${value}")
    done
  fi

  if [ -n "${target_csv}" ]; then
    IFS=, read -r -a values <<< "${target_csv}"
    for value in "${values[@]}"; do
      if [ -z "${value}" ]; then
        continue
      fi
      output_array+=("${value}")
    done
  fi
}

random_targets_csv="$(generate_random_distances_csv "${RANDOM_TARGET_COUNT}" "${RANDOM_TARGET_SEED}" "${RANDOM_MIN_DISTANCE_M}" "${RANDOM_MAX_DISTANCE_M}")"
declare -a raw_targets=()
append_unique_targets "${FIXED_DISTANCES_CSV}" "${random_targets_csv}" raw_targets

declare -a TARGETS=()
declare -A seen_targets=()
for target in "${raw_targets[@]}"; do
  normalized_target="$(printf "%.3f" "${target}")"
  if [ -n "${seen_targets[${normalized_target}]:-}" ]; then
    continue
  fi
  seen_targets["${normalized_target}"]=1
  TARGETS+=("${normalized_target}")
done

apply_airborne_priority_defaults
export GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE="${GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE:-true}"

run_trial_with_retry() {
  local target="$1"
  local trial_index="$2"
  local attempts=1

  while true; do
    if "${ROOT_DIR}/scripts/docker_run_single_jump_trial.sh" "${target}"; then
      return 0
    fi

    if [ "${attempts}" -ge "${MAX_ATTEMPTS_PER_TRIAL}" ]; then
      echo "Trial failed after ${attempts} attempt(s): target=${target} m trial=${trial_index}" >&2
      return 1
    fi

    attempts=$((attempts + 1))
    echo "Retrying trial after a runtime failure: target=${target} m trial=${trial_index} attempt=${attempts}/${MAX_ATTEMPTS_PER_TRIAL}" >&2
    sleep 1
  done
}

printf "target_distance_m,trial_index,final_forward_displacement_m,distance_error_m,landing_forward_displacement_m,airborne_forward_progress_m,airborne_completion_ratio,post_landing_forward_gain_m,post_landing_completion_ratio,support_hold_forward_gain_m,release_to_complete_forward_gain_m,recovery_release_forward_speed_mps,landing_pitch_deg,final_pitch_deg,max_abs_pitch_deg,measured_flight_time_s,takeoff_speed_scale_mode,takeoff_speed_scale\n" \
  > "${RAW_CSV_PATH}"

for target in "${TARGETS[@]}"; do
  for trial_index in $(seq 1 "${TRIALS_PER_TARGET}"); do
    echo "Running airborne generalization trial: target=${target} m trial=${trial_index}/${TRIALS_PER_TARGET}"
    run_trial_with_retry "${target}" "${trial_index}"

    final_forward_displacement_m="$(extract_metric "final_forward_displacement_m" "${RAW_REPORT_PATH}")"
    distance_error_m="$(extract_metric "distance_error_m" "${RAW_REPORT_PATH}")"
    landing_forward_displacement_m="$(extract_metric "landing_forward_displacement_m" "${RAW_REPORT_PATH}")"
    airborne_forward_progress_m="$(extract_metric "airborne_forward_progress_m" "${RAW_REPORT_PATH}")"
    airborne_completion_ratio="$(extract_metric "airborne_completion_ratio" "${RAW_REPORT_PATH}")"
    post_landing_forward_gain_m="$(extract_metric "post_landing_forward_gain_m" "${RAW_REPORT_PATH}")"
    post_landing_completion_ratio="$(extract_metric "post_landing_completion_ratio" "${RAW_REPORT_PATH}")"
    support_hold_forward_gain_m="$(extract_metric "support_hold_forward_gain_m" "${RAW_REPORT_PATH}")"
    release_to_complete_forward_gain_m="$(extract_metric "release_to_complete_forward_gain_m" "${RAW_REPORT_PATH}")"
    recovery_release_forward_speed_mps="$(extract_metric "recovery_release_forward_speed_mps" "${RAW_REPORT_PATH}")"
    landing_pitch_deg="$(extract_metric "landing_pitch_deg" "${RAW_REPORT_PATH}")"
    final_pitch_deg="$(extract_metric "final_pitch_deg" "${RAW_REPORT_PATH}")"
    max_abs_pitch_deg="$(extract_metric "max_abs_pitch_deg" "${RAW_REPORT_PATH}")"
    measured_flight_time_s="$(extract_metric "measured_flight_time_s" "${RAW_REPORT_PATH}")"
    takeoff_speed_scale_mode="$(extract_metric "takeoff_speed_scale_mode" "${RAW_REPORT_PATH}")"
    takeoff_speed_scale="$(extract_metric "takeoff_speed_scale" "${RAW_REPORT_PATH}")"

    printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
      "${target}" "${trial_index}" \
      "${final_forward_displacement_m}" "${distance_error_m}" \
      "${landing_forward_displacement_m}" "${airborne_forward_progress_m}" \
      "${airborne_completion_ratio}" "${post_landing_forward_gain_m}" \
      "${post_landing_completion_ratio}" "${support_hold_forward_gain_m}" \
      "${release_to_complete_forward_gain_m}" \
      "${recovery_release_forward_speed_mps}" "${landing_pitch_deg}" \
      "${final_pitch_deg}" "${max_abs_pitch_deg}" \
      "${measured_flight_time_s}" "${takeoff_speed_scale_mode}" \
      "${takeoff_speed_scale}" >> "${RAW_CSV_PATH}"
  done
done

{
  echo "Raw generalization data: ${RAW_CSV_PATH}"
  echo "Fixed targets: ${FIXED_DISTANCES_CSV}"
  echo "Random target seed: ${RANDOM_TARGET_SEED}"
  echo "Random targets: ${random_targets_csv:-none}"
  echo
  echo "Per-target averages:"
  awk -F, '
    NR == 1 { next }
    {
      key = $1
      count[key] += 1
      final[key] += $3
      error[key] += $4
      landing[key] += $5
      airborne[key] += $6
      airborne_ratio[key] += $7
      post[key] += $8
      post_ratio[key] += $9
      support_hold[key] += $10
      release[key] += $11
      release_speed[key] += $12
      landing_pitch[key] += $13
      final_pitch[key] += $14
      max_pitch[key] += $15
      flight_time[key] += $16
    }
    END {
      printf "target_m,trials,avg_final_m,avg_abs_error_m,avg_landing_m,avg_airborne_m,avg_airborne_ratio,avg_post_m,avg_post_ratio,avg_support_hold_m,avg_release_to_complete_m,avg_recovery_release_forward_speed_mps,avg_landing_pitch_deg,avg_final_pitch_deg,avg_max_abs_pitch_deg,avg_flight_time_s\n"
      for (key in count) {
        avg_error = error[key] / count[key]
        if (avg_error < 0.0) {
          avg_error = -avg_error
        }
        printf "%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               key + 0.0, count[key], final[key] / count[key], avg_error,
               landing[key] / count[key], airborne[key] / count[key],
               airborne_ratio[key] / count[key], post[key] / count[key],
               post_ratio[key] / count[key], support_hold[key] / count[key],
               release[key] / count[key], release_speed[key] / count[key],
               landing_pitch[key] / count[key], final_pitch[key] / count[key],
               max_pitch[key] / count[key], flight_time[key] / count[key]
      }
    }
  ' "${RAW_CSV_PATH}" | sort -t, -k1,1n
  echo
  echo "Overall averages:"
  awk -F, '
    NR == 1 { next }
    {
      count += 1
      error += ($4 < 0.0 ? -$4 : $4)
      airborne_ratio += $7
      post_ratio += $9
      support_hold += $10
      release += $11
      release_speed += $12
      landing_pitch += $13
      final_pitch += $14
      max_pitch += $15
    }
    END {
      if (count == 0) {
        exit
      }
      printf "trials,avg_abs_error_m,avg_airborne_ratio,avg_post_ratio,avg_support_hold_m,avg_release_to_complete_m,avg_recovery_release_forward_speed_mps,avg_landing_pitch_deg,avg_final_pitch_deg,avg_max_abs_pitch_deg\n"
      printf "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             count, error / count, airborne_ratio / count, post_ratio / count,
             support_hold / count, release / count, release_speed / count,
             landing_pitch / count, final_pitch / count, max_pitch / count
    }
  ' "${RAW_CSV_PATH}"
} | tee "${SUMMARY_PATH}"

echo
echo "Wrote airborne generalization summary to ${SUMMARY_PATH}"
