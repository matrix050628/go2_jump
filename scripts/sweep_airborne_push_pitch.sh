#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DISTANCE_M="${1:-${GO2_JUMP_AIRBORNE_SWEEP_DISTANCE:-0.25}}"
FRONT_SCALES_CSV="${2:-${GO2_JUMP_SWEEP_PUSH_FRONT_TAU_SCALES:-0.88,0.92,0.96}}"
REAR_SCALES_CSV="${3:-${GO2_JUMP_SWEEP_PUSH_REAR_TAU_SCALES:-1.08,1.12,1.16}}"
PUSH_PITCH_TARGETS_CSV="${4:-${GO2_JUMP_SWEEP_PUSH_PITCH_TARGETS:--8.0,-5.0,-2.0}}"
FLIGHT_PITCH_TARGETS_CSV="${5:-${GO2_JUMP_SWEEP_FLIGHT_PITCH_TARGETS:--2.0,0.0}}"
TRIALS_PER_SETTING="${6:-${GO2_JUMP_SWEEP_TRIALS:-1}}"
FINAL_ERROR_GATE_M="${GO2_JUMP_AIRBORNE_FINAL_ERROR_GATE_M:-0.06}"
OUT_DIR="${ROOT_DIR}/reports/calibration"
RAW_REPORT_PATH="${ROOT_DIR}/reports/jump_metrics/latest_report.txt"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RAW_CSV_PATH="${OUT_DIR}/airborne_sweep_${TIMESTAMP}.csv"
AVG_CSV_PATH="${OUT_DIR}/airborne_sweep_${TIMESTAMP}_avg.csv"
SUMMARY_PATH="${OUT_DIR}/airborne_sweep_${TIMESTAMP}_summary.txt"

mkdir -p "${OUT_DIR}"

extract_metric() {
  local key="$1"
  local report_path="$2"
  awk -v key="${key}" '$1 == key ":" {print $2; exit}' "${report_path}"
}

IFS=, read -r -a FRONT_SCALES <<< "${FRONT_SCALES_CSV}"
IFS=, read -r -a REAR_SCALES <<< "${REAR_SCALES_CSV}"
IFS=, read -r -a PUSH_PITCH_TARGETS <<< "${PUSH_PITCH_TARGETS_CSV}"
IFS=, read -r -a FLIGHT_PITCH_TARGETS <<< "${FLIGHT_PITCH_TARGETS_CSV}"

printf "target_distance_m,push_front_tau_scale,push_rear_tau_scale,push_pitch_target_deg,flight_pitch_target_deg,trial_index,final_forward_displacement_m,distance_error_m,landing_forward_displacement_m,airborne_forward_progress_m,airborne_completion_ratio,post_landing_forward_gain_m,post_landing_completion_ratio,max_airborne_forward_displacement_m,max_height_above_start_m,max_abs_pitch_deg,takeoff_pitch_deg,landing_pitch_deg,measured_flight_time_s\n" \
  > "${RAW_CSV_PATH}"

for front_scale in "${FRONT_SCALES[@]}"; do
  for rear_scale in "${REAR_SCALES[@]}"; do
    for push_pitch_target in "${PUSH_PITCH_TARGETS[@]}"; do
      for flight_pitch_target in "${FLIGHT_PITCH_TARGETS[@]}"; do
        for trial_index in $(seq 1 "${TRIALS_PER_SETTING}"); do
          echo "Running airborne sweep trial: target=${TARGET_DISTANCE_M} m front=${front_scale} rear=${rear_scale} push_pitch=${push_pitch_target} flight_pitch=${flight_pitch_target} trial=${trial_index}/${TRIALS_PER_SETTING}"
          GO2_JUMP_PUSH_FRONT_TAU_SCALE="${front_scale}" \
          GO2_JUMP_PUSH_REAR_TAU_SCALE="${rear_scale}" \
          GO2_JUMP_PUSH_PITCH_TARGET_DEG="${push_pitch_target}" \
          GO2_JUMP_FLIGHT_PITCH_TARGET_DEG="${flight_pitch_target}" \
            "${ROOT_DIR}/scripts/docker_run_single_jump_trial.sh" "${TARGET_DISTANCE_M}"

          final_forward_displacement_m="$(extract_metric "final_forward_displacement_m" "${RAW_REPORT_PATH}")"
          distance_error_m="$(extract_metric "distance_error_m" "${RAW_REPORT_PATH}")"
          landing_forward_displacement_m="$(extract_metric "landing_forward_displacement_m" "${RAW_REPORT_PATH}")"
          airborne_forward_progress_m="$(extract_metric "airborne_forward_progress_m" "${RAW_REPORT_PATH}")"
          airborne_completion_ratio="$(extract_metric "airborne_completion_ratio" "${RAW_REPORT_PATH}")"
          post_landing_forward_gain_m="$(extract_metric "post_landing_forward_gain_m" "${RAW_REPORT_PATH}")"
          post_landing_completion_ratio="$(extract_metric "post_landing_completion_ratio" "${RAW_REPORT_PATH}")"
          max_airborne_forward_displacement_m="$(extract_metric "max_airborne_forward_displacement_m" "${RAW_REPORT_PATH}")"
          max_height_above_start_m="$(extract_metric "max_height_above_start_m" "${RAW_REPORT_PATH}")"
          max_abs_pitch_deg="$(extract_metric "max_abs_pitch_deg" "${RAW_REPORT_PATH}")"
          takeoff_pitch_deg="$(extract_metric "takeoff_pitch_deg" "${RAW_REPORT_PATH}")"
          landing_pitch_deg="$(extract_metric "landing_pitch_deg" "${RAW_REPORT_PATH}")"
          measured_flight_time_s="$(extract_metric "measured_flight_time_s" "${RAW_REPORT_PATH}")"

          printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
            "${TARGET_DISTANCE_M}" "${front_scale}" "${rear_scale}" \
            "${push_pitch_target}" "${flight_pitch_target}" "${trial_index}" \
            "${final_forward_displacement_m}" "${distance_error_m}" \
            "${landing_forward_displacement_m}" "${airborne_forward_progress_m}" \
            "${airborne_completion_ratio}" "${post_landing_forward_gain_m}" \
            "${post_landing_completion_ratio}" \
            "${max_airborne_forward_displacement_m}" \
            "${max_height_above_start_m}" "${max_abs_pitch_deg}" \
            "${takeoff_pitch_deg}" "${landing_pitch_deg}" \
            "${measured_flight_time_s}" >> "${RAW_CSV_PATH}"
        done
      done
    done
  done
done

awk -F, -v target="${TARGET_DISTANCE_M}" '
  NR == 1 { next }
  {
    key = $2 FS $3 FS $4 FS $5
    count[key] += 1
    final[key] += $7
    landing[key] += $9
    airborne[key] += $10
    airborne_ratio[key] += $11
    post[key] += $12
    post_ratio[key] += $13
    airborne_max[key] += $14
    height[key] += $15
    pitch[key] += $16
    takeoff_pitch[key] += $17
    landing_pitch[key] += $18
    flight_time[key] += $19
  }
  END {
    printf "push_front_tau_scale,push_rear_tau_scale,push_pitch_target_deg,flight_pitch_target_deg,trials,avg_final_m,avg_abs_final_error_m,avg_landing_m,avg_airborne_m,avg_airborne_completion_ratio,avg_post_landing_m,avg_post_landing_completion_ratio,avg_max_airborne_m,avg_height_m,avg_max_abs_pitch_deg,avg_takeoff_pitch_deg,avg_landing_pitch_deg,avg_measured_flight_time_s\n"
    for (key in count) {
      split(key, parts, FS)
      avg_final = final[key] / count[key]
      abs_error = avg_final - target
      if (abs_error < 0.0) {
        abs_error = -abs_error
      }
      printf "%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             parts[1] + 0.0, parts[2] + 0.0, parts[3] + 0.0, parts[4] + 0.0,
             count[key], avg_final, abs_error,
             landing[key] / count[key], airborne[key] / count[key],
             airborne_ratio[key] / count[key], post[key] / count[key],
             post_ratio[key] / count[key], airborne_max[key] / count[key],
             height[key] / count[key], pitch[key] / count[key],
             takeoff_pitch[key] / count[key], landing_pitch[key] / count[key],
             flight_time[key] / count[key]
    }
  }
' "${RAW_CSV_PATH}" > "${AVG_CSV_PATH}"

{
  echo "Raw sweep data: ${RAW_CSV_PATH}"
  echo "Averaged sweep data: ${AVG_CSV_PATH}"
  echo
  echo "Top settings by average airborne forward progress:"
  {
    head -n 1 "${AVG_CSV_PATH}"
    tail -n +2 "${AVG_CSV_PATH}" | sort -t, -k9,9gr -k7,7g | head -n 10
  }
  echo
  echo "Top settings by average airborne completion ratio:"
  {
    head -n 1 "${AVG_CSV_PATH}"
    tail -n +2 "${AVG_CSV_PATH}" | sort -t, -k10,10gr -k7,7g | head -n 10
  }
  echo
  echo "Best setting within average final-error gate <= ${FINAL_ERROR_GATE_M} m:"
  FILTERED_ROWS="$(awk -F, -v gate="${FINAL_ERROR_GATE_M}" 'NR > 1 && ($7 + 0.0) <= gate { print }' "${AVG_CSV_PATH}")"
  if [ -n "${FILTERED_ROWS}" ]; then
    echo "$(head -n 1 "${AVG_CSV_PATH}")"
    printf "%s\n" "${FILTERED_ROWS}" | sort -t, -k9,9gr -k10,10gr | head -n 1
  else
    echo "No setting satisfied the final-error gate."
  fi
} | tee "${SUMMARY_PATH}"

echo
echo "Wrote airborne sweep summary to ${SUMMARY_PATH}"
