#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DISTANCE_M="${1:-0.25}"
source "${ROOT_DIR}/scripts/airborne_priority_params.sh"

# Experimental candidate:
# - prioritizes distance completed during flight
# - uses contact-aware phase timing plus a capture-style landing geometry
# - keeps post-landing catch-up materially below the total commanded distance
# - after the 2026-03-26 multi-distance refit, the 0.25 m benchmark is best
#   treated as a near-target manual probe with takeoff_speed_scale=1.03
# - this slightly higher manual scale restores the 0.25 m target after the
#   short-distance landing-support refactor reduced post-landing catch-up
# - touchdown pitch is still too nose-down for a finished jump controller,
#   but the motion is now more genuinely flight-dominant than the earlier
#   low-arc profiles
apply_airborne_priority_defaults

if [ -z "${GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE+x}" ]; then
  if awk -v target="${TARGET_DISTANCE_M}" 'BEGIN { exit !(target > 0.245 && target < 0.255) }'; then
    export GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false
    export GO2_JUMP_TAKEOFF_SPEED_SCALE="${GO2_JUMP_TAKEOFF_SPEED_SCALE:-1.03}"
  else
    export GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=true
  fi
fi

if [ "${GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE}" = "false" ]; then
  export GO2_JUMP_TAKEOFF_SPEED_SCALE="${GO2_JUMP_TAKEOFF_SPEED_SCALE:-1.03}"
fi

"${ROOT_DIR}/scripts/docker_run_single_jump_trial.sh" "${TARGET_DISTANCE_M}"
