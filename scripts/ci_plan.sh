#!/usr/bin/env bash
# Decide which sim-tests jobs run; prints key=value lines for $GITHUB_OUTPUT:
#   run     false only for a nightly run when main had no commit in 24 hours
#   sim     the flight-test jobs (basic, survey, failsafes)
#   ros     the ROS jobs (ros-image and ros-survey)
#   full    every ROS survey scenario, else the smoke test (the main scenario)
#   weekly  the weekly 3-seed ROS run (weekly-only scenarios included)
# Push to main runs the unit job alone and does not call this script. A pull
# request runs sim and ros only when it changes their paths (SIM_PATHS,
# ROS_PATHS; docs/qa-notes.md, "CI cost control"), and every ROS scenario only
# with the full-ci label (FULL_LABEL=true).
#
# Manual dispatch runs everything but the weekly-only scenarios; with DISPATCH_WEEKLY=true it
# runs the weekly plan instead (the ROS survey, three seeds, every scenario).
#
# Usage: ci_plan.sh EVENT_NAME BASE_SHA SCHEDULE FULL_LABEL [DISPATCH_WEEKLY]
set -euo pipefail

# Inputs of the flight-test jobs. Conservative: every scenario, every framework file.
SIM_PATHS='^(framework/|tests/(conftest\.py|test_smoke\.py|test_basic_flight\.py|test_survey_mission\.py|test_failsafes\.py)$|scenarios/|docker/px4-sitl/|config\.yaml$|scripts/(start_sitl\.sh|stop_sitl\.sh|build_px4_image\.sh|summarize\.py|ci_plan\.sh)$|\.github/(workflows/sim-tests\.yml|actions/px4-image/)|pyproject\.toml$)'
# Inputs of the ROS jobs: everything the line-survey test and its containers read.
ROS_PATHS='^(ros2_ws/|framework/|tests/|scenarios/|scripts/|docker/|config\.yaml$|pyproject\.toml$|\.github/)'
NIGHTLY='17 2 * * *'
WEEKLY='41 4 * * 1'

[[ $# -eq 4 || $# -eq 5 ]] || { grep -m1 '^# Usage' "$0" >&2; exit 2; }
event="$1" base="$2" schedule="$3" label="$4" dispatch_weekly="${5:-false}"

plan() { printf 'run=%s\nsim=%s\nros=%s\nfull=%s\nweekly=%s\n' "$@"; }

case "$event" in
  pull_request)
    [[ -n "$base" ]] || { echo "pull_request without a base SHA" >&2; exit 2; }
    changed="$(git diff --name-only "$base"...HEAD)"
    if [[ "$label" == true ]]; then
      plan true true true true false
      exit 0
    fi
    sim=false ros=false
    if grep -Eq "$SIM_PATHS" <<<"$changed"; then sim=true; fi
    if grep -Eq "$ROS_PATHS" <<<"$changed"; then ros=true; fi
    plan true "$sim" "$ros" false false
    ;;
  schedule)
    if [[ "$schedule" == "$WEEKLY" ]]; then
      plan true false true true true
    elif [[ "$schedule" == "$NIGHTLY" ]]; then
      # Skip the night when nothing landed on main (the checkout is main's tip).
      if [[ -n "$(git log --since='24 hours ago' --format=%H -1 HEAD)" ]]; then
        plan true true true true false
      else
        plan false false false false false
      fi
    else
      echo "unknown schedule: $schedule" >&2
      exit 2
    fi
    ;;
  *)
    # workflow_dispatch: everything except the weekly-only scenarios, or the weekly plan.
    if [[ "$dispatch_weekly" == true ]]; then
      plan true false true true true
    else
      plan true true true true false
    fi
    ;;
esac
