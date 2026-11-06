#!/usr/bin/env bash
# Stop a ROS container with SIGINT (the launch passes it on, so the bag
# recorder closes the bag and writes metadata.yaml), save its log and remove
# it. Succeeds if the container is already gone.
#
# Usage: stop_ros2.sh --name <name> --log-file <path>
set -uo pipefail

STOP_WAIT_S=20

name="" log_file=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --name) name="$2" ;;
    --log-file) log_file="$2" ;;
    *) sed -n '6p' "$0" >&2; exit 2 ;;
  esac
  shift 2
done
[[ -n "$name" && -n "$log_file" ]] || { sed -n '6p' "$0" >&2; exit 2; }

docker inspect "$name" >/dev/null 2>&1 || exit 0
docker kill --signal INT "$name" >/dev/null 2>&1 || true
# Poll instead of timeout(1), which macOS lacks.
deadline=$((SECONDS + STOP_WAIT_S))
while [[ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" == true ]] \
      && (( SECONDS < deadline )); do
  sleep 0.5
done
mkdir -p "$(dirname "$log_file")"
docker logs "$name" >"$log_file" 2>&1 || true
docker rm -f "$name" >/dev/null 2>&1 || true
