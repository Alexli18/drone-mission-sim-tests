#!/usr/bin/env bash
# Save a SITL container's log, stop it (PX4 gets 10 s to close its ULog file)
# and remove it. Succeeds if the container is already gone.
#
# Usage: stop_sitl.sh --name <name> --log-file <path>
set -uo pipefail

name="" log_file=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --name) name="$2" ;;
    --log-file) log_file="$2" ;;
    *) sed -n '5p' "$0" >&2; exit 2 ;;
  esac
  shift 2
done
[[ -n "$name" && -n "$log_file" ]] || { sed -n '5p' "$0" >&2; exit 2; }

docker inspect "$name" >/dev/null 2>&1 || exit 0
mkdir -p "$(dirname "$log_file")"
docker logs "$name" >"$log_file" 2>&1 || true
docker stop -t 10 "$name" >/dev/null 2>&1 || true
docker rm -f "$name" >/dev/null 2>&1 || true
