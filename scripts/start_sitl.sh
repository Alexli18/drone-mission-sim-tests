#!/usr/bin/env bash
# Start one PX4 SITL (SIH) container and wait until PX4's startup script has
# finished. Prints the container name on success.
#
# Usage: start_sitl.sh --image <local-tag> --name <name> --log-dir <empty dir>
#          --model <model> --speed-factor <n> --home-lat <x> --home-lon <y>
#          --home-alt <z> --timeout <s> [--param NAME=VALUE ...]
#          [--env NAME=VALUE ...] [--dds-own-ip]
#
# --env passes an environment variable to the container (ROS tests: ROS_DOMAIN_ID, which
# PX4's rcS copies into UXRCE_DDS_DOM_ID after the --param overrides).
# --dds-own-ip (ROS tests only; Docker Desktop only, a no-op on Linux): the image's
# entrypoint points the uXRCE-DDS client at host.docker.internal, and UDP replies
# over 127.0.0.1 are lost there (docs/qa-notes.md, "DDS"), so the client is pointed at the
# container's own address instead (a second -h; the last one wins).
set -euo pipefail

# Found by spike S1 (CI run 41707210020).
STARTUP_LINE="Startup script returned successfully"

usage() {
  sed -n '2,15p' "$0" >&2
  exit 2
}

image="" name="" log_dir="" model="" speed_factor="" timeout=""
home_lat="" home_lon="" home_alt=""
params=()
env_args=()
dds_own_ip=0
while [[ $# -gt 0 ]]; do
  if [[ "$1" == --dds-own-ip ]]; then
    dds_own_ip=1
    shift
    continue
  fi
  [[ $# -ge 2 ]] || usage
  case "$1" in
    --image) image="$2" ;;
    --name) name="$2" ;;
    --log-dir) log_dir="$2" ;;
    --model) model="$2" ;;
    --speed-factor) speed_factor="$2" ;;
    --home-lat) home_lat="$2" ;;
    --home-lon) home_lon="$2" ;;
    --home-alt) home_alt="$2" ;;
    --timeout) timeout="$2" ;;
    --param) params+=(-e "PX4_PARAM_$2") ;;
    --env) env_args+=(-e "$2") ;;
    *) usage ;;
  esac
  shift 2
done
for value in "$image" "$name" "$log_dir" "$model" "$speed_factor" "$timeout" \
             "$home_lat" "$home_lon" "$home_alt"; do
  [[ -n "$value" ]] || usage
done

if [[ ! -d "$log_dir" || -n "$(ls -A "$log_dir")" ]]; then
  echo "log dir must be an existing, empty directory: $log_dir" >&2
  exit 2
fi
log_dir="$(realpath "$log_dir")"

# On Linux, run as the caller so the mounted data dir stays ours (the whole
# XDG_DATA_HOME is mounted, so every folder PX4 creates belongs to us). Docker
# Desktop (macOS) maps mount ownership itself, and the image's entrypoint must
# run as root there: it rewrites PX4's MAVLink target to host.docker.internal.
user_args=()
if [[ "$(uname -s)" == Linux ]]; then
  user_args=(--user "$(id -u):$(id -g)")
fi

# Docker Desktop workaround (docs/qa-notes.md, "DDS"; see --dds-own-ip): add
# "-h <own IP>" to rcS's DDS client line, then run the image's entrypoint, which inserts its own
# "-h <host.docker.internal>" after "-t udp"; the last -h wins. The check for
# host.docker.internal is the entrypoint's own test for Docker Desktop.
# shellcheck disable=SC2016  # expanded by the container's shell
DDS_OWN_IP_PATCH='
if getent ahostsv4 host.docker.internal >/dev/null 2>&1; then
  ip=$(getent ahostsv4 "$(hostname)" | awk "/STREAM/ {print \$1; exit}")
  sed -i "s|^uxrce_dds_client start -t udp .*|& -h $ip|" /opt/px4/etc/init.d-posix/rcS
fi
exec /opt/px4/bin/px4-entrypoint.sh'
entrypoint_args=("$image")
if (( dds_own_ip )) && [[ "$(uname -s)" != Linux ]]; then
  entrypoint_args=(--entrypoint sh "$image" -c "$DDS_OWN_IP_PATCH")
fi

# -t gives PX4's shell a TTY; without one it busy-loops on its prompt (spike S1).
docker run -d -t --pull never --network host --name "$name" \
  ${user_args[@]+"${user_args[@]}"} \
  -e XDG_DATA_HOME=/px4data \
  -e PX4_SIM_MODEL="$model" \
  -e PX4_SIM_SPEED_FACTOR="$speed_factor" \
  -e PX4_HOME_LAT="$home_lat" -e PX4_HOME_LON="$home_lon" -e PX4_HOME_ALT="$home_alt" \
  ${params[@]+"${params[@]}"} \
  ${env_args[@]+"${env_args[@]}"} \
  --mount "type=bind,source=$log_dir,target=/px4data" \
  "${entrypoint_args[@]}" >/dev/null

deadline=$((SECONDS + timeout))
until docker logs "$name" 2>&1 | grep -F "$STARTUP_LINE" >/dev/null; do
  if (( SECONDS >= deadline )); then
    echo "PX4 did not finish its startup script within ${timeout} s; last log lines:" >&2
    docker logs --tail 50 "$name" >&2 2>&1 || true
    exit 1
  fi
  sleep 0.5
done
echo "$name"
