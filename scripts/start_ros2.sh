#!/usr/bin/env bash
# Start one ROS container running "ros2 launch" and wait until the launch
# logs "SURVEY READY" and the rosbridge port accepts connections. Prints the
# container name on success.
#
# Usage: start_ros2.sh --image <tag> --name <name> --repo-dir <dir> --out-dir <dir>
#          [--in-dir <dir>] --launch "<PKG FILE>|<path>" [--launch-arg NAME:=VALUE ...]
#          --domain-id <n> --localhost-only 0|1 --rosbridge-port <p> --timeout <s>
set -euo pipefail

READY_LINE="SURVEY READY"

usage() {
  sed -n '6,8p' "$0" >&2
  exit 2
}

image="" name="" repo_dir="" out_dir="" in_dir="" launch="" domain_id=""
localhost_only="" rosbridge_port="" timeout=""
launch_args=()
while [[ $# -gt 0 ]]; do
  [[ $# -ge 2 ]] || usage
  case "$1" in
    --image) image="$2" ;;
    --name) name="$2" ;;
    --repo-dir) repo_dir="$2" ;;
    --out-dir) out_dir="$2" ;;
    --in-dir) in_dir="$2" ;;
    --launch) launch="$2" ;;
    --launch-arg) launch_args+=("$2") ;;
    --domain-id) domain_id="$2" ;;
    --localhost-only) localhost_only="$2" ;;
    --rosbridge-port) rosbridge_port="$2" ;;
    --timeout) timeout="$2" ;;
    *) usage ;;
  esac
  shift 2
done
for value in "$image" "$name" "$repo_dir" "$out_dir" "$launch" "$domain_id" \
             "$localhost_only" "$rosbridge_port" "$timeout"; do
  [[ -n "$value" ]] || usage
done
[[ "$localhost_only" == 0 || "$localhost_only" == 1 ]] || usage

for dir in "$repo_dir" "$out_dir" ${in_dir:+"$in_dir"}; do
  [[ -d "$dir" ]] || { echo "not a directory: $dir" >&2; exit 2; }
done
repo_dir="$(realpath "$repo_dir")"
out_dir="$(realpath "$out_dir")"
mounts=(
  --mount "type=bind,source=$repo_dir,target=/work,readonly"
  --mount "type=bind,source=$out_dir,target=/out"
)
if [[ -n "$in_dir" ]]; then
  mounts+=(--mount "type=bind,source=$(realpath "$in_dir"),target=/in,readonly")
fi

# On Linux, run as the caller so the output directory stays ours (as for PX4).
user_args=()
if [[ "$(uname -s)" == Linux ]]; then
  user_args=(--user "$(id -u):$(id -g)")
fi

# "PKG FILE" becomes two words; a path stays one.
read -r -a launch_words <<<"$launch"

# The image's entrypoint sources ROS and the workspaces, then runs the command.
docker run -d --pull never --network host --name "$name" \
  ${user_args[@]+"${user_args[@]}"} \
  -e ROS_DOMAIN_ID="$domain_id" \
  -e ROS_LOCALHOST_ONLY="$localhost_only" \
  -e ROS_HOME=/out/ros -e ROS_LOG_DIR=/out/ros/log -e HOME=/out/ros \
  -e REPO_ROOT=/work \
  "${mounts[@]}" \
  "$image" ros2 launch "${launch_words[@]}" ${launch_args[@]+"${launch_args[@]}"} >/dev/null

fail() {
  echo "$1; last log lines:" >&2
  docker logs --tail 50 "$name" >&2 2>&1 || true
  exit 1
}

deadline=$((SECONDS + timeout))
ready=0
while true; do
  if [[ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" != true ]]; then
    fail "container $name stopped during start-up"
  fi
  if (( ! ready )) && docker logs "$name" 2>&1 | grep -F "$READY_LINE" >/dev/null; then
    ready=1
  fi
  # bash's /dev/tcp: no nc or curl needed in the runner or on the Mac.
  if (( ready )) && (exec 3<>"/dev/tcp/127.0.0.1/$rosbridge_port") 2>/dev/null; then
    break
  fi
  if (( SECONDS >= deadline )); then
    if (( ready )); then
      fail "rosbridge port $rosbridge_port not reachable within ${timeout} s"
    fi
    fail "\"$READY_LINE\" not logged within ${timeout} s"
  fi
  sleep 0.5
done
echo "$name"
