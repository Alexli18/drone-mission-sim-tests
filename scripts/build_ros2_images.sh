#!/usr/bin/env bash
# Build the ROS images natively for this machine's architecture, tagged from
# config.yaml: "base" (ros.base_image_tag, docker/ros2-survey-base) and "thin"
# (ros.image_tag, docker/ros2-survey on top of the local base tag, built from the
# repository root). Used by the ROS image action and locally (Mac, Linux).
#
# Usage: build_ros2_images.sh base|thin
set -euo pipefail

[[ $# -eq 1 && ( "$1" == base || "$1" == thin ) ]] || { sed -n '8p' "$0" >&2; exit 2; }

cd "$(dirname "$0")/.."
base="$(python -m framework.config ros-base-tag)"
case "$1" in
  base)
    tag="$base"
    docker build -t "$tag" docker/ros2-survey-base
    ;;
  thin)
    tag="$(python -m framework.config ros-tag)"
    docker build --build-arg BASE="$base" -t "$tag" -f docker/ros2-survey/Dockerfile .
    ;;
esac
echo "built $tag in ${SECONDS} s"
