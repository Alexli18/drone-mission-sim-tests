#!/usr/bin/env bash
# Build the PX4 SITL image the tests run (python -m framework.config local-tag):
# the pinned px4io/px4-sitl image (config.yaml px4.image and px4.digest) with
# its px4 binary rebuilt from the same PX4 source plus the lockstep scheduler
# fix (docker/px4-sitl/). Pulls the pinned image if it is not present. Built
# natively for this machine's architecture; used by the PX4 image action and
# locally (Linux, Mac).
#
# Usage: build_px4_image.sh
set -euo pipefail

[[ $# -eq 0 ]] || { sed -n '9p' "$0" >&2; exit 2; }

cd "$(dirname "$0")/.."
base="$(python -m framework.config image)"
tag="$(python -m framework.config local-tag)"
docker image inspect "$base" >/dev/null 2>&1 || docker pull "$base"
docker build --build-arg BASE="$base" -t "$tag" docker/px4-sitl
echo "built $tag in ${SECONDS} s"
