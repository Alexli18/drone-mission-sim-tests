#!/usr/bin/env bash
# Entrypoint of the ROS images: source ROS 2 Humble, px4_msgs (/ws_base) and,
# in the thin image, the survey packages (/ws), then run the command.
set -eo pipefail  # no -u: the ROS setup scripts read unset variables

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source /ws_base/install/setup.bash
if [[ -f /ws/install/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /ws/install/setup.bash
fi
exec "$@"
