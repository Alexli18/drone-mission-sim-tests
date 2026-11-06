# check=skip=InvalidDefaultArgInFrom
# Thin image: the survey ROS packages on top of the base image. Rebuilt on every CI
# run (not cached), so code changes never rebuild px4_msgs. Built from the repository
# root by scripts/build_ros2_images.sh thin.

# No default: the build script passes ros.base_image_tag (the locally loaded tag).
ARG BASE
FROM ${BASE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# sensor_bridge_msgs, survey_demo_msgs, sensor_bridge and survey_demo.
COPY ros2_ws/src /ws/src
RUN cd /ws \
    && source /opt/ros/humble/setup.bash \
    && source /ws_base/install/setup.bash \
    && colcon build --install-base /ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /ws/build /ws/log

# Every node imports at build time, so a broken import fails the build, not a test.
RUN /survey_entrypoint.sh python3 -c \
    "import sensor_bridge.georeference_node, survey_demo.toy_sensor_node, survey_demo.marker_detector_node, survey_demo.bag_player, survey_demo.ready_gate"
