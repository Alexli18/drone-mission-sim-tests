# ROS base image: ROS 2 Humble, Micro XRCE-DDS Agent, px4_msgs and rosbridge.
# Built natively per architecture (amd64 in CI, arm64 on Apple Silicon) by
# scripts/build_ros2_images.sh base; the CI cache key is the hash of this
# directory, so every pin lives here.

# Multi-arch index digest of ros:humble-ros-base, recorded 2026-11-04.
# Per platform: linux/amd64 sha256:a3f4de78e42cca693a543ece5798aedcf1cae0171fe71c334a047332cc72ef93,
# linux/arm64/v8 sha256:48435c01f2876ff519a0d07edf8dd3ab29a8106c907965c64385a14c76470d6b.
FROM ros:humble-ros-base@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855

# Agent tag; the commit guards against a moved tag.
ARG XRCE_AGENT_TAG=v2.4.3
ARG XRCE_AGENT_COMMIT=73622810d984349b80bbac0ef55fc0b694d62222
# The agent's superbuild fetches Fast-CDR and Fast-DDS by release branch
# (2.2.x, 2.14.x); these release tags of those branches replace them.
ARG FASTCDR_TAG=v2.2.8
ARG FASTDDS_TAG=v2.14.7
# px4_msgs branch release/1.18, commit "Update to PX4 fca3df8": the
# messages of PX4 v1.18.0-rc1 (config.yaml px4.image), identical file for file.
ARG PX4_MSGS_COMMIT=935c120c51916b55dc62cce0b9a406178cde6e97

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The ROS apt repository keeps only the newest build of each package, so apt
# versions follow the base image digest and the day of the build.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-server git cmake build-essential python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Built without the ROS environment, so the superbuild uses its own Fast-DDS
# and not Humble's (different sonames; both coexist).
RUN git clone --depth 1 -b "$XRCE_AGENT_TAG" \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cd /tmp/agent \
    && test "$(git rev-parse HEAD)" = "$XRCE_AGENT_COMMIT" \
    && sed -i -e "s/set(_fastcdr_tag 2.2.x)/set(_fastcdr_tag $FASTCDR_TAG)/" \
              -e "s/set(_fastdds_tag 2.14.x)/set(_fastdds_tag $FASTDDS_TAG)/" CMakeLists.txt \
    && grep -F "set(_fastcdr_tag $FASTCDR_TAG)" CMakeLists.txt \
    && grep -F "set(_fastdds_tag $FASTDDS_TAG)" CMakeLists.txt \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build \
    && ldconfig \
    && rm -rf /tmp/agent \
    && command -v MicroXRCEAgent

RUN mkdir -p /ws_base/src/px4_msgs \
    && cd /ws_base/src/px4_msgs \
    && git init -q \
    && git remote add origin https://github.com/PX4/px4_msgs.git \
    && git fetch -q --depth 1 origin "$PX4_MSGS_COMMIT" \
    && git checkout -q FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "$PX4_MSGS_COMMIT" \
    && cd /ws_base \
    && source /opt/ros/humble/setup.bash \
    && colcon build --install-base /ws_base/install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /ws_base/build /ws_base/log

# numpy below 2: Humble's message extensions use the numpy 1.x C API. The apt
# PyYAML has no pip record to uninstall.
RUN pip install --no-cache-dir numpy==1.26.4 \
    && pip install --no-cache-dir --ignore-installed PyYAML==6.0.3

COPY survey_entrypoint.sh /survey_entrypoint.sh
ENTRYPOINT ["/survey_entrypoint.sh"]
CMD ["bash"]
