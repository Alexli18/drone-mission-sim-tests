# check=skip=InvalidDefaultArgInFrom
# PX4 SITL image: the pinned px4io/px4-sitl image (config.yaml px4.image and
# px4.digest, passed as BASE) with its px4 binary rebuilt from the same source,
# v1.18.0-rc1 (fca3df865a), plus lockstep_scheduler.patch. Built natively per
# architecture by scripts/build_px4_image.sh; the CI cache key is the pinned
# digest plus the hash of this directory. Background: docs/qa-notes.md, "PX4
# lockstep scheduler crash".
#
# lockstep_scheduler.patch is PX4 PR #29061 (gh pr diff 29061), the open
# release/1.18 backport of 27a21488b2 and 1fd3a84780 from main, for PX4 issue
# #29060 (the text after the line numbers of three hunk headers is dropped; git
# apply ignores it): in rc1, LockstepScheduler::set_absolute_time() can lock and broadcast
# a waiter's stack-local mutex and condition variable (px4_poll's semaphore)
# after the waiter has returned, which aborts PX4 with a glibc assertion
# (tpp.c:83 or pthread_mutex_lock.c:94), crashes it or freezes its clock. Drop
# this image once the pinned PX4 release contains the fix.
ARG BASE

# Built on the pinned image itself (Ubuntu 24.04), so the binary links against
# the same glibc and libstdc++ it runs with.
# hadolint ignore=DL3006
FROM ${BASE} AS build
ARG PX4_TAG=v1.18.0-rc1
ARG PX4_COMMIT=fca3df865af36124a28c9d607e850f111dbaaea9
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends sudo git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 -b "$PX4_TAG" https://github.com/PX4/PX4-Autopilot.git /px4 \
    && test "$(git -C /px4 rev-parse HEAD)" = "$PX4_COMMIT"
# PX4's own deb build: Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools with
# RUNS_IN_DOCKER=true, then make px4_sitl_sih
# (.github/workflows/build_deb_package.yml at the tag).
RUN RUNS_IN_DOCKER=true /px4/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
    && rm -rf /var/lib/apt/lists/*
COPY lockstep_scheduler.patch /tmp/
WORKDIR /px4
RUN git apply /tmp/lockstep_scheduler.patch \
    && make px4_sitl_sih \
    && strip build/px4_sitl_sih/bin/px4

# hadolint ignore=DL3006
FROM ${BASE}
COPY --from=build /px4/build/px4_sitl_sih/bin/px4 /opt/px4/bin/px4
