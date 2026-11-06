"""The toy survey sensor (simulation only): a thin wrapper over ``survey_demo.sensor_sim``.

Driven by the arriving PX4 poses in true (simulated) time: the pose callback only
appends the pose to the history; a wall-clock timer (``ADVANCE_S``) and ``/survey/stop``
advance the sensor to the newest pose time. Emits nothing until ``/survey/ready`` has
arrived (``wait_for_ready``); its start time is the first pose time after that.
Publishes ``/survey/capture`` (what a real sensor driver would publish) and, for the
test only, ``/sim/capture_truth`` and ``/sim/pose_received`` (latched, at the first
pose, for the ready gate). ``SingleThreadedExecutor``.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_bridge.line import Line
from sensor_bridge.pose import PoseHistory
from sensor_bridge.px4_pose import message_type, pose_from_px4
from sensor_bridge_msgs.msg import SensorCapture
from sensor_bridge_msgs.srv import Stop
from std_msgs.msg import Empty
from survey_demo_msgs.msg import CaptureTruth

from survey_demo.common import (
    LATCHED,
    PX4_POSE,
    RELIABLE,
    declare,
    declare_optional,
    f32,
    u64,
)
from survey_demo.markers import GroundMarker
from survey_demo.sensor_sim import SensorParams, SensorSim

STATS_LOG_S = 10.0   # wall-clock period of the throughput log line
ADVANCE_S = 0.01     # wall-clock period of the sensor advance (one per callback batch)

PARAMS = {
    "seed": Parameter.Type.INTEGER,
    "clock_offset_ms": Parameter.Type.DOUBLE,
    "clock_jitter_ms": Parameter.Type.DOUBLE,
    "line_start_north_m": Parameter.Type.DOUBLE,
    "line_start_east_m": Parameter.Type.DOUBLE,
    "line_heading_deg": Parameter.Type.DOUBLE,
    "line_length_m": Parameter.Type.DOUBLE,
    "sensor_offset_m": Parameter.Type.DOUBLE_ARRAY,
    "capture_rate_hz": Parameter.Type.DOUBLE,
    "samples_per_capture": Parameter.Type.INTEGER,
    "sample_spacing_s": Parameter.Type.DOUBLE,
    "footprint_sigma_m": Parameter.Type.DOUBLE,
    "noise": Parameter.Type.DOUBLE,
    "max_pose_gap_s": Parameter.Type.DOUBLE,
    "pose_topic": Parameter.Type.STRING,
    "wait_for_ready": Parameter.Type.BOOL,
}
MARKER_PARAMS = {   # absent when the line has no markers
    "markers_s_m": Parameter.Type.DOUBLE_ARRAY,
    "markers_d_m": Parameter.Type.DOUBLE_ARRAY,
    "markers_amplitude": Parameter.Type.DOUBLE_ARRAY,
}


class ToySensorNode(Node):
    def __init__(self):
        super().__init__("toy_sensor_node")
        p = declare(self, PARAMS)
        m = declare_optional(self, MARKER_PARAMS)
        self.history = PoseHistory(max_gap_s=p["max_pose_gap_s"])
        self.line = Line(p["line_start_north_m"], p["line_start_east_m"], p["line_heading_deg"],
                         p["line_length_m"])
        self.markers = [GroundMarker(s, d, a) for s, d, a in zip(
            m["markers_s_m"], m["markers_d_m"], m["markers_amplitude"], strict=True)]
        self.params = SensorParams(p["capture_rate_hz"], p["samples_per_capture"],
                                   p["sample_spacing_s"], p["footprint_sigma_m"], p["noise"])
        seed = p["seed"]
        # Seeds: reading noise [seed, 0], clock jitter [seed, 1].
        self.rng_noise = np.random.default_rng([seed, 0])
        self.rng_jitter = np.random.default_rng([seed, 1])
        self.offset = tuple(float(v) for v in p["sensor_offset_m"])
        self.clock = (p["clock_offset_ms"], p["clock_jitter_ms"])
        self.sim: SensorSim | None = None
        self.ready = not p["wait_for_ready"]
        self.stopped = False
        self._captures = 0
        self._busy_s = 0.0
        self._advances = 0
        self._poses = 0
        self._advanced_to = None
        self._last_log = time.monotonic()

        self.pub_capture = self.create_publisher(SensorCapture, "/survey/capture", RELIABLE)
        self.pub_truth = self.create_publisher(CaptureTruth, "/sim/capture_truth", RELIABLE)
        self.pub_pose_received = self.create_publisher(Empty, "/sim/pose_received", LATCHED)
        self.create_subscription(message_type(), p["pose_topic"], self.on_pose, PX4_POSE)
        self.create_subscription(Empty, "/survey/ready", self.on_ready, LATCHED)
        self.create_service(Stop, "/survey/stop", self.on_stop)
        self.create_timer(ADVANCE_S, self.advance)
        self.get_logger().info(f"{len(self.markers)} markers on a {p['line_length_m']} m line")

    def on_ready(self, _msg) -> None:
        if not self.ready:
            self.get_logger().info("/survey/ready received")
        self.ready = True

    def on_pose(self, msg) -> None:
        """Cheap: append the pose (the sensor advances in ``advance``)."""
        pose = pose_from_px4(msg)
        first = self.history.newest is None
        if pose is None or not self.history.add(pose):
            return
        self._poses += 1
        if first:
            self.pub_pose_received.publish(Empty())
            self.get_logger().info(f"first pose at t = {pose.t_us} us")
        if self.ready and not self.stopped and self.sim is None:
            self.sim = SensorSim(self.params, self.markers, self.line, self.offset,
                                 self.clock[0], self.clock[1], self.rng_noise, self.rng_jitter,
                                 pose.t_us)
            self._advanced_to = pose.t_us
            self.get_logger().info(f"sensor started at t = {pose.t_us} us")

    def advance(self) -> None:
        """Emit every capture due up to the newest pose time (timer and ``/survey/stop``)."""
        if self.sim is None or self.stopped:
            return
        newest = self.history.newest.t_us
        if newest > self._advanced_to:
            self._advanced_to = newest
            busy = time.monotonic()
            before = self.sim.without_pose
            for capture, truth in self.sim.advance_to(newest, self.history):
                self.pub_capture.publish(self._capture_msg(capture))
                self.pub_truth.publish(self._truth_msg(truth))
                self._captures += 1
            if self.sim.without_pose > before and not before:
                self.get_logger().warning(
                    f"samples without bracketing poses (gap > {self.history.max_gap_us / 1e6} s)")
            self._busy_s += time.monotonic() - busy
            self._advances += 1
        if time.monotonic() - self._last_log >= STATS_LOG_S:
            self._last_log = time.monotonic()
            self.log_stats()

    def on_stop(self, request, response):
        self.advance()        # captures due by the poses already received
        last = self.sim.stop() if self.sim is not None else None
        self.stopped = True
        response.has_capture = last is not None
        response.last_capture_id = 0 if last is None else int(last)
        self.get_logger().info(f"stopped; last capture {last}")
        self.log_stats()
        return response

    @staticmethod
    def _capture_msg(capture) -> SensorCapture:
        msg = SensorCapture()
        msg.header.stamp.sec = int(capture.stamp_us // 1_000_000)
        msg.header.stamp.nanosec = int(capture.stamp_us % 1_000_000) * 1000
        msg.header.frame_id = "survey0"
        msg.sensor_id = "survey0"
        msg.capture_id = int(capture.capture_id)
        msg.sample_stamps_us = u64(capture.sample_stamps_us)
        msg.readings = f32(capture.readings)
        msg.saturated = bool(capture.saturated)
        return msg

    @staticmethod
    def _truth_msg(truth) -> CaptureTruth:
        msg = CaptureTruth()
        msg.capture_id = int(truth.capture_id)
        msg.sample_true_us = u64(truth.sample_true_us)
        msg.sample_stamp_us = u64(truth.sample_stamp_us)
        msg.sample_s = f32(truth.sample_s)
        msg.sample_d = f32(truth.sample_d)
        return msg

    def log_stats(self) -> None:
        without = self.sim.without_pose if self.sim is not None else 0
        self.get_logger().info(
            f"stats: captures={self._captures} samples_without_pose={without} "
            f"advances={self._advances} busy_s={self._busy_s:.2f} poses={self._poses} "
            f"poses_dropped={self.history.dropped}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ToySensorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.log_stats()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
