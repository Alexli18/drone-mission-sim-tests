"""ROS 2 wrapper of ``sensor_bridge.georeference`` (same node on the real drone).

Subscribes to the
pose topic (``pose_type`` ``px4``: ``px4_msgs/VehicleLocalPosition`` through the
``px4_pose`` adapter; ``sample``: ``sensor_bridge_msgs/PoseSample``, replay) and to
``/survey/capture``; publishes ``/survey/georef_capture``, ``/survey/status``,
``/survey/pose_received`` (``std_msgs/Empty``, latched, once at the first pose; read by
the ready gate) and, with ``republish_poses``, ``/survey/pose_used``
(each pose with the z0 of its release); serves ``/survey/flush``. z0
(``sensor_bridge.ground``): live (``px4``), from ``home_topic``
(``px4_msgs/HomePosition``); replay (``sample``), from ``PoseSample.ground_z_m``.
``SingleThreadedExecutor``. Simulated time is the newest pose time; wall-clock is used
only for the 0.1 s release timer, the wall fallback and the post-release status.
"""

from __future__ import annotations

import array
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_bridge_msgs.msg import GeorefCapture, PoseSample, SensorCapture, SurveyStatus
from sensor_bridge_msgs.srv import Flush
from std_msgs.msg import Empty

from sensor_bridge.georeference import (
    DecisionLog,
    GeorefParams,
    HoldQueue,
    StatusCounters,
    replay_params,
)
from sensor_bridge.ground import GroundReference
from sensor_bridge.line import Line
from sensor_bridge.pose import PoseHistory, pose_from_sample

TIMER_S = 0.1           # wall-clock release timer
STATS_LOG_S = 10.0      # wall-clock period of the throughput log line (QoS and load diagnosis)
POSE_INTERVAL_US = 20_000  # PX4 publishes 50 Hz (simulated); longer intervals are counted as gaps

RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=100)
STATUS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
# PX4 subscriptions keep a deep queue: at a speed factor of about 6 the 50 Hz poses
# arrive at about 300 Hz wall, and a single-threaded node busy with a capture for longer
# than depth/300 s loses the oldest ones (depth 5, about 16 ms of wall time, lost a
# third of the poses at the busy simulated sensor in a CI flake). 500 is about 1.7 s of wall
# time; a pose is about 200 bytes.
PX4_DEPTH = 500
PX4_POSE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=PX4_DEPTH)
# PX4 publishes home_position best-effort and transient-local (published on change), so
# a late subscriber still gets the current home.
PX4_HOME = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=PX4_DEPTH)

PARAMS = {
    "line_start_north_m": Parameter.Type.DOUBLE,
    "line_start_east_m": Parameter.Type.DOUBLE,
    "line_heading_deg": Parameter.Type.DOUBLE,
    "line_length_m": Parameter.Type.DOUBLE,
    "sensor_height_m": Parameter.Type.DOUBLE,
    "sensor_offset_m": Parameter.Type.DOUBLE_ARRAY,
    "off_line_m": Parameter.Type.DOUBLE,
    "height_band_m": Parameter.Type.DOUBLE,
    "hold_timeout_s": Parameter.Type.DOUBLE,
    "wall_fallback_s": Parameter.Type.DOUBLE,
    "max_pose_gap_s": Parameter.Type.DOUBLE,
    "coverage_gap_factor": Parameter.Type.DOUBLE,
    "ds_m": Parameter.Type.DOUBLE,
    "status_period_s": Parameter.Type.DOUBLE,
    "decisions_path": Parameter.Type.STRING,
    "pose_topic": Parameter.Type.STRING,
    "pose_type": Parameter.Type.STRING,
    "home_topic": Parameter.Type.STRING,
    "republish_poses": Parameter.Type.BOOL,
}


def declare(node: Node, types: dict) -> dict:
    """Declare typed parameters without defaults; a missing one raises at once."""
    for name, kind in types.items():
        node.declare_parameter(name, kind)
    return {name: node.get_parameter(name).value for name in types}


def f32(values) -> array.array:
    return array.array("f", np.asarray(values, dtype=np.float32).tobytes())


class GeoreferenceNode(Node):
    def __init__(self):
        super().__init__("georeference_node")
        p = declare(self, PARAMS)
        self.line = Line(p["line_start_north_m"], p["line_start_east_m"],
                         p["line_heading_deg"], p["line_length_m"])
        self.cfg = GeorefParams(
            sensor_height_m=p["sensor_height_m"],
            sensor_offset_m=tuple(float(v) for v in p["sensor_offset_m"]),
            off_line_m=p["off_line_m"], height_band_m=p["height_band_m"],
            hold_timeout_s=p["hold_timeout_s"], wall_fallback_s=p["wall_fallback_s"])
        if p["pose_type"] == "sample":
            # Replay: no simulated-time hold timeout (see replay_params).
            self.cfg = replay_params(self.cfg)
        self.history = PoseHistory(max_gap_s=p["max_pose_gap_s"])
        self.ground = GroundReference()
        self.queue = HoldQueue(self.line, self.cfg)
        self.counters = StatusCounters(p["line_length_m"], p["ds_m"], p["coverage_gap_factor"])
        self.decisions = DecisionLog(p["decisions_path"])
        self.status_period_us = p["status_period_s"] * 1e6
        self.republish = p["republish_poses"]
        self.replay = p["pose_type"] == "sample"
        self._last_status_us = None
        self._dirty = False
        # Throughput figures (logged, not published).
        self._poses = 0
        self._pose_gaps = 0
        self._max_pose_interval_us = 0
        self._last_log = time.monotonic()

        # Every publisher in the constructor, so the bag recorder finds them before the
        # first capture.
        self.pub_georef = self.create_publisher(GeorefCapture, "/survey/georef_capture", RELIABLE)
        self.pub_status = self.create_publisher(SurveyStatus, "/survey/status", STATUS)
        self.pub_pose = (self.create_publisher(PoseSample, "/survey/pose_used", RELIABLE)
                         if self.republish else None)
        self.pub_pose_received = self.create_publisher(Empty, "/survey/pose_received", LATCHED)

        if p["pose_type"] == "px4":
            from sensor_bridge.px4_pose import (
                home_message_type,
                home_z_from_px4,
                message_type,
                pose_from_px4,
            )

            self._to_pose = pose_from_px4
            self._home_z = home_z_from_px4
            self.create_subscription(message_type(), p["pose_topic"], self.on_pose, PX4_POSE)
            self.create_subscription(home_message_type(), p["home_topic"], self.on_home,
                                     PX4_HOME)
        elif p["pose_type"] == "sample":
            self._to_pose = pose_from_sample
            self.create_subscription(PoseSample, p["pose_topic"], self.on_pose, RELIABLE)
        else:
            raise ValueError(f"pose_type: expected px4 or sample, got {p['pose_type']!r}")
        self.create_subscription(SensorCapture, "/survey/capture", self.on_raw, RELIABLE)
        self.create_service(Flush, "/survey/flush", self.on_flush)
        self.create_timer(TIMER_S, self.on_timer)

    # Inputs -------------------------------------------------------------------------

    def on_pose(self, msg) -> None:
        pose = self._to_pose(msg)
        if pose is None:
            return
        previous = self.history.newest
        if not self.history.add(pose):
            return
        if self.replay:
            # The z0 the live node used for the release that republished this pose.
            self._log_ground(self.ground.fix(msg.ground_z_m))
        if previous is not None:
            interval = pose.t_us - previous.t_us
            self._max_pose_interval_us = max(self._max_pose_interval_us, interval)
            self._pose_gaps += int(interval > 1.5 * POSE_INTERVAL_US)
        self._poses += 1
        if not self.counters.pose_received:
            self.pub_pose_received.publish(Empty())
            self.get_logger().info(f"first pose at t = {pose.t_us} us")
        self.counters.on_pose(pose.t_us)
        self.queue.pose_arrived(time.monotonic())
        self._release()
        now = self.counters.now_us
        if self._last_status_us is None or now - self._last_status_us >= self.status_period_us:
            self._last_status_us = now
            self.publish_status()

    def on_home(self, msg) -> None:
        self._log_ground(self.ground.update(*self._home_z(msg)))

    def _log_ground(self, changed: bool) -> None:
        if changed:
            self.get_logger().info(f"ground reference z0 = {self.ground.z0:.4f} m (update "
                                   f"{self.ground.updates}, now {self.counters.now_us} us)")

    def on_raw(self, msg: SensorCapture) -> None:
        self.counters.on_raw(msg.capture_id)
        now_us = self.counters.now_us if self.counters.pose_received else None
        self.queue.push(msg, now_us, time.monotonic())
        self._release()

    def on_timer(self) -> None:
        self._release()
        if self._dirty:
            self.publish_status()
        if time.monotonic() - self._last_log >= STATS_LOG_S:
            self._last_log = time.monotonic()
            self.log_stats()

    def on_flush(self, request, response):
        released = self.queue.flush(self.history, self.ground.value)
        self._publish(released)
        self.publish_status()
        response.has_capture = self.counters.has_capture
        response.last_capture_id = self.counters.last_capture_id
        response.released_count = len(released)
        self.log_stats()
        return response

    # Outputs ------------------------------------------------------------------------

    def _release(self) -> None:
        now_us = self.counters.now_us if self.counters.pose_received else None
        self._publish(self.queue.release(self.history, now_us, time.monotonic(),
                                         self.ground.value))

    def _publish(self, released: list) -> None:
        for item in released:
            if self.republish:
                for pose in item.new_poses:
                    self.pub_pose.publish(PoseSample(
                        t_us=int(pose.t_us), x_m=float(pose.x), y_m=float(pose.y),
                        z_m=float(pose.z), heading_rad=float(pose.heading_rad),
                        ground_z_m=float(item.ground_z)))
            g = item.georef
            msg = GeorefCapture()
            msg.raw = g.raw
            msg.sample_s = f32(g.sample_s)
            msg.sample_d = f32(g.sample_d)
            msg.sample_h = f32(g.sample_h)
            msg.sample_has_pose = [bool(v) for v in g.sample_has_pose]
            msg.s, msg.d, msg.h = float(g.s), float(g.d), float(g.h)
            msg.usable = bool(g.usable)
            msg.reason = g.reason
            self.pub_georef.publish(msg)
            self.decisions.append(item)
            self.counters.on_release(item, republished=self.republish)
        if released:
            self._dirty = True

    def publish_status(self) -> None:
        snap = self.counters.snapshot(queue_empty=len(self.queue) == 0)
        msg = SurveyStatus()
        msg.now_us = int(snap["now_us"])
        msg.pose_received = bool(snap["pose_received"])
        msg.last_pose_stamp_us = int(snap["last_pose_stamp_us"])
        msg.captures_seen = int(snap["captures_seen"])
        msg.has_capture = bool(snap["has_capture"])
        msg.last_capture_id = int(snap["last_capture_id"])
        msg.released = int(snap["released"])
        msg.usable = int(snap["usable"])
        msg.reasons = list(snap["reasons"])
        msg.counts = [int(c) for c in snap["counts"]]
        msg.poses_used = int(snap["poses_used"])
        msg.coverage = float(snap["coverage"])
        msg.max_gap_m = float(snap["max_gap_m"])
        msg.queue_empty = bool(snap["queue_empty"])
        self.pub_status.publish(msg)
        self._dirty = False

    def log_stats(self) -> None:
        c = self.counters
        counts = ", ".join(f"{r}={n}" for r, n in c.counts.items() if n)
        self.get_logger().info(
            f"stats: poses={self._poses} dropped={self.history.dropped} "
            f"ground_z={self.ground.z0} "
            f"pose_gaps={self._pose_gaps} max_pose_interval_us={self._max_pose_interval_us} "
            f"captures_seen={c.captures_seen} released={c.released} held={len(self.queue)} "
            f"[{counts}]")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeoreferenceNode()
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
