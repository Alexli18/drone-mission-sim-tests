"""Ready gate: waits until the survey graph is complete, then publishes ``/survey/ready``.

Polls the graph every 0.2 s until every node in ``nodes`` and every service in
``services`` exists, each topic in ``topics_min_subscribers`` (``"<topic>:<n>"``) has
at least n subscriptions and a message has arrived on each topic in ``flag_topics``
(``std_msgs/Empty``, latched: ``/sim/pose_received`` and ``/survey/pose_received``, so
the toy sensor and the georeferencing node both have PX4 poses before the sensor
starts); then publishes ``/survey/ready`` (transient-local), prints ``SURVEY READY``
(flushed stdout, read by ``scripts/start_ros2.sh`` from ``docker logs``) and, if
``exit_when_ready``, exits. ``flag_topics`` is optional (absent in replay): a ROS 2
Humble parameter file cannot hold an empty list. The launch files start it with
``output="screen"``.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty

from survey_demo.common import LATCHED, declare
from survey_demo.gate import missing_items, parse_topic_minimum

READY_LINE = "SURVEY READY"
POLL_S = 0.2
MISSING_LOG_S = 5.0   # wall-clock period of the "still waiting for" log line

PARAMS = {
    "nodes": Parameter.Type.STRING_ARRAY,
    "services": Parameter.Type.STRING_ARRAY,
    "topics_min_subscribers": Parameter.Type.STRING_ARRAY,
    "exit_when_ready": Parameter.Type.BOOL,
}
OPTIONAL_PARAMS = {"flag_topics": Parameter.Type.STRING_ARRAY}


def declare_optional(node: Node, types: dict) -> dict:
    """Declare typed parameters that may be absent; an absent one is an empty list."""
    out = {}
    for name, kind in types.items():
        node.declare_parameter(name, kind)
        try:
            out[name] = list(node.get_parameter(name).value)
        except ParameterUninitializedException:
            out[name] = []
    return out


class ReadyGate(Node):
    def __init__(self):
        super().__init__("ready_gate")
        p = declare(self, PARAMS)
        p.update(declare_optional(self, OPTIONAL_PARAMS))
        self.want_nodes = list(p["nodes"])
        self.want_services = list(p["services"])
        self.want_topics = [parse_topic_minimum(item) for item in p["topics_min_subscribers"]]
        self.want_flags = list(p["flag_topics"])
        self.flags_received: set[str] = set()
        self.exit_when_ready = p["exit_when_ready"]
        self.done = False
        self.started = time.monotonic()
        self._last_missing_log = self.started
        self.pub = self.create_publisher(Empty, "/survey/ready", LATCHED)
        for topic in self.want_flags:
            self.create_subscription(Empty, topic,
                                     lambda _msg, topic=topic: self.on_flag(topic), LATCHED)
        self.create_timer(POLL_S, self.poll)

    def on_flag(self, topic: str) -> None:
        if topic not in self.flags_received:
            self.get_logger().info(f"{topic} received")
        self.flags_received.add(topic)

    def missing(self) -> list[str]:
        return missing_items(set(self.get_node_names()),
                             {name for name, _ in self.get_service_names_and_types()},
                             self.count_subscribers, self.want_nodes, self.want_services,
                             self.want_topics, self.want_flags, self.flags_received)

    def poll(self) -> None:
        if self.done:
            return
        missing = self.missing()
        if missing:
            if time.monotonic() - self._last_missing_log >= MISSING_LOG_S:
                self._last_missing_log = time.monotonic()
                self.get_logger().info("waiting for: " + ", ".join(missing))
            return
        self.pub.publish(Empty())
        print(f"{READY_LINE} after {time.monotonic() - self.started:.1f} s", flush=True)
        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReadyGate()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not (node.done and node.exit_when_ready):
            executor.spin_once(timeout_sec=POLL_S)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
