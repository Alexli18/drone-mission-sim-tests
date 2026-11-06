"""Bag player for the replay check.

Plays the live run's ``/survey/capture`` and ``/survey/pose_used`` at the recorded pace
(rate 1), but only after each of its publishers has matched at least ``min_subscribers``
subscriptions, plus ``settle_s`` for the subscription side of the match.

It replaces ``ros2 bag play --delay 2.0``: with reliable, volatile QoS a message
published before the subscription has matched is lost, and a fixed delay is a race.
In a CI run the replay's georeferencing node got nothing from the first 0.77 s of the
bag (captures 0-55 and the 89 poses before them, on both topics alike), so the replay
check failed on captures the replay never saw.

Exits 0 after the last message; exits 1 if a publisher does not match within
``match_timeout_s`` (nothing is played then).
"""

from __future__ import annotations

import sys
import time

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from survey_demo.common import RELIABLE

POLL_S = 0.05


def open_reader(uri: str, topics: list[str]) -> tuple[rosbag2_py.SequentialReader, dict]:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        raise RuntimeError(f"topics not in the bag {uri}: {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader, {t: types[t] for t in topics}


def main(args=None) -> int:
    rclpy.init(args=args)
    node = Node("bag_player")
    try:
        bag = node.declare_parameter("bag", "/in/bag").value
        topics = list(node.declare_parameter("topics", ["/survey/capture"]).value)
        min_subs = int(node.declare_parameter("min_subscribers", 1).value)
        match_timeout_s = float(node.declare_parameter("match_timeout_s", 30.0).value)
        settle_s = float(node.declare_parameter("settle_s", 1.0).value)
        log = node.get_logger()

        reader, types = open_reader(bag, topics)
        classes = {t: get_message(types[t]) for t in topics}
        pubs = {t: node.create_publisher(classes[t], t, RELIABLE) for t in topics}

        start = time.monotonic()
        while True:
            counts = {t: p.get_subscription_count() for t, p in pubs.items()}
            if all(n >= min_subs for n in counts.values()):
                break
            if time.monotonic() - start > match_timeout_s:
                log.error(f"publishers not matched after {match_timeout_s} s: {counts}")
                return 1
            time.sleep(POLL_S)
        log.info(f"matched after {time.monotonic() - start:.2f} s: {counts}; "
                 f"settling {settle_s} s")
        time.sleep(settle_s)

        sent = dict.fromkeys(topics, 0)
        t0_bag = None
        t0_wall = time.monotonic()
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            if t0_bag is None:
                t0_bag = stamp_ns
            wait = (stamp_ns - t0_bag) / 1e9 - (time.monotonic() - t0_wall)
            if wait > 0:
                time.sleep(wait)
            pubs[topic].publish(deserialize_message(data, classes[topic]))
            sent[topic] += 1
        log.info(f"played {sent} in {time.monotonic() - t0_wall:.1f} s")
        # Leave time for the reliable writers to deliver the last messages.
        time.sleep(settle_s)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
