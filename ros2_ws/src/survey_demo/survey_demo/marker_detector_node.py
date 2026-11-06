"""Marker detector node (the code under test; it would run unchanged on a real sensor).

Buffers the usable ``/survey/georef_capture`` messages and the ids received.
``/markers/finalize``:

1.  calls ``/survey/flush`` with ``call_async`` and waits for the future, getting
    F = ``last_capture_id``;
2.  waits until it has received the georeferenced capture with id F; steps 1 and 2 share
    one deadline, ``finalize_wait_s`` after the call arrived; on timeout it returns
    ``success`` false with the ids in ``message``;
3.  runs ``survey_demo.markers.detect_markers`` once on the usable captures, writes
    ``markers.json`` and returns the list, F and the count of usable captures.

It reads no ground truth and no simulation-only topic: its only input is
``/survey/georef_capture``.

``MultiThreadedExecutor``; the service server and the flush client share one
``ReentrantCallbackGroup``, and the node never blocks on ``call()``, only on
``call_async()`` futures.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_bridge_msgs.msg import GeorefCapture, Marker, MarkerList
from sensor_bridge_msgs.srv import Finalize, Flush

from survey_demo.common import RELIABLE, declare
from survey_demo.markers import detect_markers

POLL_S = 0.01

PARAMS = {
    "ds_m": Parameter.Type.DOUBLE,
    "threshold": Parameter.Type.DOUBLE,
    "min_captures": Parameter.Type.INTEGER,
    "max_gap_factor": Parameter.Type.DOUBLE,
    "markers_path": Parameter.Type.STRING,
    "finalize_wait_s": Parameter.Type.DOUBLE,
}


class MarkerDetectorNode(Node):
    def __init__(self):
        super().__init__("marker_detector_node")
        p = declare(self, PARAMS)
        self.markers_path = Path(p["markers_path"])
        self.finalize_wait_s = p["finalize_wait_s"]
        self.threshold = p["threshold"]
        self.min_captures = p["min_captures"]
        self.max_gap_m = p["max_gap_factor"] * p["ds_m"]
        self._lock = threading.Lock()
        self._usable: list = []           # (s, mean reading) of the usable captures
        self._ids: set = set()
        self._group = ReentrantCallbackGroup()
        self.create_subscription(GeorefCapture, "/survey/georef_capture", self.on_georef, RELIABLE,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.flush_client = self.create_client(Flush, "/survey/flush", callback_group=self._group)
        self.create_service(Finalize, "/markers/finalize", self.on_finalize,
                            callback_group=self._group)

    def on_georef(self, msg: GeorefCapture) -> None:
        with self._lock:
            self._ids.add(int(msg.raw.capture_id))
            if msg.usable:
                readings = list(msg.raw.readings)
                self._usable.append((float(msg.s), sum(readings) / len(readings)))

    def _wait(self, done, deadline: float) -> bool:
        while not done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(POLL_S)
        return True

    def on_finalize(self, request, response):
        deadline = time.monotonic() + self.finalize_wait_s
        response.markers = MarkerList(line_id=request.line_id)
        if not self._wait(self.flush_client.service_is_ready, deadline):
            response.success = False
            response.message = "/survey/flush not available"
            return response
        future = self.flush_client.call_async(Flush.Request())
        if not self._wait(future.done, deadline):
            response.success = False
            response.message = f"/survey/flush gave no reply within {self.finalize_wait_s} s"
            return response
        flushed = future.result()
        last = int(flushed.last_capture_id)
        if flushed.has_capture:
            def received():
                with self._lock:
                    return last in self._ids

            if not self._wait(received, deadline):
                with self._lock:
                    newest = max(self._ids) if self._ids else None
                response.success = False
                response.last_capture_id = last
                response.message = (f"georeferenced capture {last} not received within "
                                    f"{self.finalize_wait_s} s (newest received: {newest})")
                return response
        with self._lock:
            captures = list(self._usable)
        found = detect_markers(captures, self.threshold, self.min_captures, self.max_gap_m)
        result = {"line_id": request.line_id, "last_capture_id": last,
                  "captures_used": len(captures),
                  "markers": [{"s_m": m.s_m, "strength": m.strength,
                               "n_captures": m.n_captures} for m in found]}
        self.markers_path.parent.mkdir(parents=True, exist_ok=True)
        self.markers_path.write_text(json.dumps(result, indent=2) + "\n")
        response.markers.markers = [Marker(s_m=float(m.s_m), strength=float(m.strength),
                                           n_captures=int(m.n_captures)) for m in found]
        response.markers.header.frame_id = "line"
        response.success = True
        response.message = f"{len(found)} markers from {len(captures)} usable captures"
        response.last_capture_id = last
        response.captures_used = len(captures)
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
