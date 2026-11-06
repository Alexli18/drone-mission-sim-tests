"""QoS profiles, typed parameters and array conversions shared by the survey nodes.

The QoS profiles and ``declare`` are those of ``sensor_bridge.georeference_node``, so
the nodes here and the node under test cannot drift apart.
"""

from __future__ import annotations

import array

import numpy as np
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from sensor_bridge.georeference_node import (
    LATCHED,
    PX4_HOME,
    PX4_POSE,
    RELIABLE,
    STATUS,
    declare,
    f32,
)

__all__ = ["LATCHED", "PX4_HOME", "PX4_POSE", "RELIABLE", "STATUS", "declare",
           "declare_optional", "f32", "u64"]


def declare_optional(node: Node, types: dict) -> dict:
    """Declare typed list parameters that may be absent; an absent one is an empty list.

    A ROS 2 Humble parameter file cannot hold an empty list, so a scenario without
    markers leaves the keys out.
    """
    out = {}
    for name, kind in types.items():
        node.declare_parameter(name, kind)
        try:
            out[name] = list(node.get_parameter(name).value)
        except ParameterUninitializedException:
            out[name] = []
    return out


def u64(values) -> array.array:
    return array.array("Q", np.asarray(values, dtype=np.uint64).tobytes())
