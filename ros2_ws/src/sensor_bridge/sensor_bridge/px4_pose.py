"""Adapters from ``px4_msgs/VehicleLocalPosition`` to ``Pose`` and from
``px4_msgs/HomePosition`` to the ground reference.

The only ``sensor_bridge`` module
that imports ``px4_msgs``, and only inside ``message_type()`` and ``home_message_type()``,
so the conversions themselves
works on any object with the message's fields.
"""

from __future__ import annotations

from sensor_bridge.pose import Pose


def message_type():
    """The ``VehicleLocalPosition`` class (imports ``px4_msgs``)."""
    from px4_msgs.msg import VehicleLocalPosition

    return VehicleLocalPosition


def pose_from_px4(msg) -> Pose | None:
    """``Pose`` from ``timestamp_sample``, x, y, z and heading; None without xy and z valid."""
    if not (msg.xy_valid and msg.z_valid):
        return None
    return Pose(t_us=int(msg.timestamp_sample), x=float(msg.x), y=float(msg.y), z=float(msg.z),
                heading_rad=float(msg.heading))




def home_message_type():
    """The ``HomePosition`` class (imports ``px4_msgs``)."""
    from px4_msgs.msg import HomePosition

    return HomePosition


def home_z_from_px4(msg) -> tuple[float, bool]:
    """(``z``, ``valid_lpos``) of a home position, for ``GroundReference.update``."""
    return float(msg.z), bool(msg.valid_lpos)
