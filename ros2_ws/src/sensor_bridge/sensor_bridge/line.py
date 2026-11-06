"""The survey line and the sensor position (pure; no ROS).

Line frame: s along the line from its start, d cross-track (positive to the right of
the heading), h height above the ground reference z0 (``sensor_bridge.ground``). The
sensor sits at a body-frame offset from the vehicle origin (a lever arm).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Line:
    """A straight line in metres from home; ``heading_deg`` clockwise from north."""

    start_north_m: float
    start_east_m: float
    heading_deg: float
    length_m: float

    def project(self, north: float, east: float) -> tuple[float, float]:
        """(s, d) of a (north, east) offset from home."""
        h = math.radians(self.heading_deg)
        dn, de = north - self.start_north_m, east - self.start_east_m
        return dn * math.cos(h) + de * math.sin(h), -dn * math.sin(h) + de * math.cos(h)


def offset_xyz(offset) -> tuple[float, float, float]:
    """(x, y, z) of an offset given as a sequence or an object with x, y, z."""
    if hasattr(offset, "x"):
        return float(offset.x), float(offset.y), float(offset.z)
    x, y, z = offset
    return float(x), float(y), float(z)


def sensor_position(pose, offset, ground_z: float = 0.0) -> tuple[float, float, float]:
    """(north, east, h) of the sensor: the body-frame offset rotated by heading.

    north = x + cos ψ·o_x − sin ψ·o_y, east = y + sin ψ·o_x + cos ψ·o_y,
    h = −(z − z0) − o_z with z0 = ``ground_z`` (``sensor_bridge.ground``).
    """
    ox, oy, oz = offset_xyz(offset)
    c, s = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
    return pose.x + c * ox - s * oy, pose.y + s * ox + c * oy, -(pose.z - ground_z) - oz
