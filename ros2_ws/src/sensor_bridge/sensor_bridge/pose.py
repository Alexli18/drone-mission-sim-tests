"""Vehicle poses and their history, with linear interpolation (pure; no ROS).

Times are PX4 simulated time in
µs (``timestamp_sample``); x, y, z are NED from home (vehicle origin).
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    t_us: int
    x: float                # NED north, m
    y: float                # NED east, m
    z: float                # NED down, m
    heading_rad: float


def _wrap(angle: float) -> float:
    """Angle in [−π, π)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class PoseHistory:
    """Poses in time order, kept for ``max_age_s`` behind the newest.

    A pose not newer than the newest is dropped and counted in ``dropped``.
    """

    def __init__(self, max_gap_s: float, max_age_s: float = 10.0):
        self.max_gap_us = max_gap_s * 1e6
        self.max_age_us = max_age_s * 1e6
        self._poses: list[Pose] = []
        self._times: list[int] = []
        self.dropped = 0

    def __len__(self) -> int:
        return len(self._poses)

    @property
    def newest(self) -> Pose | None:
        return self._poses[-1] if self._poses else None

    def add(self, pose: Pose) -> bool:
        """Append ``pose``; returns False if it was dropped (not newer than the newest)."""
        if self._times and pose.t_us <= self._times[-1]:
            self.dropped += 1
            return False
        self._poses.append(pose)
        self._times.append(pose.t_us)
        cut = bisect.bisect_left(self._times, pose.t_us - self.max_age_us)
        if cut:
            del self._poses[:cut]
            del self._times[:cut]
        return True

    def covers(self, t_us: int) -> bool:
        """The newest pose time is at or after ``t_us``."""
        return bool(self._times) and self._times[-1] >= t_us

    def bracketing(self, t_us: int):
        """The two adjacent poses around ``t_us`` (one pose twice at an exact match).

        None if ``t_us`` is outside the history or the two are more than
        ``max_gap_s`` apart.
        """
        if not self._times or t_us < self._times[0] or t_us > self._times[-1]:
            return None
        j = bisect.bisect_left(self._times, t_us)
        if self._times[j] == t_us:
            return self._poses[j], self._poses[j]
        p0, p1 = self._poses[j - 1], self._poses[j]
        if p1.t_us - p0.t_us > self.max_gap_us:
            return None
        return p0, p1

    def window(self, t0_us: int, t1_us: int) -> list[Pose]:
        """The poses that can bracket a time in [``t0_us``, ``t1_us``]: from the last at
        or before ``t0_us`` (else the first) to the first at or after ``t1_us`` (else the
        newest), in time order."""
        if not self._times:
            return []
        lo = max(bisect.bisect_right(self._times, t0_us) - 1, 0)
        hi = bisect.bisect_left(self._times, t1_us) + 1
        return self._poses[lo:hi]

    def interpolate(self, t_us: int) -> Pose | None:
        """Linear interpolation at ``t_us`` (heading along the shorter arc), or None."""
        pair = self.bracketing(t_us)
        if pair is None:
            return None
        return interpolate_pair(pair[0], pair[1], t_us)


def interpolate_pair(p0: Pose, p1: Pose, t_us: int) -> Pose:
    """The pose at ``t_us`` between ``p0`` and ``p1``."""
    if p1.t_us == p0.t_us:
        return Pose(t_us, p0.x, p0.y, p0.z, p0.heading_rad)
    w = (t_us - p0.t_us) / (p1.t_us - p0.t_us)
    heading = _wrap(p0.heading_rad + w * _wrap(p1.heading_rad - p0.heading_rad))
    return Pose(t_us, p0.x + w * (p1.x - p0.x), p0.y + w * (p1.y - p0.y),
                p0.z + w * (p1.z - p0.z), heading)


def pose_from_sample(msg) -> Pose:
    """``Pose`` from a ``sensor_bridge_msgs/PoseSample`` (or any object with its fields)."""
    return Pose(t_us=int(msg.t_us), x=float(msg.x_m), y=float(msg.y_m), z=float(msg.z_m),
                heading_rad=float(msg.heading_rad))
