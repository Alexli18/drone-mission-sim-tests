"""The ground reference z0: the PX4 local z that heights are measured from (pure; no ROS).

PX4's local origin is not the ground (on SIH the landed vehicle reads
z ≈ +0.04 m), and MAVSDK's relative altitude (pass rule 5) is measured from PX4's
home position: relative_alt = alt − home.alt = −(z − home.z) while the EKF origin is
fixed. The simulated sensor and the georeferencing node therefore measure heights from the
same reference: h = −(z − z0) − o_z with z0 = ``home_position.z``.

Rule (deterministic, from the recorded messages alone):

- z0 is the ``z`` of the newest PX4 home position with ``valid_lpos``; a home position
  without it is ignored. PX4 publishes ``home_position`` (transient local) when home
  changes: at boot, at arming, at the start of the takeoff and after landing
  (disarmed); a node started after boot may miss the boot value. z0 therefore changes
  only at home, before the climb or after landing, where every capture is off the line
  (the line starts away from home).
- Before any home position, ``value`` is 0.0 (heights from the PX4 origin): only on
  the ground before arming.
- Replay: ``fix`` sets z0 from ``PoseSample.ground_z_m``, the z0 the live node used for
  the release that republished the pose.
"""

from __future__ import annotations


class GroundReference:
    def __init__(self):
        self.z0: float | None = None
        self.updates = 0           # how many times z0 was set or changed

    @property
    def is_set(self) -> bool:
        return self.z0 is not None

    @property
    def value(self) -> float:
        """z0, or 0.0 while it is not set."""
        return 0.0 if self.z0 is None else self.z0

    def update(self, z_m: float, valid: bool) -> bool:
        """One PX4 home position (``z``, ``valid_lpos``); True if z0 was set or changed."""
        if not valid:
            return False
        return self.fix(z_m)

    def fix(self, z0: float) -> bool:
        """Set z0 (replay: from a recording); True if it was set or changed."""
        z0 = float(z0)
        if self.z0 == z0:
            return False
        self.z0 = z0
        self.updates += 1
        return True
