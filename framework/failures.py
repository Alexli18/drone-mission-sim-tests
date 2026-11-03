"""Fault mechanisms, one per failsafe scenario kind (chosen by spikes S5-S9).

Each fault declares the PX4 parameters it needs (the loader checks them after
the baseline merge; faults never set parameters) and hooks the test calls:
``prepare`` before arming, ``trigger`` at the trigger point, ``cleanup`` always.
A fault with ``flow = "offboard"`` leaves the fence by offboard velocity
setpoints instead of flying a mission (PX4 rejects missions and goto targets
outside the fence, spike S9 and the geofence follow-up).
"""

from __future__ import annotations

import asyncio
import contextlib
import platform

import pytest
from mavsdk_grpc.failure import FailureType, FailureUnit
from mavsdk_grpc.geofence import FenceType, GeofenceData, Point, Polygon
from mavsdk_grpc.offboard import VelocityNedYaw

from framework.assertions import call
from framework.mission import offset_to_latlon

# Test -> PX4 traffic on PX4's API link (S6: this cuts the GCS heartbeats PX4 sees
# while telemetry from PX4 keeps flowing).
IPTABLES_RULE = ["OUTPUT", "-o", "lo", "-p", "udp", "--dport", "14580", "-j", "DROP"]


class Fault:
    kind = ""
    flow = "mission"
    required_params: frozenset = frozenset()

    def __init__(self, drone, scenario, recorder, cfg, home):
        self.drone = drone
        self.scenario = scenario
        self.recorder = recorder
        self.cfg = cfg
        self.home = home

    async def prepare(self) -> None:
        pass

    async def trigger(self) -> None:
        pass

    async def cleanup(self) -> None:
        pass


class GpsLoss(Fault):
    kind = "gps_loss"
    required_params = frozenset({"SYS_FAILURE_EN", "EKF2_NOAID_TOUT", "COM_POS_FS_EPH"})

    async def trigger(self) -> None:
        await call(self.drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, 0),
                   self.cfg, "inject GPS failure")


class LinkLoss(Fault):
    """S5: built-in mavlink_signal injection is unsupported; S6: drop test -> PX4 traffic."""

    kind = "link_loss"
    required_params = frozenset({"NAV_DLL_ACT", "COM_DLL_EXCEPT", "COM_FAIL_ACT_T",
                                 "COM_DL_LOSS_T"})
    _rule_added = False

    async def prepare(self) -> None:
        if platform.system() != "Linux":
            pytest.skip("link_loss uses iptables, which needs Linux (spike S6)")

    async def trigger(self) -> None:
        await _iptables("-I")
        self._rule_added = True

    async def cleanup(self) -> None:
        if self._rule_added:
            await _iptables("-D")
            self._rule_added = False


async def _iptables(action: str) -> None:
    proc = await asyncio.create_subprocess_exec(
        "sudo", "iptables", action, *IPTABLES_RULE,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    _, err = await proc.communicate()
    if proc.returncode != 0:
        raise RuntimeError(f"iptables {action} failed: {err.decode().strip()}")


class RcLoss(Fault):
    """S5: built-in rc_signal injection is unsupported; S7: stop the stick stream."""

    kind = "rc_loss"
    required_params = frozenset({"COM_RC_IN_MODE", "NAV_RCL_ACT", "COM_RCL_EXCEPT",
                                 "COM_FAIL_ACT_T", "COM_RC_LOSS_T"})
    _stream: asyncio.Task | None = None

    async def prepare(self) -> None:
        # PX4 ignores RC loss if there was no RC at arming, so stream from before arming.
        async def stream():
            while True:
                await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
                await asyncio.sleep(0.02)  # 50 Hz wall-clock; neutral sticks
        self._stream = asyncio.create_task(stream())

    async def trigger(self) -> None:
        await self.cleanup()

    async def cleanup(self) -> None:
        if self._stream is not None:
            self._stream.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._stream
            self._stream = None


class LowBattery(Fault):
    """No trigger: the simulated battery drains from arming."""

    kind = "low_battery"
    required_params = frozenset({"SIM_BAT_DRAIN", "SIM_BAT_MIN_PCT", "BAT_LOW_THR",
                                 "BAT_CRIT_THR", "BAT_EMERGEN_THR", "COM_LOW_BAT_ACT"})


class GeofenceBreach(Fault):
    kind = "geofence_breach"
    flow = "offboard"
    required_params = frozenset({"GF_ACTION"})
    _pusher: asyncio.Task | None = None

    async def prepare(self) -> None:
        points = [Point(*offset_to_latlon(self.home.latitude_deg, self.home.longitude_deg, n, e))
                  for n, e in self.scenario.geofence]
        fence = GeofenceData([Polygon(points, FenceType.INCLUSION)], [])
        await call(self.drone.geofence.upload_geofence(fence), self.cfg, "upload geofence")

    async def trigger(self) -> None:
        """Fly out of the fence with offboard velocity until PX4 takes over."""
        velocity = VelocityNedYaw(self.scenario.failure.breach_north_m_s, 0.0, 0.0, 0.0)
        offboard = self.drone.offboard
        await call(offboard.set_velocity_ned(velocity), self.cfg, "initial offboard setpoint")
        await call(offboard.start(), self.cfg, "start offboard")

        async def push():  # offboard needs a continuous setpoint stream
            while True:
                mode = self.recorder.latest("flight_mode")
                if mode is not None and mode.name != "OFFBOARD":
                    return  # the fence action took over (as in the geofence spike)
                await offboard.set_velocity_ned(velocity)
                await asyncio.sleep(0.05)
        self._pusher = asyncio.create_task(push())

    async def cleanup(self) -> None:
        if self._pusher is not None:
            self._pusher.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._pusher
            self._pusher = None


FAULTS = {cls.kind: cls for cls in (GpsLoss, LinkLoss, RcLoss, LowBattery, GeofenceBreach)}
