"""M4: apply a fault during flight and check PX4's failsafe reaction.

Parametrized over every scenario of type ``failsafe``. The fault mechanism per
kind is in framework/failures.py; the pass rule in framework/assertions.py.
"""

import time

import pytest

from framework.assertions import (
    Moment,
    Reference,
    assert_failsafe_outcome,
    call,
    wait_for_stream,
    wait_until,
)
from framework.failures import FAULTS
from framework.mission import lawnmower_waypoints, list_scenarios, to_mission_plan
from framework.sitl import wait_armable

pytestmark = pytest.mark.sitl


def now(recorder) -> Moment:
    return Moment(recorder.sim_time_s(), time.monotonic())


def mode_is(recorder, name):
    mode = recorder.latest("flight_mode")
    return mode is not None and mode.name == name


async def start_mission(drone, scenario, recorder, cfg, home) -> Moment:
    waypoints = lawnmower_waypoints(scenario.area)
    plan, _ = to_mission_plan(waypoints, home.latitude_deg, home.longitude_deg,
                              scenario.altitude_m, scenario.speed_m_s,
                              cfg.tolerances.waypoint_m / 2)
    # No Return at the end: a failsafe that never fires must not look like one.
    await call(drone.mission.set_return_to_launch_after_mission(False), cfg, "no RTL after mission")
    await call(drone.mission.upload_mission(plan), cfg, "upload mission")
    await wait_armable(drone, cfg)
    await call(drone.action.arm(), cfg, "arm")
    await call(drone.mission.start_mission(), cfg, "start mission")
    await wait_until(lambda: mode_is(recorder, "MISSION"), cfg.timeouts_wall_s.mission_start,
                     "flight mode MISSION", recorder, cfg.poll_interval_s)
    return now(recorder)


async def wait_for_trigger(trigger, start: Moment, recorder, cfg) -> None:
    if trigger is None:
        return
    if trigger.type == "at_waypoint":
        def reached():
            progress = recorder.latest("mission_progress")
            return progress is not None and progress.current >= trigger.value
    else:  # after_s: simulated seconds since the active phase started
        def reached():
            return recorder.sim_time_s() >= start.sim_s + trigger.value
    await wait_until(reached, cfg.timeouts_wall_s.mission, f"trigger {trigger}", recorder,
                     cfg.poll_interval_s)


async def take_off_and_hold(drone, recorder, cfg) -> Moment:
    await wait_armable(drone, cfg)
    await call(drone.action.arm(), cfg, "arm")
    takeoff_wall = time.monotonic()
    await call(drone.action.takeoff(), cfg, "takeoff")

    def holding_after_takeoff():
        modes = [m for _, wall, m in recorder.mode_history() if wall >= takeoff_wall]
        return "TAKEOFF" in modes and modes[-1] == "HOLD"

    await wait_until(holding_after_takeoff, cfg.timeouts_wall_s.takeoff,
                     "takeoff finished (TAKEOFF then HOLD)", recorder, cfg.poll_interval_s)
    return now(recorder)


@pytest.mark.parametrize("scenario", list_scenarios("failsafe"), indirect=True,
                         ids=lambda path: path.stem)
async def test_failsafe(cfg, scenario, drone, recorder, record_property):
    home = await wait_for_stream(drone.telemetry.home(), lambda h: True,
                                 cfg.timeouts_wall_s.health, "home position")
    fault = FAULTS[scenario.failure.kind](drone, scenario, recorder, cfg, home)
    try:
        await fault.prepare()
        if fault.flow == "mission":
            active_mode = "MISSION"
            start = await start_mission(drone, scenario, recorder, cfg, home)
            await wait_for_trigger(scenario.failure.trigger, start, recorder, cfg)
        else:  # offboard: the fault itself moves the vehicle (geofence breach)
            active_mode = "OFFBOARD"
            await take_off_and_hold(drone, recorder, cfg)
            start = now(recorder)
        trigger = now(recorder)
        await fault.trigger()
        if fault.flow == "offboard":
            await wait_until(lambda: mode_is(recorder, "OFFBOARD"),
                             cfg.timeouts_wall_s.mission_start, "flight mode OFFBOARD",
                             recorder, cfg.poll_interval_s)
        reference = Reference(start=start, trigger=trigger)
        distance = await assert_failsafe_outcome(recorder, scenario.expected, reference, cfg,
                                                 home, active_mode)
    finally:
        await fault.cleanup()

    after = [(round(t - trigger.sim_s, 1), m) for t, wall, m in recorder.mode_history()
             if wall >= trigger.wall_s]
    record_property("trigger_sim_s", round(trigger.sim_s, 1))
    record_property("modes_after_trigger", after)
    record_property("landing_distance_from_home_m", round(distance, 1))
    record_property("effective_speed_factor", round(recorder.effective_speed_factor(), 2))
