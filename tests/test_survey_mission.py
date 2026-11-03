"""M3: fly every scenario of type ``mission`` as an uploaded MAVSDK mission."""

import pytest

from framework.assertions import (
    assert_duration,
    assert_waypoints_reached,
    call,
    wait_for_stream,
    wait_landed_and_disarmed,
    wait_until,
)
from framework.mission import lawnmower_waypoints, list_scenarios, to_mission_plan
from framework.sitl import wait_armable

pytestmark = pytest.mark.sitl


@pytest.mark.parametrize("scenario", list_scenarios("mission"), indirect=True,
                         ids=lambda path: path.stem)
async def test_survey_mission(cfg, scenario, drone, recorder, record_property):
    timeouts = cfg.timeouts_wall_s
    home = await wait_for_stream(drone.telemetry.home(), lambda h: True, timeouts.health,
                                 "home position")
    waypoints = lawnmower_waypoints(scenario.area)
    # PX4 counts a waypoint reached inside the acceptance radius; half the check's
    # tolerance leaves room for position sampling.
    plan, waypoints_latlon = to_mission_plan(
        waypoints, home.latitude_deg, home.longitude_deg, scenario.altitude_m,
        scenario.speed_m_s, cfg.tolerances.waypoint_m / 2)

    # MAVSDK adds the Return item at upload time, so this comes first.
    await call(drone.mission.set_return_to_launch_after_mission(True), cfg, "set RTL after mission")
    await call(drone.mission.upload_mission(plan), cfg, "upload mission")
    await wait_armable(drone, cfg)
    await call(drone.action.arm(), cfg, "arm")
    await call(drone.mission.start_mission(), cfg, "start mission")

    def mode_is_mission():
        mode = recorder.latest("flight_mode")
        return mode is not None and mode.name == "MISSION"

    await wait_until(mode_is_mission, timeouts.mission_start, "flight mode MISSION", recorder,
                     cfg.poll_interval_s)
    start_sim = recorder.sim_time_s()

    async def finished():
        return await call(drone.mission.is_mission_finished(), cfg, "is mission finished")

    await wait_until(finished, timeouts.mission, "mission finished", recorder,
                     cfg.poll_interval_s)
    end_sim = recorder.sim_time_s()
    await wait_landed_and_disarmed(recorder, timeouts.land, cfg)

    worst_miss = assert_waypoints_reached(recorder, waypoints_latlon, cfg.tolerances.waypoint_m)
    duration = assert_duration(start_sim, end_sim, scenario.duration_bounds_s, recorder)
    record_property("waypoints", len(waypoints))
    record_property("max_waypoint_miss_m", round(worst_miss, 2))
    record_property("mission_duration_sim_s", round(duration, 1))
    record_property("effective_speed_factor", round(recorder.effective_speed_factor(), 2))
