"""M2: arm, take off, hover, land, disarm, with no failsafe on the way."""

import time

import pytest

from framework.assertions import (
    assert_altitude,
    assert_no_unexpected_modes,
    call,
    wait_altitude,
    wait_landed_and_disarmed,
    wait_until,
)
from framework.sitl import wait_armable

pytestmark = pytest.mark.sitl

# Hold is normal here (PX4 holds after takeoff); any other mode means a failsafe.
ALLOWED_MODES = {"READY", "TAKEOFF", "HOLD", "LAND"}


@pytest.mark.scenario("hover_and_land.yaml")
async def test_hover_and_land(cfg, scenario, drone, recorder, record_property):
    timeouts = cfg.timeouts_wall_s
    tol = cfg.tolerances.altitude_m

    await wait_armable(drone, cfg)
    await call(drone.action.arm(), cfg, "arm")
    takeoff_wall = time.monotonic()
    takeoff_sim = recorder.sim_time_s()
    await call(drone.action.takeoff(), cfg, "takeoff")
    await wait_altitude(recorder, scenario.altitude_m, tol, timeouts.takeoff, cfg)

    hover_start = recorder.sim_time_s()
    await wait_until(lambda: recorder.sim_time_s() >= hover_start + scenario.hover_s,
                     timeouts.hover, f"{scenario.hover_s} simulated seconds of hover",
                     recorder, cfg.poll_interval_s)
    max_error = assert_altitude(recorder, scenario.altitude_m, tol, hover_start,
                                recorder.sim_time_s())

    await call(drone.action.land(), cfg, "land")
    await wait_landed_and_disarmed(recorder, timeouts.land, cfg)
    assert_no_unexpected_modes(recorder, ALLOWED_MODES, since_wall_s=takeoff_wall)

    record_property("hover_max_altitude_error_m", round(max_error, 2))
    record_property("flight_duration_sim_s", round(recorder.sim_time_s() - takeoff_sim, 1))
    record_property("effective_speed_factor", round(recorder.effective_speed_factor(), 2))
