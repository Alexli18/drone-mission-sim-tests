"""M1 smoke test: PX4 SITL boots headless and reports a usable vehicle state."""

import os
import warnings

import pytest

from framework.assertions import horizontal_distance_m, wait_for_stream, wait_until

pytestmark = pytest.mark.sitl


async def test_sitl_boots_and_reports_ready(cfg, drone, recorder, record_property, request):
    record_property("boot_to_ready_s", round(request.node.boot_to_ready_s, 2))
    timeout_s = cfg.timeouts_wall_s.health
    home_cfg = cfg.px4.home

    home = await wait_for_stream(drone.telemetry.home(), lambda h: True, timeout_s, "home position")
    home_error = horizontal_distance_m(home.latitude_deg, home.longitude_deg,
                                       home_cfg.lat, home_cfg.lon)
    assert home_error <= 1.0, f"home is {home_error:.2f} m from the configured home"

    position = await wait_for_stream(drone.telemetry.position(), lambda p: True, timeout_s,
                                     "position")
    position_error = horizontal_distance_m(position.latitude_deg, position.longitude_deg,
                                           home.latitude_deg, home.longitude_deg)
    assert position_error <= 1.0, f"vehicle is {position_error:.2f} m from home on the ground"

    battery = await wait_for_stream(drone.telemetry.battery(), lambda b: True, timeout_s,
                                    "battery")
    assert battery.remaining_percent > 90, f"battery at {battery.remaining_percent}%"

    mode = await wait_for_stream(drone.telemetry.flight_mode(), lambda m: True, timeout_s,
                                 "flight mode")
    assert mode.name in {"READY", "HOLD"}, f"unexpected flight mode on the ground: {mode.name}"

    # Measure the effective speed factor over a few wall-clock seconds (spike S2).
    await wait_until(lambda: recorder.effective_speed_factor() is not None
                     and recorder.sim_time_s() is not None, 10, "simulated clock running",
                     recorder, cfg.poll_interval_s)
    factor = recorder.effective_speed_factor()
    record_property("effective_speed_factor", round(factor, 2))
    floor = cfg.min_effective_speed_factor
    message = f"effective speed factor {factor:.1f} below floor {floor:.1f}"
    if os.environ.get("CI") == "true":
        assert factor >= floor, message
    elif factor < floor:
        # Docker Desktop on macOS holds about half the requested factor; the floor
        # guards the CI runner, so locally it is only reported.
        warnings.warn(message, stacklevel=1)
