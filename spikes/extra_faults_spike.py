"""Spikes for extra failure scenarios: barometer, magnetometer and motor loss.

Each spike boots a fresh PX4, flies the 100 x 100 m failsafe survey, injects
one failure at waypoint 3 with MAVSDK, and reports the injection result, the
flight mode sequence, PX4 status texts, and the final altitude and state.
Usage: python spikes/extra_faults_spike.py {baro|mag|motor}. Experiment code.
"""

import argparse
import asyncio
import json
import time

from mavsdk_grpc.failure import FailureType, FailureUnit

from framework.config import load_baseline_params, load_config
from framework.mission import Area, lawnmower_waypoints, to_mission_plan
from framework.sitl import (
    ARTIFACTS,
    MavsdkServer,
    SitlInstance,
    connect,
    scaled_loss_params,
    wait_armable,
    wait_ready,
)
from framework.telemetry_log import TelemetryRecorder

AREA = Area(origin_north_m=10, origin_east_m=10, width_m=100, height_m=100, line_spacing_m=10)

SPIKES = {
    "baro": (FailureUnit.SENSOR_BARO, 0, {"SYS_FAILURE_EN": 1}),
    "mag": (FailureUnit.SENSOR_MAG, 0, {"SYS_FAILURE_EN": 1}),
    # Motor 1; CA_FAILURE_MODE lets the allocator drop it, FD_ACT_EN detects it,
    # COM_ACT_FAIL_ACT 2 = Land on an actuator failure.
    "motor": (FailureUnit.SYSTEM_MOTOR, 1, {"SYS_FAILURE_EN": 1, "CA_FAILURE_MODE": 1,
                                             "FD_ACT_EN": 1, "COM_ACT_FAIL_ACT": 2}),
}


async def guarded(coro, timeout=30):
    try:
        return f"ok: {await asyncio.wait_for(coro, timeout)}"
    except Exception as exc:  # noqa: BLE001 - the spike wants every failure mode
        return f"{type(exc).__name__}: {exc}"


async def run(spike):
    unit, instance, extra = SPIKES[spike]
    cfg = load_config()
    params = scaled_loss_params(dict(load_baseline_params(), **extra), cfg)
    name = f"extra_{spike}"
    sitl = SitlInstance(cfg, name, params)
    sitl.start()
    server = MavsdkServer(cfg, ARTIFACTS / "logs" / f"{name}.mavsdk_server.log")
    server.start()
    events, texts, recorder = {}, [], None
    try:
        drone = await connect(cfg, server)
        await wait_ready(drone, cfg)
        recorder = TelemetryRecorder(drone, cfg, name)
        await recorder.start()

        async def collect():
            async for st in drone.telemetry.status_text():
                texts.append((recorder.sim_time_s(), st.type.name, st.text.strip()))

        text_task = asyncio.create_task(collect())
        home = await anext(drone.telemetry.home())
        plan, _ = to_mission_plan(lawnmower_waypoints(AREA), home.latitude_deg,
                                  home.longitude_deg, 15, 5, 1.5)
        events["set_rtl"] = await guarded(drone.mission.set_return_to_launch_after_mission(False))
        events["upload"] = await guarded(drone.mission.upload_mission(plan))
        await wait_armable(drone, cfg)
        events["arm"] = await guarded(drone.action.arm())
        events["start"] = await guarded(drone.mission.start_mission())

        start = time.monotonic()
        while time.monotonic() - start < 60:  # wait for waypoint 3
            progress = recorder.latest("mission_progress")
            if progress is not None and progress.current >= 3:
                break
            await asyncio.sleep(0.1)
        events["trigger_sim_t"] = recorder.sim_time_s()
        events["inject"] = await guarded(drone.failure.inject(unit, FailureType.OFF, instance))

        min_alt, took_off, start = None, True, time.monotonic()
        while time.monotonic() - start < 90:
            pos = recorder.latest("position")
            if pos is not None:
                min_alt = pos.relative_altitude_m if min_alt is None else min(
                    min_alt, pos.relative_altitude_m)
            landed = recorder.latest("landed_state")
            if (took_off and recorder.latest("armed") is False and landed is not None
                    and landed.name == "ON_GROUND"):
                break
            await asyncio.sleep(0.2)
        pos = recorder.latest("position")
        print("RESULT " + json.dumps({
            "spike": spike,
            "events": events,
            "modes": [(round(t, 1) if t else None, m) for t, _, m in recorder.mode_history()],
            "final_rel_alt_m": round(pos.relative_altitude_m, 1) if pos else None,
            "final_armed": recorder.latest("armed"),
            "final_landed": str(recorder.latest("landed_state")),
            "final_progress": str(recorder.latest("mission_progress")),
            "status_texts": [(round(t, 1) if t else None, k, x) for t, k, x in texts
                             if "Preflight" not in x][-30:],
        }, default=str), flush=True)
        text_task.cancel()
    finally:
        if recorder is not None:
            await recorder.stop()
        server.stop()
        sitl.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("spike", choices=sorted(SPIKES))
    asyncio.run(run(parser.parse_args().spike))
