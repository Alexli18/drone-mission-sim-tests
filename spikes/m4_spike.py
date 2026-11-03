"""M4 spikes S5-S9 (see docs/designs/drone_mission_sim_tests.md, Open Questions).

Each spike boots a fresh PX4, flies the 100 x 100 m failsafe survey, applies
one fault mechanism, and reports what PX4 did: command results, the flight
mode sequence, PX4 status texts, and whether telemetry kept flowing. Run by
.github/workflows/spike-m4.yml, one matrix job per spike. This is an
experiment, not framework code.
"""

import argparse
import asyncio
import json
import subprocess
import time

from mavsdk_grpc.failure import FailureType, FailureUnit
from mavsdk_grpc.geofence import FenceType, GeofenceData, Point, Polygon

from framework.config import load_baseline_params, load_config
from framework.mission import Area, lawnmower_waypoints, offset_to_latlon, to_mission_plan
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
FENCE = [(-20, -20), (30, -20), (30, 130), (-20, 130)]  # north, east from home
IPTABLES_RULE = ["OUTPUT", "-o", "lo", "-p", "udp", "--dport", "14580", "-j", "DROP"]

SPIKES = {
    "s5_link": {"NAV_DLL_ACT": 2, "COM_DL_LOSS_T": 10, "COM_DLL_EXCEPT": 0, "SYS_FAILURE_EN": 1},
    "s6_iptables": {"NAV_DLL_ACT": 2, "COM_DL_LOSS_T": 10, "COM_DLL_EXCEPT": 0},
    "s5_rc": {"COM_RC_IN_MODE": 1, "NAV_RCL_ACT": 2, "COM_RCL_EXCEPT": 0, "COM_RC_LOSS_T": 0.5,
              "SYS_FAILURE_EN": 1},
    "s7_rc_stop": {"COM_RC_IN_MODE": 1, "NAV_RCL_ACT": 2, "COM_RCL_EXCEPT": 0,
                   "COM_RC_LOSS_T": 0.5},
    "s8_gps": {"SYS_FAILURE_EN": 1, "EKF2_NOAID_TOUT": 5000000, "COM_POS_FS_EPH": 5},
    "s9_fence_before": {"GF_ACTION": 3, "GF_PREDICT": 0},
    "s9_fence_after": {"GF_ACTION": 3, "GF_PREDICT": 0},
    "s9_maxdist": {"GF_ACTION": 3, "GF_MAX_HOR_DIST": 30},
}


def report(key, **data):
    print(f"SPIKE {key}: {json.dumps(data, default=str)}", flush=True)


async def guarded(label, coro, timeout=30):
    """Run one MAVSDK call; return its result or the error, never raise."""
    try:
        return {"ok": True, "result": str(await asyncio.wait_for(coro, timeout))}
    except Exception as exc:  # noqa: BLE001 - the spike wants to see every failure mode
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}", "call": label}


async def run(spike):
    cfg = load_config()
    params = scaled_loss_params(dict(load_baseline_params(), **SPIKES[spike]), cfg)
    sitl = SitlInstance(cfg, spike, params)
    sitl.start()
    server = MavsdkServer(cfg, ARTIFACTS / "logs" / f"{spike}.mavsdk_server.log")
    server.start()
    events = {}
    texts = []
    stream_task = None
    recorder = None
    try:
        drone = await connect(cfg, server)
        await wait_ready(drone, cfg)
        recorder = TelemetryRecorder(drone, cfg, spike)
        await recorder.start()

        async def collect_texts():
            async for st in drone.telemetry.status_text():
                texts.append((recorder.sim_time_s(), st.type.name, st.text))

        text_task = asyncio.create_task(collect_texts())
        home = await anext(drone.telemetry.home())
        plan, _ = to_mission_plan(lawnmower_waypoints(AREA), home.latitude_deg,
                                  home.longitude_deg, 15, 5, 1.5)
        fence = GeofenceData(
            [Polygon([Point(*offset_to_latlon(home.latitude_deg, home.longitude_deg, n, e))
                      for n, e in FENCE], FenceType.INCLUSION)], [])

        if spike in ("s5_rc", "s7_rc_stop"):
            async def stream_sticks():
                while True:
                    await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
                    await asyncio.sleep(0.02)  # 50 Hz wall-clock, the design's rate
            stream_task = asyncio.create_task(stream_sticks())
            await asyncio.sleep(1)
        if spike == "s9_fence_before":
            events["upload_geofence"] = await guarded("upload_geofence",
                                                      drone.geofence.upload_geofence(fence))

        events["set_rtl"] = await guarded(
            "rtl", drone.mission.set_return_to_launch_after_mission(False))
        events["upload_mission"] = await guarded("upload_mission",
                                                 drone.mission.upload_mission(plan))
        await wait_armable(drone, cfg)
        events["arm"] = await guarded("arm", drone.action.arm())
        events["start_mission"] = await guarded("start_mission", drone.mission.start_mission())
        if spike == "s9_fence_after":
            events["upload_geofence"] = await guarded("upload_geofence",
                                                      drone.geofence.upload_geofence(fence))

        # Trigger at waypoint 3 (GPS: 60 simulated seconds after MISSION).
        start_wall = time.monotonic()
        trigger_sim = None
        while time.monotonic() - start_wall < 60:
            progress = recorder.latest("mission_progress")
            mode = recorder.latest("flight_mode")
            sim = recorder.sim_time_s()
            if spike == "s8_gps":
                mission_t = next((t for t, _, m in recorder.mode_history() if m == "MISSION"), None)
                if mission_t is not None and sim >= mission_t + 60:
                    break
            elif spike.startswith("s9"):
                break  # no trigger: the fence acts on its own
            elif progress is not None and progress.current >= 3:
                break
            if mode is not None and mode.name not in ("MISSION", "HOLD", "TAKEOFF", "READY"):
                break
            await asyncio.sleep(0.1)
        trigger_sim = recorder.sim_time_s()
        events["trigger_sim_t"] = trigger_sim

        if spike == "s5_link":
            events["inject"] = await guarded("inject", drone.failure.inject(
                FailureUnit.SYSTEM_MAVLINK_SIGNAL, FailureType.OFF, 0))
        elif spike == "s5_rc":
            events["inject"] = await guarded("inject", drone.failure.inject(
                FailureUnit.SYSTEM_RC_SIGNAL, FailureType.OFF, 0))
        elif spike == "s8_gps":
            events["inject"] = await guarded("inject", drone.failure.inject(
                FailureUnit.SENSOR_GPS, FailureType.OFF, 0))
        elif spike == "s7_rc_stop":
            stream_task.cancel()
            events["inject"] = "stopped manual control stream"
        elif spike == "s6_iptables":
            result = subprocess.run(  # noqa: ASYNC221 - one short call
                ["sudo", "iptables", "-I", *IPTABLES_RULE], check=False,
                capture_output=True, text=True)
            events["inject"] = f"iptables rc={result.returncode} {result.stderr.strip()}"

        positions_at_trigger = len(recorder.positions())
        obs_start = time.monotonic()
        while time.monotonic() - obs_start < 90:
            landed = recorder.latest("landed_state")
            if (recorder.latest("armed") is False and landed is not None
                    and landed.name == "ON_GROUND"):
                break
            await asyncio.sleep(0.2)

        report(spike,
               events=events,
               modes=[(round(t, 1) if t else None, m) for t, _, m in recorder.mode_history()],
               telemetry_after_trigger=len(recorder.positions()) - positions_at_trigger,
               final_armed=recorder.latest("armed"),
               final_landed=str(recorder.latest("landed_state")),
               final_progress=str(recorder.latest("mission_progress")),
               effective_speed_factor=recorder.effective_speed_factor(),
               status_texts=[(round(t, 1) if t else None, k, x) for t, k, x in texts][-40:])
        text_task.cancel()
    finally:
        if stream_task is not None:
            stream_task.cancel()
        if spike == "s6_iptables":
            subprocess.run(["sudo", "iptables", "-D", *IPTABLES_RULE],  # noqa: ASYNC221
                           check=False, capture_output=True)
        if recorder is not None:
            await recorder.stop()
        server.stop()
        sitl.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("spike", choices=sorted(SPIKES))
    asyncio.run(run(parser.parse_args().spike))
