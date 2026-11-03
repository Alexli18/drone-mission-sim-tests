"""Geofence follow-up to spike S9: breach the fence without a mission.

S9 showed PX4 rejects any mission with a waypoint outside the fence. This
spike takes off inside an inclusion fence and leaves it either with a goto
command or with offboard velocity setpoints, then records what PX4 does.
Usage: python spikes/geofence_spike.py {goto|offboard}. Experiment code.
"""

import argparse
import asyncio
import json
import math
import time

from mavsdk_grpc.geofence import FenceType, GeofenceData, Point, Polygon
from mavsdk_grpc.offboard import VelocityNedYaw

from framework.config import load_baseline_params, load_config
from framework.mission import offset_to_latlon
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

FENCE = [(-20, -20), (30, -20), (30, 130), (-20, 130)]  # north, east from home


async def guarded(coro, timeout=30):
    try:
        return f"ok: {await asyncio.wait_for(coro, timeout)}"
    except Exception as exc:  # noqa: BLE001 - the spike wants every failure mode
        return f"{type(exc).__name__}: {exc}"


async def run(variant):
    cfg = load_config()
    params = scaled_loss_params(dict(load_baseline_params(), GF_ACTION=3, GF_PREDICT=0), cfg)
    name = f"geofence_{variant}"
    sitl = SitlInstance(cfg, name, params)
    sitl.start()
    server = MavsdkServer(cfg, ARTIFACTS / "logs" / f"{name}.mavsdk_server.log")
    server.start()
    events, texts, recorder, pusher = {}, [], None, None
    try:
        drone = await connect(cfg, server)
        await wait_ready(drone, cfg)
        recorder = TelemetryRecorder(drone, cfg, name)
        await recorder.start()

        async def collect():
            async for st in drone.telemetry.status_text():
                texts.append((recorder.sim_time_s(), st.text.strip()))

        text_task = asyncio.create_task(collect())
        home = await anext(drone.telemetry.home())
        fence = GeofenceData([Polygon(
            [Point(*offset_to_latlon(home.latitude_deg, home.longitude_deg, n, e))
             for n, e in FENCE], FenceType.INCLUSION)], [])
        events["upload_geofence"] = await guarded(drone.geofence.upload_geofence(fence))
        await wait_armable(drone, cfg)
        events["arm"] = await guarded(drone.action.arm())
        events["takeoff"] = await guarded(drone.action.takeoff())
        start = time.monotonic()
        while time.monotonic() - start < 60:  # wait for TAKEOFF -> HOLD
            modes = [m for _, _, m in recorder.mode_history()]
            if "TAKEOFF" in modes and modes[-1] == "HOLD":
                break
            await asyncio.sleep(0.1)
        events["breach_start_sim_t"] = recorder.sim_time_s()

        if variant == "goto":
            lat, lon = offset_to_latlon(home.latitude_deg, home.longitude_deg, 60, 0)
            events["goto"] = await guarded(drone.action.goto_location(
                lat, lon, home.absolute_altitude_m + 15, float("nan")))
        else:
            north = VelocityNedYaw(3.0, 0.0, 0.0, 0.0)
            events["set_initial_setpoint"] = await guarded(drone.offboard.set_velocity_ned(north))
            events["offboard_start"] = await guarded(drone.offboard.start())

            async def push():  # offboard needs a continuous setpoint stream
                while True:
                    await drone.offboard.set_velocity_ned(north)
                    await asyncio.sleep(0.05)
            pusher = asyncio.create_task(push())

        max_north = 0.0
        start = time.monotonic()
        while time.monotonic() - start < 90:
            pos = recorder.latest("position")
            if pos is not None:
                dlat = math.radians(pos.latitude_deg - home.latitude_deg)
                max_north = max(max_north, dlat * 6_371_000)
            mode = recorder.latest("flight_mode")
            if pusher is not None and mode is not None and mode.name != "OFFBOARD":
                pusher.cancel()  # the fence took over; stop pushing
            landed = recorder.latest("landed_state")
            if recorder.latest("armed") is False and landed and landed.name == "ON_GROUND":
                break
            await asyncio.sleep(0.2)

        print("RESULT " + json.dumps({
            "variant": variant,
            "events": events,
            "modes": [(round(t, 1) if t else None, m) for t, _, m in recorder.mode_history()],
            "max_north_m": round(max_north, 1),
            "final_armed": recorder.latest("armed"),
            "final_landed": str(recorder.latest("landed_state")),
            "status_texts": [(round(t, 1) if t else None, x) for t, x in texts
                             if "Preflight" not in x],
        }, default=str), flush=True)
        text_task.cancel()
    finally:
        if pusher is not None:
            pusher.cancel()
        if recorder is not None:
            await recorder.stop()
        server.stop()
        sitl.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("variant", choices=["goto", "offboard"])
    asyncio.run(run(parser.parse_args().variant))
