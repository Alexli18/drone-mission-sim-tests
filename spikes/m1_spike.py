"""M1 spikes S1-S4 (see docs/designs/drone_mission_sim_tests.md, Open Questions).

Run by .github/workflows/spike-m1.yml. Each step prints a "SPIKE <id>:" line so
the results can be read from the job log and written back into the design.
This script is an experiment, not framework code.
"""

import argparse
import asyncio
import itertools
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import mavsdk_grpc
from mavsdk_grpc import System

IMAGE = os.environ["PX4_IMAGE"]
BASELINE = {
    "COM_FAIL_ACT_T": 5,
    "RTL_RETURN_ALT": 30,
    "RTL_LAND_DELAY": 0,
    "COM_DISARM_LAND": 2,
    "MPC_XY_CRUISE": 5,
    "MIS_TAKEOFF_ALT": 15,
    "MIS_TKO_LAND_REQ": 0,
    "NAV_DLL_ACT": 0,
    "SIM_BAT_MIN_PCT": 50,
    "COM_RC_OVERRIDE": 0,
}
SPEED = 10


def report(spike, **data):
    print(f"SPIKE {spike}: {json.dumps(data, default=str)}", flush=True)


def docker(*args, check=True):
    return subprocess.run(["docker", *args], capture_output=True, text=True, check=check)


def start_container(name, data_dir, params):
    data_dir.mkdir(parents=True)
    cmd = [
        "run", "-d", "--pull", "never", "--network", "host", "--name", name,
        "--user", f"{os.getuid()}:{os.getgid()}",
        "-e", "XDG_DATA_HOME=/px4data",
        "-e", "PX4_SIM_MODEL=sihsim_quadx",
        "-e", f"PX4_SIM_SPEED_FACTOR={SPEED}",
        "--mount", f"type=bind,source={data_dir.resolve()},target=/px4data",
    ]
    for key, value in params.items():
        cmd += ["-e", f"PX4_PARAM_{key}={value}"]
    docker(*cmd, IMAGE)


def stop_container(name):
    logs = docker("logs", name, check=False)
    docker("stop", "-t", "10", name, check=False)
    docker("rm", "-f", name, check=False)
    return logs.stdout + logs.stderr


async def first(gen_factory, predicate, timeout):
    async def run():
        async for value in gen_factory():
            if predicate(value):
                return value
    return await asyncio.wait_for(run(), timeout)


async def connect(port):
    drone = System(port=port)
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    return drone


async def read_params(drone, names):
    all_params = await asyncio.wait_for(drone.param.get_all_params(), 30)
    ints = {p.name: p.value for p in all_params.int_params}
    floats = {p.name: p.value for p in all_params.float_params}
    out = {}
    for name in names:
        if name in ints:
            out[name] = ("int", ints[name])
        elif name in floats:
            out[name] = ("float", floats[name])
        else:
            out[name] = ("unknown", None)
    return out


async def s1_s2_s4(workdir):
    name = "spike-main"
    t0 = time.monotonic()
    start_container(name, workdir / "main", BASELINE)
    report("S1", step="container started")
    drone = await connect(50051)
    await first(drone.core.connection_state, lambda s: s.is_connected, 60)
    t_conn = time.monotonic()
    await first(drone.telemetry.health,
                lambda h: h.is_global_position_ok and h.is_home_position_ok, 120)
    t_health = time.monotonic()
    await first(drone.telemetry.health, lambda h: h.is_armable, 120)
    t_armable = time.monotonic()
    report("S2", boot_to_connect_s=round(t_conn - t0, 1),
           boot_to_health_s=round(t_health - t0, 1),
           boot_to_armable_s=round(t_armable - t0, 1))

    mode = await asyncio.wait_for(drone.telemetry.flight_mode().__anext__(), 10)
    params = await read_params(drone, BASELINE)
    report("S1", preflight_mode=str(mode), params=params)

    # S4 and S2: simulated clock and effective speed factor, idle and in flight.
    await asyncio.wait_for(drone.telemetry.set_rate_position(5.0), 10)
    await asyncio.wait_for(drone.telemetry.set_rate_imu(5.0), 10)

    async def measure(label, seconds):
        imu_first = await asyncio.wait_for(drone.telemetry.imu().__anext__(), 10)
        w0 = time.monotonic()
        stamps = []

        async def collect():
            async for pos in drone.telemetry.position():
                stamps.append(time.monotonic())
                if time.monotonic() - w0 > seconds:
                    return
        await asyncio.wait_for(collect(), seconds + 20)
        imu_last = await asyncio.wait_for(drone.telemetry.imu().__anext__(), 10)
        w1 = time.monotonic()
        sim = (imu_last.timestamp_us - imu_first.timestamp_us) / 1e6
        gaps = [b - a for a, b in itertools.pairwise(stamps)]
        report("S4" if label == "idle" else "S2", phase=label,
               wall_s=round(w1 - w0, 2), imu_sim_s=round(sim, 2),
               effective_factor=round(sim / (w1 - w0), 2),
               position_msgs=len(stamps),
               position_wall_hz=round(len(stamps) / (w1 - w0), 1),
               max_position_gap_s=round(max(gaps), 3) if gaps else None)

    await measure("idle", 10)
    await asyncio.wait_for(drone.action.arm(), 30)
    await asyncio.wait_for(drone.action.takeoff(), 30)
    await measure("flying", 15)
    await asyncio.wait_for(drone.action.land(), 30)
    await first(drone.telemetry.armed, lambda a: not a, 120)

    bc = docker("exec", name, "sh", "-c", "command -v bc || echo missing", check=False)
    report("S3", bc=bc.stdout.strip())
    logs = stop_container(name)
    tail = [line for line in logs.splitlines() if "nsh>" not in line]
    report("S1", log_lines=len(tail))
    for line in tail[:120]:
        print(f"S1-LOG {line}")
    files = sorted(str(p.relative_to(workdir / "main"))
                   for p in (workdir / "main").rglob("*") if p.is_file())
    report("S1", data_dir_files=files[:50])


async def s3(workdir):
    results = []
    for i, (dl, rc) in enumerate([(7, 0.3), (13, 0.7)]):
        name = f"spike-s3-{i}"
        params = dict(BASELINE, COM_DL_LOSS_T=dl, COM_RC_LOSS_T=rc)
        start_container(name, workdir / f"s3-{i}", params)
        drone = await connect(50052 + i)
        await first(drone.core.connection_state, lambda s: s.is_connected, 60)
        await first(drone.telemetry.health, lambda h: h.is_home_position_ok, 120)
        back = await read_params(drone, ["COM_DL_LOSS_T", "COM_RC_LOSS_T"])
        results.append({"configured": {"COM_DL_LOSS_T": dl, "COM_RC_LOSS_T": rc},
                        "read_back": back})
        stop_container(name)
    report("S3", speed_factor=SPEED, results=results)


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workdir", type=Path, default=Path("spike-out"))
    args = parser.parse_args()
    report("env", mavsdk_grpc_path=str(Path(mavsdk_grpc.__file__).parent),
           python=sys.version.split()[0])
    try:
        await s1_s2_s4(args.workdir)
    finally:
        stop_container("spike-main")
    await s3(args.workdir)


if __name__ == "__main__":
    asyncio.run(main())
