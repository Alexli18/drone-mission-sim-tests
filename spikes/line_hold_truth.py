"""True-versus-estimated cross-track of a survey line flight: a measurement harness.

The line-survey test checks the position PX4 *estimates* (MAVSDK telemetry). An
estimator that is wrong in a way the control loop follows passes that check while the
vehicle really wanders: the loop corrects the estimate, not the truth. This harness
measures both. It flies a line_survey scenario's line the way
tests/test_line_survey.py does (take off to the scenario height, mission run-in /
start / end / run-out, land) in a fresh PX4 SITL container, without the ROS
container, then reads PX4's ulog and reports, over the line (s from 0.5 m to L - 0.5 m):

* TRUE cross-track: vehicle_local_position_groundtruth projected on the line (max,
  RMS, dominant period of the sideways oscillation),
* the ESTIMATE's cross-track: vehicle_local_position (what the EKF reports; the
  test's cross-track check uses the MAVSDK position telemetry, which is this estimate),
* estimate minus truth (cross-track, along-track, height) and the height hold of both,
* line speed.

Usage (from the repository root, with the patched image built)::

    python spikes/line_hold_truth.py line_survey_markers --seeds 0 1 2
    python spikes/line_hold_truth.py line_survey_markers --set MPC_XY_VEL_P_ACC=2.0
    python spikes/line_hold_truth.py --analyse path/to/flight.ulg --scenario line_survey_markers

--analyse reads the first home_position of the ulog as the mission's home; a
flight run uses the home MAVSDK reported when it uploaded the mission. The two differ by
GNSS noise (~3 cm), which moves the true maximum by up to 0.01 m.

--set NAME=VALUE overrides scenario px4_params for this run only.

Repeats: PX4 SIH is deterministic on one machine (the sensor noise is a fixed
pseudo-random sequence in simulated time), so a plain repeat gives the same flight to
1 mm. A CI run differs because the MAVSDK commands reach PX4 at slightly different
simulated times. --seeds K imitates that: seed K waits K * --delay-step-s
simulated seconds in the hover before it starts the mission, which moves the line
through a different stretch of the noise sequence (seed 0: no wait).

Finding it was made for: with the default estimator settings the true cross-track
reached 0.35-0.44 m while the estimate showed 0.26-0.30 m; with the profile
scenarios/_low_altitude.yaml the true maximum is about 0.2 m. The profile's
EKF2_IMU_CTRL: 0 is a simulation-only setting (SIH's IMU has no bias).

An experiment, not framework code. Needs pyulog (pip install pyulog) and numpy.
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from framework.assertions import (
    assert_altitude,
    assert_cross_track,
    call,
    line_speed,
    line_window,
    wait_altitude,
    wait_landed_and_disarmed,
    wait_until,
)
from framework.config import load_config
from framework.mission import (
    line_waypoints,
    load_scenario,
    to_mission_plan,
)
from framework.sitl import (
    ARTIFACTS,
    MavsdkServer,
    SitlInstance,
    assert_params,
    connect,
    scaled_loss_params,
    wait_armable,
    wait_ready,
)
from framework.telemetry_log import TelemetryRecorder

RUN_M = 2.0           # survey.pass_rule.run_m (config.yaml)
ACCEPTANCE_M = 0.5    # survey.pass_rule.acceptance_radius_m
FLIGHT_HEIGHT_M = 0.15  # survey.pass_rule.flight_height_m
EDGE_M = 0.5          # the line's ends excluded from the true cross-track figures


def scenario_path(name: str) -> Path:
    return REPO / "scenarios" / f"{name}.yaml"


# ---------------------------------------------------------------- analysis (ulog)


def _series(ulog, name: str) -> dict[str, np.ndarray]:
    data = ulog.get_dataset(name).data
    return {k: np.asarray(v) for k, v in data.items()}


def _line_frame(line, north, east):
    h = math.radians(line.heading_deg)
    dn, de = north - line.start_north_m, east - line.start_east_m
    return dn * math.cos(h) + de * math.sin(h), -dn * math.sin(h) + de * math.cos(h)


def _ned_from_home(series, home_lat, home_lon):
    """North/east of a local-position series from home (the series' own origin moved to home)."""
    ref_lat = np.radians(series["ref_lat"])
    ref_lon = np.radians(series["ref_lon"])
    r = 6378137.0
    d_n = (ref_lat - math.radians(home_lat)) * r
    d_e = (ref_lon - math.radians(home_lon)) * r * math.cos(math.radians(home_lat))
    return series["x"] + d_n, series["y"] + d_e


def dominant_period_s(t: np.ndarray, d: np.ndarray) -> float | None:
    """Period of the strongest spectral line of ``d`` (resampled to 10 Hz, mean removed)."""
    if len(t) < 20 or t[-1] - t[0] < 10:
        return None
    grid = np.arange(t[0], t[-1], 0.1)
    x = np.interp(grid, t, d)
    x = x - x.mean()
    x = x * np.hanning(len(x))
    spec = np.abs(np.fft.rfft(x, n=8 * len(x)))
    freq = np.fft.rfftfreq(8 * len(x), 0.1)
    keep = (freq > 0.03) & (freq < 1.0)  # periods 1..33 s
    if not keep.any():
        return None
    return float(1.0 / freq[keep][np.argmax(spec[keep])])


def analyse_ulog(path: Path, line, home_lat: float | None, home_lon: float | None,
                 height_m: float) -> dict:
    from pyulog import ULog

    ulog = ULog(str(path))
    truth = _series(ulog, "vehicle_local_position_groundtruth")
    if home_lat is None:
        # The first home PX4 published (the flight reads it before it uploads the mission, which
        # is relative to it; later ones differ by ~3 cm of GNSS noise, and PX4 sets it again
        # on landing, 27 m away).
        home = ulog.get_dataset("home_position").data
        idx = np.nonzero(home["lat"] != 0.0)[0][0]
        home_lat, home_lon = float(home["lat"][idx]), float(home["lon"][idx])
    est = _series(ulog, "vehicle_local_position")
    out: dict = {"ulog": str(path)}

    def track(series):
        n, e = _ned_from_home(series, home_lat, home_lon)
        s, d = _line_frame(line, n, e)
        return series["timestamp"] / 1e6, s, d, -series["z"]

    t_t, s_t, d_t, z_t = track(truth)
    t_e, s_e, d_e, z_e = track(est)
    # Ground height: PX4's local z of the truth at rest before the first move (z = 0 at home).
    ground_t = float(np.median(-truth["z"][:50]))
    ground_e = float(np.median(-est["z"][:50]))
    h_t = z_t - ground_t
    h_e = z_e - ground_e

    # Line window (as the test does for the estimate: first s >= 0 to last s <= L) and the
    # figures without the ends.
    def window(s, lo, hi):
        idx = np.nonzero((s >= lo) & (s <= hi))[0]
        if len(idx) < 2:
            return None
        # first crossing of lo, last point with s <= hi (the vehicle also lands and may wander)
        return idx[0], idx[-1]

    L = line.length_m
    w_t = window(s_t, 0.0, L)
    w_e = window(s_e, 0.0, L)
    if w_t is None or w_e is None:
        return {**out, "error": "never crossed the line"}

    def sl(arr, w):
        return arr[w[0]: w[1] + 1]

    def figures(t, s, d, h, w):
        t, s, d, h = (sl(a, w) for a in (t, s, d, h))
        inner = (s >= EDGE_M) & (s <= L - EDGE_M)
        tt, dd = t[inner], d[inner]
        return {
            "window_s": [round(float(t[0]), 2), round(float(t[-1]), 2)],
            "cross_max_m": round(float(np.abs(dd).max()), 4),
            "cross_max_full_m": round(float(np.abs(d).max()), 4),
            "cross_rms_m": round(float(np.sqrt(np.mean(dd**2))), 4),
            "cross_mean_m": round(float(dd.mean()), 4),
            "period_s": None if dominant_period_s(tt, dd) is None
            else round(dominant_period_s(tt, dd), 2),
            "height_max_err_m": round(float(np.abs(h - height_m).max()), 4),
            "height_rms_err_m": round(float(np.sqrt(np.mean((h - height_m) ** 2))), 4),
            "line_speed_m_s": round(float((s[-1] - s[0]) / (t[-1] - t[0])), 4),
        }

    out["truth"] = figures(t_t, s_t, d_t, h_t, w_t)
    out["estimate"] = figures(t_e, s_e, d_e, h_e, w_e)

    # Estimate minus truth over the inner window, truth interpolated to the estimate's times.
    t0, t1 = out["estimate"]["window_s"]
    sel = (t_e >= t0) & (t_e <= t1) & (s_e >= EDGE_M) & (s_e <= L - EDGE_M)
    te = t_e[sel]
    ok = (te >= t_t[0]) & (te <= t_t[-1])
    te = te[ok]
    if len(te):
        dt_i = np.interp(te, t_t, d_t)
        st_i = np.interp(te, t_t, s_t)
        ht_i = np.interp(te, t_t, h_t)
        err_d = d_e[sel][ok] - dt_i
        err_s = s_e[sel][ok] - st_i
        err_h = h_e[sel][ok] - ht_i
        out["estimate_minus_truth"] = {
            "cross_max_m": round(float(np.abs(err_d).max()), 4),
            "cross_rms_m": round(float(np.sqrt(np.mean(err_d**2))), 4),
            "along_max_m": round(float(np.abs(err_s).max()), 4),
            "height_max_m": round(float(np.abs(err_h).max()), 4),
        }
    # Estimator resets in flight.
    out["xy_reset_counter_changes"] = int(np.count_nonzero(np.diff(est["xy_reset_counter"]))) \
        if "xy_reset_counter" in est else None
    return out


# ------------------------------------------------------------------- the flight


def load_params(name: str, overrides: dict, cfg) -> tuple:
    scenario = load_scenario(scenario_path(name))
    params = dict(scenario.px4_params)
    params.update(overrides)
    return scenario, scaled_loss_params(params, cfg)


async def fly(cfg, scenario, params: dict, test_id: str, delay_sim_s: float = 0.0) -> dict:
    """One flight of the scenario's line; returns events and the ulog path."""
    survey = scenario.survey
    line = survey.line
    h_v = scenario.altitude_m
    timeouts = cfg.timeouts_wall_s
    instance = SitlInstance(cfg, test_id, params,
                            env={"ROS_DOMAIN_ID": str(cfg.ros.domain_id)}, dds_own_ip=True)
    server = MavsdkServer(cfg, ARTIFACTS / "logs" / f"{test_id}.mavsdk_server.log")
    recorder = None
    info: dict = {}
    try:
        instance.start()
        server.start()
        drone = await connect(cfg, server)
        checked = dict(params)
        await assert_params(drone, checked, cfg)
        await wait_ready(drone, cfg)
        recorder = TelemetryRecorder(drone, cfg, test_id)
        await recorder.start()
        home = await anext(drone.telemetry.home())
        info["home"] = (home.latitude_deg, home.longitude_deg)
        plan, _ = to_mission_plan(line_waypoints(line, RUN_M), home.latitude_deg,
                                  home.longitude_deg, h_v, scenario.speed_m_s, ACCEPTANCE_M)
        await call(drone.action.set_takeoff_altitude(h_v), cfg, "set takeoff altitude")
        await call(drone.mission.set_return_to_launch_after_mission(False), cfg, "no RTL")
        await call(drone.mission.upload_mission(plan), cfg, "upload mission")
        await wait_armable(drone, cfg)
        await call(drone.action.arm(), cfg, "arm")
        await call(drone.action.takeoff(), cfg, "takeoff")
        await wait_altitude(recorder, h_v, FLIGHT_HEIGHT_M, timeouts.takeoff, cfg)
        if delay_sim_s > 0:
            until = recorder.sim_time_s() + delay_sim_s
            await wait_until(lambda: recorder.sim_time_s() >= until, 120, "start delay", recorder,
                             cfg.poll_interval_s)
        await call(drone.mission.start_mission(), cfg, "start mission")

        async def finished():
            return await call(drone.mission.is_mission_finished(), cfg, "is mission finished")

        await wait_until(finished, timeouts.mission, "mission finished", recorder,
                         cfg.poll_interval_s)
        # The figures tests/test_line_survey.py checks (MAVSDK telemetry).
        lat, lon = home.latitude_deg, home.longitude_deg
        t_in, t_out = line_window(recorder, line, lat, lon)
        info["test_checks"] = {
            "height_err_m": round(assert_altitude(recorder, h_v, math.inf, t_in, t_out), 4),
            "cross_track_max_m": round(assert_cross_track(recorder, line, lat, lon, math.inf), 4),
            "line_speed_m_s": round(line_speed(recorder, line, lat, lon), 4)}
        await call(drone.action.land(), cfg, "land")
        await wait_landed_and_disarmed(recorder, timeouts.land, cfg)
        info["effective_speed_factor"] = round(recorder.effective_speed_factor(), 2)
    finally:
        if recorder is not None:
            await recorder.stop()
        server.stop()
        instance.stop()
    ulogs = sorted(instance.data_dir.glob("**/*.ulg"))
    info["ulog"] = str(ulogs[-1]) if ulogs else None
    return info


def parse_set(items) -> dict:
    out = {}
    for item in items or []:
        key, value = item.split("=", 1)
        out[key] = float(value) if ("." in value or "e" in value.lower()) else int(value)
    return out


def report(key: str, **data) -> None:
    print(f"SPIKE {key}: {json.dumps(data, default=str)}", flush=True)


def one_line(res: dict) -> str:
    t, e, d = res["truth"], res["estimate"], res.get("estimate_minus_truth", {})
    return (f"truth max {t['cross_max_m']:.3f} rms {t['cross_rms_m']:.3f} T {t['period_s']} | "
            f"est max {e['cross_max_m']:.3f} rms {e['cross_rms_m']:.3f} | "
            f"est-truth max {d.get('cross_max_m')} | h truth {t['height_max_err_m']:.3f} "
            f"est {e['height_max_err_m']:.3f} | v {t['line_speed_m_s']:.3f}"
            + (f" | test xt {res['test_checks']['cross_track_max_m']:.3f}"
               f" h {res['test_checks']['height_err_m']:.3f}" if res.get("test_checks") else ""))


async def main_async(args) -> None:
    cfg = load_config()
    # ROS runs use px4.speed_factor_ros (tests/conftest.py).
    cfg = dataclasses.replace(cfg, px4=dataclasses.replace(
        cfg.px4, speed_factor=args.speed_factor or cfg.px4.speed_factor_ros))
    overrides = parse_set(args.set)
    for name in args.scenarios:
        scenario, params = load_params(name, overrides, cfg)
        line = scenario.survey.line
        for seed in args.seeds:
            tag = args.tag or ("base" if not overrides else "var")
            test_id = (f"hold_{name.removeprefix('line_survey_')}_{tag}_{seed}"
                       f"_{time.strftime('%H%M%S')}")  # a fresh data directory per flight
            flight = await fly(cfg, scenario, params, test_id, seed * args.delay_step_s)
            if not flight.get("ulog"):
                report(test_id, error="no ulog", **flight)
                continue
            res = analyse_ulog(Path(flight["ulog"]), line, *flight["home"], scenario.altitude_m)
            res["test_checks"] = flight.get("test_checks")
            report(test_id, scenario=name, overrides=overrides, speed_factor=cfg.px4.speed_factor,
                   effective_speed_factor=flight.get("effective_speed_factor"), **res)
            print(f"RESULT {test_id}: {one_line(res)}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("scenarios", nargs="*", help="scenario names (scenarios/NAME.yaml)")
    parser.add_argument("--seeds", type=int, nargs="+", default=[0], help="repeat indices")
    parser.add_argument("--set", nargs="*", default=[], help="NAME=VALUE PX4 parameter overrides")
    parser.add_argument("--delay-step-s", type=float, default=1.3,
                        help="simulated seconds of hover per seed before the mission starts")
    parser.add_argument("--tag", default=None, help="label in the output / artifact names")
    parser.add_argument("--speed-factor", type=int, default=None,
                        help="PX4 speed factor (default px4.speed_factor_ros)")
    parser.add_argument("--analyse", type=Path, default=None, help="analyse an existing ulog")
    parser.add_argument("--scenario", default=None, help="with --analyse: the scenario")
    args = parser.parse_args()
    if args.analyse:
        scenario = load_scenario(scenario_path(args.scenario))
        res = analyse_ulog(args.analyse, scenario.survey.line, None, None, scenario.altitude_m)
        print(json.dumps(res, indent=1))
        if "truth" in res:
            print(one_line(res))
        return
    if not args.scenarios:
        parser.error("name at least one scenario")
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
