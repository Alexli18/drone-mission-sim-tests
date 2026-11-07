"""Fly every ``line_survey`` scenario with the survey pipeline running in a ROS 2 container.

PX4 SIH flies the line through MAVSDK (as in the other flight tests); the ROS container
runs the toy sensor, the georeferencing node and the marker detector; the test talks
to it over rosbridge only (``/survey/status``, ``/sim/capture_truth``, ``rosapi``,
``/survey/stop``, ``/markers/finalize``). See docs/line-survey.md for the steps and the
pass rules; the numbers in the comments are the pass rules' numbers.
"""

from __future__ import annotations

import json
import math
import os

import pytest
import yaml

pytest.importorskip("roslibpy")

from framework import replay_check
from framework.assertions import (
    FlightAssertionError,
    assert_altitude,
    assert_cross_track,
    call,
    line_speed,
    line_window,
    wait_altitude,
    wait_for_stream,
    wait_landed_and_disarmed,
    wait_until,
)
from framework.mission import (
    ScenarioError,
    line_waypoints,
    list_scenarios,
    load_scenario,
    to_mission_plan,
)
from framework.rosbridge import record_name, wait_for_topic
from framework.sitl import ARTIFACTS, wait_armable
from framework.survey import (
    capture_stream_failures,
    capture_window_start,
    ds_m,
    evaluate_markers,
    node_params,
    reason_counts,
    sensor_lag_s,
    takeoff_sim_s,
    truth_figures,
)

STATUS = "/survey/status"
TRUTH = "/sim/capture_truth"
GEOREF = "/survey/georef_capture"
STATUS_TYPE = "sensor_bridge_msgs/msg/SurveyStatus"
TRUTH_TYPE = "survey_demo_msgs/msg/CaptureTruth"
GEOREF_TYPE = "sensor_bridge_msgs/msg/GeorefCapture"
CHECKED_NODES = ("/georeference_node", "/marker_detector_node")


def _params():
    params = []
    for path in list_scenarios("line_survey"):
        try:
            schedule = load_scenario(path).survey.schedule
        except ScenarioError:
            # Marks from the raw file; the scenario fixture raises the error in the test
            # itself, so a broken file fails its own test, not every run's collection.
            schedule = (yaml.safe_load(path.read_text()) or {}).get("schedule", "push")
        marks = [pytest.mark.sitl, pytest.mark.ros]
        if schedule == "weekly":
            marks.append(pytest.mark.weekly)
        params.append(pytest.param(path, id=path.stem, marks=marks))
    return params


async def assert_no_sim_subscriptions(rosbridge, cfg, when: str) -> None:
    """The nodes under test subscribe to no ``/sim/*`` topic (they must not see the truth)."""
    for node in CHECKED_NODES:
        details = await rosbridge.call_service(
            "/rosapi/node_details", "rosapi_msgs/srv/NodeDetails", {"node": node},
            cfg.timeouts_wall_s.mavsdk_call)
        subscribing = list(details.get("subscribing", []))
        if not subscribing:
            raise FlightAssertionError(f"{node} known to rosapi ({when})",
                                       f"no subscriptions listed: {details}")
        sim = [t for t in subscribing if t.startswith("/sim/")]
        if sim:
            raise FlightAssertionError(f"{node} subscribes to no /sim/* topic ({when})",
                                       f"subscribes to {sim}")


def recorded(rosbridge, topic: str) -> list[dict]:
    """The messages the client recorded for ``topic`` (JSON lines written as they arrive)."""
    path = rosbridge.record_dir / f"{record_name(topic)}.jsonl"
    return [json.loads(line)["msg"] for line in path.read_text().splitlines() if line.strip()]


async def replay(cfg, scenario, test_id, ros2, rosbridge, status: dict, live_markers: list,
                 last: int, takeoff_s: float | None, record_property) -> list[tuple[str, str]]:
    """Replay check: replay the live recording; return the (expected, actual) failures.

    The live container is stopped first (the bag is closed); a fresh georeferencing node
    and detector read ``/survey/capture`` and ``/survey/pose_used`` from the bag. The
    whole recording is replayed (the poses before takeoff are the first in-flight
    captures' pre-roll), but only the captures whose first sample is at or after takeoff
    are compared: the sensor captures on the ground from launch, and those captures are
    not part of the check.
    """
    from framework.ros2 import Ros2Instance

    timeouts = cfg.timeouts_wall_s
    rule = cfg.survey.pass_rule
    live_dir = ros2.out_dir
    ros2.stop()
    live_decisions = replay_check.read_decisions(live_dir / "decisions.jsonl")
    errors = replay_check.recording_errors(replay_check.bag_message_counts(live_dir / "bag"),
                                           status, live_decisions)
    if errors:
        return [("recording error: the bag and decisions.jsonl hold what the live run saw",
                 "; ".join(errors))]
    if takeoff_s is None:
        return [("replay: PX4 reported TAKEOFF (the start of the comparison)",
                 "no TAKEOFF in the mode history")]
    compared = replay_check.compared_captures(replay_check.capture_stamps_us(live_dir / "bag"),
                                              round(takeoff_s * 1e6))
    record_property("replay_from_takeoff_s", round(takeoff_s, 3))
    record_property("replay_compared_captures", len(compared))
    record_property("replay_ground_captures", len(live_decisions) - len(compared))
    seed_offset = int(os.environ.get("SURVEY_SEED_OFFSET", "0"))
    params = node_params(scenario, cfg, seed_offset, "replay",
                         finalize_timeout_s=timeouts.finalize)
    instance = Ros2Instance(cfg, test_id, name_suffix="replay",
                            launch="survey_demo replay.launch.py",
                            out_dir=ARTIFACTS / "ros" / test_id / "replay", params=params,
                            in_dir=live_dir, launch_args={"bag": "/in/bag"})
    try:
        instance.start()
        await rosbridge.reconnect(ARTIFACTS / "ros" / test_id / "recordings" / "replay")
        rosbridge.subscribe(STATUS, STATUS_TYPE)
        await instance.wait_for_log_line("SURVEY REPLAY DONE", timeouts.replay)
        await wait_for_topic(rosbridge, STATUS,
                             lambda m: m["has_capture"] and m["last_capture_id"] == last
                             and m["queue_empty"], timeouts.sensor_drain,
                             f"replay {STATUS} last_capture_id {last} and queue_empty")
        finalized = await rosbridge.call_service(
            "/markers/finalize", "sensor_bridge_msgs/srv/Finalize",
            {"line_id": f"{scenario.name}_replay"}, timeouts.finalize)
        if not finalized["success"]:
            return [("replay finalize succeeds", finalized["message"])]
    finally:
        instance.stop()
    replay_decisions = replay_check.read_decisions(instance.out_dir / "decisions.jsonl")
    replay_markers = replay_check.read_markers(instance.out_dir / "markers.json")
    failures = []
    mismatches = replay_check.decision_mismatches(live_decisions, replay_decisions,
                                                  compared=compared)
    if mismatches:
        failures.append(((f"replay: the same usability decision for all {len(compared)} "
                          f"live captures from takeoff (t >= {takeoff_s:.3f} s)"),
                         "; ".join(mismatches)))
    mismatches = replay_check.marker_mismatches(live_markers, replay_markers,
                                                rule.replay_s_m)
    if mismatches:
        failures.append((f"replay: the same markers (s within {rule.replay_s_m} m)",
                         "; ".join(mismatches)))
    return failures


@pytest.mark.parametrize("scenario", _params(), indirect=True)
async def test_line_survey(cfg, scenario, drone, recorder, ros2, rosbridge, test_id,
                           record_property):
    timeouts = cfg.timeouts_wall_s
    survey = scenario.survey
    line = survey.line
    rule = cfg.survey.pass_rule
    h_v = scenario.altitude_m  # commanded vehicle height (sensor height + offset z)
    ds = ds_m(cfg, scenario.speed_m_s)
    record_property("ds_m", round(ds, 4))
    record_property("capture_rate_hz", cfg.survey.sensor.capture_rate_hz)

    # 1. Status topics, the rosapi check, the first PX4 pose at the georeferencing node.
    rosbridge.subscribe(STATUS, STATUS_TYPE)
    rosbridge.subscribe(TRUTH, TRUTH_TYPE)
    rosbridge.subscribe(GEOREF, GEOREF_TYPE)
    await assert_no_sim_subscriptions(rosbridge, cfg, "before the flight")
    await wait_for_topic(rosbridge, STATUS, lambda m: m["pose_received"],
                         timeouts.first_dds_position, f"{STATUS} pose_received")

    # 2. Take off to h_v.
    home = await wait_for_stream(drone.telemetry.home(), lambda h: True, timeouts.health,
                                 "home position")
    plan, _ = to_mission_plan(line_waypoints(line, rule.run_m), home.latitude_deg,
                              home.longitude_deg, h_v, scenario.speed_m_s,
                              rule.acceptance_radius_m)
    await call(drone.action.set_takeoff_altitude(h_v), cfg, "set takeoff altitude")
    await call(drone.mission.set_return_to_launch_after_mission(False), cfg,
               "no RTL after mission")
    await call(drone.mission.upload_mission(plan), cfg, "upload mission")
    await wait_armable(drone, cfg)
    await call(drone.action.arm(), cfg, "arm")
    await call(drone.action.takeoff(), cfg, "takeoff")
    await wait_altitude(recorder, h_v, rule.flight_height_m, timeouts.takeoff, cfg)

    # 3. The line as a mission (run-in, start, end, run-out); the sensor lag is read
    # while it flies.
    await call(drone.mission.start_mission(), cfg, "start mission")
    lags = []

    async def finished():
        lag = sensor_lag_s(rosbridge.latest(STATUS), rosbridge.latest(TRUTH))
        if lag is not None:
            lags.append(lag)
        return await call(drone.mission.is_mission_finished(), cfg, "is mission finished")

    await wait_until(finished, timeouts.mission, "mission finished", recorder,
                     cfg.poll_interval_s)
    lag_max = max(lags, default=math.nan)
    record_property("sensor_lag_max_s", round(lag_max, 3))
    # Reported with the pass rules below (one assertion with every failure).
    lag_failure = None if lag_max <= rule.sensor_lag_max_s else (
        (f"the sensor within {rule.sensor_lag_max_s} s (simulated) of the newest PX4 pose "
         "during the mission"),
        f"{lag_max:.2f} s behind at most ({len(lags)} readings)")
    lat, lon = home.latitude_deg, home.longitude_deg
    t_in, t_out = line_window(recorder, line, lat, lon)
    height_err = assert_altitude(recorder, h_v, math.inf, t_in, t_out)
    cross_max = assert_cross_track(recorder, line, lat, lon, math.inf)
    speed = line_speed(recorder, line, lat, lon)
    record_property("flight_height_err_m", round(height_err, 3))
    record_property("cross_track_max_m", round(cross_max, 3))
    record_property("line_speed_m_s", round(speed, 3))
    record_property("effective_speed_factor", round(recorder.effective_speed_factor(), 2))

    # 4. Land, stop the sensor, drain.
    await call(drone.action.land(), cfg, "land")
    await wait_landed_and_disarmed(recorder, timeouts.land, cfg)
    stopped = await rosbridge.call_service("/survey/stop", "sensor_bridge_msgs/srv/Stop", {},
                                           timeouts.mavsdk_call)
    if not stopped["has_capture"]:
        raise FlightAssertionError("the sensor emitted at least one capture", "none")
    last = int(stopped["last_capture_id"])
    await wait_for_topic(rosbridge, STATUS,
                         lambda m: m["has_capture"] and m["last_capture_id"] == last
                         and m["queue_empty"], timeouts.sensor_drain,
                         f"{STATUS} last_capture_id {last} and queue_empty")
    await wait_for_topic(rosbridge, TRUTH, lambda m: m["capture_id"] == last,
                         timeouts.sensor_drain, f"{TRUTH} capture_id {last}")
    await wait_for_topic(rosbridge, GEOREF, lambda m: m["raw"]["capture_id"] == last,
                         timeouts.sensor_drain, f"{GEOREF} capture_id {last}")

    # 5. Final figures; Finalize; the rosapi check again.
    status = rosbridge.latest(STATUS)
    finalized = await rosbridge.call_service(
        "/markers/finalize", "sensor_bridge_msgs/srv/Finalize", {"line_id": scenario.name},
        timeouts.finalize)
    if not finalized["success"] or int(finalized["last_capture_id"]) != last:
        raise FlightAssertionError(f"finalize succeeds with last_capture_id {last}",
                                   f"success {finalized['success']}, last_capture_id "
                                   f"{finalized['last_capture_id']}: {finalized['message']}")
    await assert_no_sim_subscriptions(rosbridge, cfg, "after finalize")
    counts = reason_counts(status)
    truth_msgs, georef_msgs = recorded(rosbridge, TRUTH), recorded(rosbridge, GEOREF)
    stream_failures = capture_stream_failures(truth_msgs, georef_msgs, status, last)
    window_start = capture_window_start(truth_msgs, georef_msgs)
    record_property("capture_ids_published", last + 1)
    record_property("capture_window_first_id", -1 if window_start is None else window_start)
    record_property("capture_stream_problems", len(stream_failures))
    figures = truth_figures(truth_msgs, georef_msgs, line.length_m, ds,
                            cfg.survey.georeference.coverage_gap_factor)
    record_property("coverage", round(figures.coverage, 4))
    record_property("coverage_georef", round(status["coverage"], 4))
    record_property("max_gap_m", round(figures.max_gap_m, 4))
    record_property("usable_without_truth", figures.usable_without_truth)
    record_property("samples_without_pose", figures.samples_without_pose)
    record_property("clock_shift_mean_m", round(figures.clock_shift_mean_m, 4))
    record_property("captures_seen", status["captures_seen"])
    record_property("usable_captures", status["usable"])
    record_property("no_pose", counts["no_pose"])
    record_property("reasons", ", ".join(f"{r}={n}" for r, n in counts.items()))
    record_property("captures_used", finalized["captures_used"])

    # The detector's output (the runner reads its file without ROS).
    markers = replay_check.read_markers(ros2.out_dir / "markers.json")
    verdict = evaluate_markers([m.s_m for m in survey.markers], [m["s_m"] for m in markers],
                               rule.position_tol_m)
    record_property("markers_found", len(markers))
    record_property("false_markers", len(verdict.false_markers))
    record_property("marker_errors_m", json.dumps(
        [None if e is None else round(e, 3) for e in verdict.errors_m]))

    # Clock offset: the georeferenced capture positions against the PX4 estimate at the
    # samples' true times; expected v x offset with v the measured line speed.
    offset_s = survey.clock.offset_ms / 1000.0
    expected_shift = speed * offset_s
    if offset_s != 0.0:
        record_property("clock_shift_expected_m", round(expected_shift, 4))

    # 6. Pass rules (one assertion with every failure).
    failures = [lag_failure] if lag_failure else []
    # Rule 0: no capture lost, repeated or reordered between the sensor, the
    # georeferencing node and the recordings the figures below are computed from.
    failures.extend(stream_failures)

    def check(ok: bool, expected: str, actual: str) -> None:
        if not ok:
            failures.append((expected, actual))

    # Rule 1: coverage from the sensor's own positions, and the georeferencing node agrees.
    gap_m = cfg.survey.georeference.coverage_gap_factor * ds
    check(figures.coverage >= rule.coverage_min,
          f"coverage >= {rule.coverage_min} (gaps <= {gap_m:.3f} m)",
          f"{figures.coverage:.4f}, max gap {figures.max_gap_m:.3f} m")
    check(abs(figures.coverage - status["coverage"]) <= rule.coverage_agreement,
          f"georeferenced coverage within {rule.coverage_agreement} of {figures.coverage:.4f}",
          f"{status['coverage']:.4f}")
    check(figures.usable_without_truth == 0,
          "no usable capture the sensor built without a pose (usable_without_truth 0)",
          f"usable_without_truth {figures.usable_without_truth} "
          f"({figures.samples_without_pose} samples without a pose)")
    # Rule 2: flight over the line, from MAVSDK telemetry.
    check(height_err <= rule.flight_height_m,
          f"height within {h_v} ± {rule.flight_height_m} m over the line",
          f"max error {height_err:.3f} m")
    check(cross_max <= rule.cross_track_m,
          f"cross-track within ± {rule.cross_track_m} m over the line", f"max {cross_max:.3f} m")
    check(abs(speed - scenario.speed_m_s) <= rule.speed_tolerance * scenario.speed_m_s,
          f"line speed {scenario.speed_m_s} m/s ± {rule.speed_tolerance:.0%}",
          f"{speed:.3f} m/s")
    # Rule 3: every painted marker found within the tolerance, no marker elsewhere.
    failures.extend(verdict.failures)
    # Rule 4 (clock-offset scenarios): mean shift = line speed x offset in the
    # georeferenced positions. (The markers found must still be within the tolerance of
    # rule 3; their individual errors, up to a few cm, carry the cross-track wander.)
    if offset_s != 0.0:
        check(abs(figures.clock_shift_mean_m - expected_shift) <= rule.clock_shift_m,
              f"clock shift {survey.clock.offset_ms:+.0f} ms: mean georeferenced - true "
              f"position {expected_shift:+.4f} m ({speed:.3f} m/s x {offset_s:+.3f} s) "
              f"± {rule.clock_shift_m} m", f"{figures.clock_shift_mean_m:+.4f} m")
    # Rule 5 (replay_check scenarios): replay the recording through a fresh
    # georeferencing node and detector.
    if survey.replay_check:
        failures.extend(await replay(cfg, scenario, test_id, ros2, rosbridge,
                                     rosbridge.latest(STATUS), markers, last,
                                     takeoff_sim_s(recorder), record_property))
    if failures:
        raise FlightAssertionError("; ".join(e for e, _ in failures),
                                   "; ".join(a for _, a in failures), recorder)
