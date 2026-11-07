"""Unit tests for framework.survey: node parameters, the sensor-lag guard, the truth
figures and the marker verdict (no simulator, no ROS)."""

import math

import numpy as np
import pytest
from sensor_bridge.coverage import coverage
from sensor_bridge.line import Line

from framework.config import load_config
from framework.mission import SurveyLine, list_scenarios, load_scenario
from framework.survey import (
    FINALIZE_MARGIN_S,
    capture_stream_failures,
    capture_window_start,
    evaluate_markers,
    node_params,
    reason_counts,
    sensor_lag_s,
    truth_figures,
)

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def scenario(name):
    return load_scenario(next(p for p in list_scenarios("line_survey") if p.stem == name))


# node_params


def test_live_params_cover_every_node_and_the_scenario(cfg):
    sc = scenario("line_survey_markers")
    params = node_params(sc, cfg, seed_offset=2, mode="live", finalize_timeout_s=60)
    assert set(params) == {"georeference_node", "toy_sensor_node", "marker_detector_node",
                           "ready_gate"}
    toy = params["toy_sensor_node"]["ros__parameters"]
    assert toy["markers_s_m"] == [2.5, 6.0, 9.5] and toy["markers_d_m"] == [0.0, 0.15, 0.0]
    assert toy["seed"] == 2 and toy["wait_for_ready"] is True
    geo = params["georeference_node"]["ros__parameters"]
    assert geo["pose_type"] == "px4" and geo["republish_poses"] is True
    assert geo["sensor_offset_m"] == [0.15, 0.0, 0.0] and geo["sensor_height_m"] == 0.4
    assert geo["ds_m"] == pytest.approx(0.05)
    det = params["marker_detector_node"]["ros__parameters"]
    assert det["finalize_wait_s"] == 60 - FINALIZE_MARGIN_S


def test_replay_params_use_the_recorded_poses_and_no_sensor(cfg):
    params = node_params(scenario("line_survey_markers"), cfg, 0, "replay",
                         finalize_timeout_s=60)
    assert set(params) == {"georeference_node", "marker_detector_node", "ready_gate"}
    geo = params["georeference_node"]["ros__parameters"]
    assert geo["pose_type"] == "sample" and geo["republish_poses"] is False
    assert geo["pose_topic"] == "/survey/pose_used"
    gate = params["ready_gate"]["ros__parameters"]
    assert gate["exit_when_ready"] is True and "flag_topics" not in gate


def test_a_line_without_markers_leaves_the_marker_lists_out(cfg):
    """A ROS 2 Humble parameter file cannot hold an empty list."""
    params = node_params(scenario("line_survey_no_markers"), cfg, 0, "live",
                         finalize_timeout_s=60)
    toy = params["toy_sensor_node"]["ros__parameters"]
    assert not any(key.startswith("markers_") for key in toy)


def test_the_clock_comes_from_the_scenario(cfg):
    toy = node_params(scenario("line_survey_clock_plus"), cfg, 0, "live",
                      finalize_timeout_s=60)["toy_sensor_node"]["ros__parameters"]
    assert (toy["clock_offset_ms"], toy["clock_jitter_ms"]) == (20.0, 1.0)


def test_unknown_mode_is_rejected(cfg):
    with pytest.raises(ValueError, match="mode"):
        node_params(scenario("line_survey_markers"), cfg, 0, "both", finalize_timeout_s=60)


def test_the_scenario_matrix():
    sc = {p.stem: load_scenario(p) for p in list_scenarios("line_survey")}
    assert sorted(sc) == ["line_survey_clock_minus", "line_survey_clock_plus",
                          "line_survey_markers", "line_survey_no_markers"]
    assert [n for n, s in sc.items() if s.survey.replay_check] == ["line_survey_markers"]
    assert sorted(n for n, s in sc.items() if s.survey.schedule == "weekly") == [
        "line_survey_clock_minus", "line_survey_no_markers"]
    for s in sc.values():
        # PX4's DDS stamps follow its simulated clock only with synchronization off.
        assert s.px4_params["UXRCE_DDS_SYNCT"] == 0
        # The low-altitude profile is applied, including the simulation-only IMU setting.
        assert s.px4_params["EKF2_IMU_CTRL"] == 0 and s.px4_params["EKF2_HGT_REF"] == 0
        # A lever arm, so a georeferencing that ignores it is caught.
        assert s.survey.sensor_offset_m != (0.0, 0.0, 0.0)


# The sensor-lag guard


def test_sensor_lag_is_the_newest_pose_minus_the_newest_true_sample():
    status = {"now_us": 12_000_000}
    truth = {"sample_true_us": [9_000_000, 9_010_000, 9_020_000, 9_030_000]}
    assert sensor_lag_s(status, truth) == pytest.approx(2.97)


@pytest.mark.parametrize("status, truth", [
    (None, {"sample_true_us": [1]}), ({"now_us": 1}, None), ({"now_us": 1}, {"sample_true_us": []}),
])
def test_sensor_lag_is_unknown_without_both_readings(status, truth):
    assert sensor_lag_s(status, truth) is None


def test_reason_counts():
    assert reason_counts({"reasons": ["ok", "no_pose"], "counts": [5, 2]}) == {"ok": 5,
                                                                              "no_pose": 2}


# Truth figures


def truth_msg(i, s0, n=4, ds=0.0025):
    return {"capture_id": i, "sample_s": [s0 + k * ds for k in range(n)]}


def georef_msg(i, s0, usable=True, shift=0.0, n=4, ds=0.0025):
    return {"raw": {"capture_id": i}, "usable": usable,
            "sample_s": [s0 + shift + k * ds for k in range(n)]}


def line_of_captures(count=240, step=0.05, **kw):
    truth = [truth_msg(i, i * step) for i in range(count)]
    georef = [georef_msg(i, i * step, **kw) for i in range(count)]
    return truth, georef


def test_truth_figures_full_coverage_and_zero_shift():
    truth, georef = line_of_captures()
    f = truth_figures(truth, georef, 12.0, 0.05, 1.5)
    assert (f.pairs, f.usable_pairs) == (240, 240)
    assert f.coverage == pytest.approx(1.0) and f.max_gap_m == pytest.approx(0.05, abs=1e-3)
    assert f.clock_shift_mean_m == pytest.approx(0.0, abs=1e-9)
    assert f.usable_without_truth == 0


def test_truth_figures_clock_shift_is_the_mean_position_difference():
    truth, georef = line_of_captures(shift=0.02)
    assert truth_figures(truth, georef, 12.0, 0.05, 1.5).clock_shift_mean_m == \
        pytest.approx(0.02)


def test_truth_figures_coverage_drops_with_unusable_captures():
    truth, georef = line_of_captures()
    for g in georef[60:100]:                     # 40 captures = 2 m without a usable one
        g["usable"] = False
    f = truth_figures(truth, georef, 12.0, 0.05, 1.5)
    assert f.coverage == pytest.approx(1 - 2.05 / 12.0, abs=0.01) and f.coverage < 0.95
    assert f.max_gap_m == pytest.approx(2.05, abs=0.01)


def test_truth_figures_recorded_nan_is_null_and_flags_a_usable_capture_without_truth():
    truth, georef = line_of_captures(count=10)
    truth[3]["sample_s"][1] = None               # a NaN on the topic is a null in the recording
    f = truth_figures(truth, georef, 12.0, 0.05, 1.5)
    assert f.usable_without_truth == 1 and f.samples_without_pose == 1
    assert not math.isnan(f.clock_shift_mean_m)


def test_truth_figures_pair_by_capture_id():
    truth, georef = line_of_captures(count=10)
    f = truth_figures(truth[:6], georef[3:], 12.0, 0.05, 1.5)
    assert f.pairs == 3


# Capture stream integrity


def counted(last, **kw):
    """The georeferencing node's status after ``last + 1`` captures."""
    return {"captures_seen": last + 1, "released": last + 1, **kw}


def test_complete_streams_have_no_failures():
    truth, georef = line_of_captures(count=10)
    assert capture_stream_failures(truth, georef, counted(9), last=9) == []


def test_recordings_that_start_after_the_sensor_are_complete_from_their_first_capture():
    truth, georef = line_of_captures(count=10)
    # The test subscribes after the sensor started: the truth stream begins at 2, the
    # georeferenced one at 3; the node itself counted every capture from 0.
    assert capture_window_start(truth[2:], georef[3:]) == 3
    assert capture_stream_failures(truth[2:], georef[3:], counted(9), last=9) == []


def test_a_missing_capture_in_the_middle_is_named_in_the_stream_that_lost_it():
    truth, georef = line_of_captures(count=10)
    del truth[4]
    (expected, actual), = capture_stream_failures(truth, georef, counted(9), last=9)
    assert "truth" in expected and "0..9" in expected
    assert actual == "missing ids 4"
    # truth_figures alone pairs the common ids and raises nothing.
    assert truth_figures(truth, georef, 12.0, 0.05, 1.5).pairs == 9


def test_a_missing_georeferenced_capture_is_named():
    truth, georef = line_of_captures(count=10)
    del georef[6], georef[3]
    (expected, actual), = capture_stream_failures(truth, georef, counted(9), last=9)
    assert "georeferenced" in expected and actual == "missing ids 3, 6"


def test_a_repeated_capture_is_named():
    truth, georef = line_of_captures(count=10)
    georef.insert(5, dict(georef[5]))
    (_, actual), = capture_stream_failures(truth, georef, counted(9), last=9)
    assert actual == "repeated ids 5"
    # truth_figures keeps the last copy and says nothing.
    assert truth_figures(truth, georef, 12.0, 0.05, 1.5).pairs == 10


def test_swapped_captures_are_reported_as_out_of_order():
    truth, georef = line_of_captures(count=10)
    truth[2], truth[3] = truth[3], truth[2]
    (expected, actual), = capture_stream_failures(truth, georef, counted(9), last=9)
    assert "truth" in expected and actual == "out of order (id 2 arrived after 3)"


def test_the_last_capture_missing_from_a_stream_is_a_missing_id():
    truth, georef = line_of_captures(count=10)
    (_, actual), = capture_stream_failures(truth[:9], georef, counted(9), last=9)
    assert actual == "missing ids 9"


def test_a_capture_after_the_last_published_one_is_reported():
    truth, georef = line_of_captures(count=11)
    (_, actual), = capture_stream_failures(truth, georef[:10], counted(9), last=9)
    assert actual == "ids after the last published one (9): 10"


def test_captures_lost_before_the_node_or_before_its_release_are_counted_against_the_sensor():
    truth, georef = line_of_captures(count=10)
    failures = capture_stream_failures(truth, georef, counted(9, captures_seen=9, released=8),
                                       last=9)
    assert [a for _, a in failures] == ["9", "8"]
    assert "raw captures counted" in failures[0][0] and "released" in failures[1][0]


def test_an_empty_stream_is_reported_and_does_not_hide_the_other_one():
    truth, _ = line_of_captures(count=10)
    failures = capture_stream_failures(truth, [], counted(9), last=9)
    assert capture_window_start(truth, []) is None
    assert failures == [("the georeferenced recording holds the captures of the flight",
                         "no message recorded")]
    del truth[4]
    assert capture_stream_failures(truth, [], counted(9), last=9)[0][1] == "missing ids 4"


def test_a_long_list_of_missing_ids_is_cut_short():
    truth, georef = line_of_captures(count=30)
    georef = georef[:5] + georef[17:]
    (_, actual), = capture_stream_failures(truth, georef, counted(29), last=29)
    assert actual == "missing ids 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, ... (12 in all)"


# Independent references (docs/oracle-independence.md)


def coverage_on_a_grid(positions, usable, length_m, ds_m, gap_factor, cells=40_000):
    """Coverage by another algorithm: walk a fine grid along the line and call a cell
    covered when the gap between the usable captures around it is within the limit."""
    points = np.array(sorted(float(s) for s, ok in zip(positions, usable, strict=True)
                             if ok and math.isfinite(s) and 0.0 <= s <= length_m))
    points = np.concatenate(([0.0], points, [length_m]))
    centres = (np.arange(cells) + 0.5) * (length_m / cells)
    right = np.searchsorted(points, centres)
    gaps = points[right] - points[right - 1]
    return float(np.mean(gaps <= gap_factor * ds_m)), float(np.max(np.diff(points)))


@pytest.mark.parametrize("seed", range(12))
def test_coverage_agrees_with_a_grid_reference_on_random_captures(seed):
    rng = np.random.default_rng(seed)
    length, ds = 12.0, 0.05
    positions = np.arange(0.0, length + 1e-9, ds / 2)
    positions = positions + rng.normal(0.0, 0.004, positions.size)
    positions[rng.random(positions.size) < rng.uniform(0.0, 0.5)] = math.nan
    for _ in range(rng.integers(0, 4)):             # holes of 1-40 captures
        a = rng.integers(0, positions.size - 40)
        positions[a:a + rng.integers(1, 40)] = math.nan
    usable = rng.random(positions.size) < 0.9
    ours = coverage(positions, usable, length, ds, 1.5)
    fraction, max_gap = coverage_on_a_grid(positions, usable, length, ds, 1.5)
    assert ours.fraction == pytest.approx(fraction, abs=2e-3)
    assert ours.max_gap_m == pytest.approx(max_gap, abs=1e-9)


@pytest.mark.parametrize("seed", range(5))
def test_the_two_line_implementations_project_alike(seed):
    """The ROS chain projects with sensor_bridge.line.Line, the test's own telemetry
    checks with mission.SurveyLine: two implementations, so a mistake in one does not
    cancel out between the sensor and the oracle."""
    rng = np.random.default_rng(seed)
    start_n, start_e = rng.uniform(-30, 30, 2)
    heading, length = rng.uniform(0, 360), rng.uniform(5, 40)
    ros_line = Line(start_n, start_e, heading, length)
    test_line = SurveyLine(start_n, start_e, heading, length)
    for north, east in rng.uniform(-60, 60, (50, 2)):
        assert ros_line.project(north, east) == pytest.approx(test_line.project(north, east),
                                                              abs=1e-9)
    s, d = rng.uniform(0, length), rng.uniform(-2, 2)
    assert test_line.project(*test_line.point(s, d)) == pytest.approx((s, d), abs=1e-9)


# The marker verdict


def test_markers_found_in_place():
    v = evaluate_markers([2.5, 6.0], [6.02, 2.49], 0.1)
    assert v.failures == [] and v.false_markers == []
    assert v.errors_m == pytest.approx([-0.01, 0.02]) and v.mean_error_m == pytest.approx(0.005)


def test_a_missed_marker_and_a_false_marker_are_reported():
    v = evaluate_markers([2.5, 6.0], [2.5, 8.0], 0.1)
    assert v.errors_m[1] is None and v.false_markers == [8.0]
    assert [f[0] for f in v.failures] == ["a marker found within 0.1 m of s = 6.00 m",
                                          "no marker where none is painted"]


def test_a_marker_further_than_the_tolerance_is_both_missed_and_false():
    v = evaluate_markers([2.5], [2.65], 0.1)
    assert v.errors_m == [None] and v.false_markers == [2.65] and len(v.failures) == 2


def test_nothing_painted_nothing_found():
    assert evaluate_markers([], [], 0.1).failures == []
    assert evaluate_markers([], [4.0], 0.1).false_markers == [4.0]


def test_one_found_marker_cannot_satisfy_two_expected_ones():
    v = evaluate_markers([2.5, 2.55], [2.52], 0.1)
    assert v.errors_m.count(None) == 1
