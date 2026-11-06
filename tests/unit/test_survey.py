"""Unit tests for framework.survey: node parameters, the sensor-lag guard, the truth
figures and the marker verdict (no simulator, no ROS)."""

import math

import pytest

from framework.config import load_config
from framework.mission import list_scenarios, load_scenario
from framework.survey import (
    FINALIZE_MARGIN_S,
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
