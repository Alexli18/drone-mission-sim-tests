"""Unit tests for the pure sensor_bridge modules (no ROS, no px4_msgs)."""

import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_bridge.coverage import coverage
from sensor_bridge.georeference import (
    REASONS,
    DecisionLog,
    GeorefParams,
    HoldQueue,
    StatusCounters,
    decision_line,
    georeference,
    replay_params,
)
from sensor_bridge.ground import GroundReference
from sensor_bridge.line import Line, sensor_position
from sensor_bridge.pose import Pose, PoseHistory, pose_from_sample
from sensor_bridge.px4_pose import home_z_from_px4, pose_from_px4

pytestmark = pytest.mark.unit

REPO = Path(__file__).resolve().parents[2]
SENSOR_BRIDGE = REPO / "ros2_ws" / "src" / "sensor_bridge"
LINE = Line(start_north_m=5.0, start_east_m=0.0, heading_deg=0.0, length_m=20.0)
PARAMS = GeorefParams(sensor_height_m=0.4, sensor_offset_m=(0.0, 0.0, 0.0), off_line_m=0.5,
                      height_band_m=0.15, hold_timeout_s=0.5, wall_fallback_s=2.0)
POSE_DT_US = 20_000          # 50 Hz
SPEED = 1.0                  # m/s along north


def flight_pose(t_us, d=0.0, h=0.4):
    """The vehicle on LINE (heading north) at 1 m/s, s = 0 at t = 1 s."""
    return Pose(t_us, 5.0 + SPEED * (t_us - 1_000_000) / 1e6, d, -h, 0.0)


def history_of(times_us, max_gap_s=0.2, **kw):
    hist = PoseHistory(max_gap_s=max_gap_s)
    for t in times_us:
        hist.add(flight_pose(t, **kw))
    return hist


def raw(capture_id, stamps, saturated=False):
    return SimpleNamespace(capture_id=capture_id, sample_stamps_us=np.array(stamps, dtype=np.uint64),
                           saturated=saturated)


# Projection, offset, interpolation


@pytest.mark.parametrize("heading_deg, north, east, s, d", [
    (0.0, 3.0, 1.0, 3.0, 1.0),
    (90.0, 1.0, 3.0, 3.0, -1.0),
    (225.0, -math.sqrt(2), -math.sqrt(2), 2.0, 0.0),
    (225.0, -math.sqrt(2), 0.0, 1.0, -1.0),    # right of south-west is north-west
])
def test_projection_for_headings(heading_deg, north, east, s, d):
    line = Line(0.0, 0.0, heading_deg, 10.0)
    assert line.project(north, east) == pytest.approx((s, d), abs=1e-12)


@pytest.mark.parametrize("heading_deg, north, east", [
    (0.0, 0.3, 0.1), (90.0, -0.1, 0.3), (225.0, -0.3 * math.sqrt(0.5) + 0.1 * math.sqrt(0.5),
                                         -0.3 * math.sqrt(0.5) - 0.1 * math.sqrt(0.5)),
])
def test_sensor_offset_rotates_with_heading(heading_deg, north, east):
    pose = Pose(0, 0.0, 0.0, -1.0, math.radians(heading_deg))
    n, e, h = sensor_position(pose, (0.3, 0.1, 0.2))
    assert (n, e) == pytest.approx((north, east), abs=1e-12)
    assert h == pytest.approx(0.8)
    assert sensor_position(pose, SimpleNamespace(x=0.3, y=0.1, z=0.2)) == (n, e, h)


def test_sensor_height_from_the_ground_reference():
    pose = Pose(0, 0.0, 0.0, -0.443, 0.0)
    assert sensor_position(pose, (0.0, 0.0, 0.0))[2] == pytest.approx(0.443)
    assert sensor_position(pose, (0.0, 0.0, 0.0), ground_z=-0.043)[2] == pytest.approx(0.4)
    assert sensor_position(pose, (0.0, 0.0, 0.1), ground_z=-0.043)[2] == pytest.approx(0.3)


# Ground reference


def test_ground_reference_follows_the_valid_home_position():
    ground = GroundReference()
    assert ground.value == 0.0 and not ground.is_set
    assert not ground.update(0.5, valid=False)                 # no local home yet
    assert ground.update(0.042, valid=True) and ground.z0 == pytest.approx(0.042)
    assert not ground.update(0.042, valid=True)                # unchanged
    assert ground.update(0.035, valid=True)                    # reset at arming
    assert ground.value == pytest.approx(0.035) and ground.updates == 2
    assert not ground.update(1.0, valid=False)
    assert ground.z0 == pytest.approx(0.035)


def test_ground_reference_fixed_from_a_recording():
    ground = GroundReference()
    assert ground.fix(-0.043) and ground.is_set and ground.z0 == -0.043
    assert not ground.fix(-0.043)


def test_home_adapter():
    msg = SimpleNamespace(z=0.04, valid_lpos=True)
    assert home_z_from_px4(msg) == (pytest.approx(0.04), True)
    msg.valid_lpos = False
    assert home_z_from_px4(msg)[1] is False


def test_interpolation_between_poses_and_heading_along_the_shorter_arc():
    hist = PoseHistory(max_gap_s=0.2)
    hist.add(Pose(1_000_000, 0.0, 0.0, -1.0, math.radians(170)))
    hist.add(Pose(1_100_000, 1.0, 2.0, -2.0, math.radians(-170)))
    p = hist.interpolate(1_025_000)
    assert (p.x, p.y, p.z) == pytest.approx((0.25, 0.5, -1.25))
    assert math.degrees(p.heading_rad) == pytest.approx(175.0)
    mid = hist.interpolate(1_050_000)
    assert abs(math.degrees(mid.heading_rad)) == pytest.approx(180.0)
    assert hist.interpolate(1_000_000).x == 0.0
    assert hist.interpolate(999_999) is None
    assert hist.interpolate(1_100_001) is None
    assert hist.covers(1_100_000) and not hist.covers(1_100_001)


def test_max_pose_gap_and_dropped_old_poses():
    hist = history_of([1_000_000, 1_200_000, 1_400_001])
    assert hist.bracketing(1_100_000) is not None          # exactly 0.2 s apart
    assert hist.bracketing(1_300_000) is None              # 0.200001 s apart
    assert not hist.add(flight_pose(1_200_000))
    assert not hist.add(flight_pose(1_400_001))
    assert hist.dropped == 2 and len(hist) == 3


def test_history_window_holds_every_bracketing_pose():
    hist = history_of([1_000_000, 1_020_000, 1_040_000, 1_060_000])
    times = lambda poses: [p.t_us for p in poses]
    assert times(hist.window(1_020_000, 1_040_000)) == [1_020_000, 1_040_000]
    assert times(hist.window(1_025_000, 1_035_000)) == [1_020_000, 1_040_000]
    assert times(hist.window(900_000, 1_001_000)) == [1_000_000, 1_020_000]
    assert times(hist.window(1_050_000, 2_000_000)) == [1_040_000, 1_060_000]
    assert history_of([]).window(0, 1) == []


def test_history_keeps_ten_seconds():
    hist = history_of(range(0, 12_000_001, 1_000_000), max_gap_s=2.0)
    assert hist.bracketing(1_500_000) is None
    assert hist.bracketing(2_500_000) is not None


def test_pose_adapters():
    msg = SimpleNamespace(timestamp_sample=123, x=1.0, y=2.0, z=-0.4, heading=0.5,
                          xy_valid=True, z_valid=True)
    assert pose_from_px4(msg) == Pose(123, 1.0, 2.0, -0.4, 0.5)
    msg.z_valid = False
    assert pose_from_px4(msg) is None
    msg.z_valid, msg.xy_valid = True, False
    assert pose_from_px4(msg) is None
    sample = SimpleNamespace(t_us=5, x_m=1.0, y_m=2.0, z_m=-0.4, heading_rad=0.1)
    assert pose_from_sample(sample) == Pose(5, 1.0, 2.0, -0.4, 0.1)


# georeference


def test_per_sample_positions_of_a_moving_capture():
    hist = history_of(range(1_000_000, 4_000_001, POSE_DT_US))
    stamps = [2_000_000 + k * 1350 for k in range(37)]
    georef, used = georeference(raw(0, stamps), hist, LINE, PARAMS)
    expected = [(t - 1_000_000) / 1e6 for t in stamps]
    assert georef.sample_s == pytest.approx(expected, abs=1e-6)
    assert georef.sample_d == pytest.approx(np.zeros(37), abs=1e-6)
    assert georef.sample_h == pytest.approx(np.full(37, 0.4), abs=1e-6)
    assert georef.s == pytest.approx(np.mean(expected), abs=1e-6)
    assert georef.usable and georef.reason == "ok"
    assert [p.t_us for p in used] == [2_000_000 + k * POSE_DT_US for k in range(4)]


def test_offset_is_applied_per_sample():
    hist = history_of(range(1_000_000, 4_000_001, POSE_DT_US), h=0.5)
    params = GeorefParams(**{**PARAMS.__dict__, "sensor_offset_m": (0.2, -0.1, 0.1)})
    georef, _ = georeference(raw(0, [2_000_000]), hist, LINE, params)
    assert (georef.s, georef.d, georef.h) == pytest.approx((1.2, -0.1, 0.4), abs=1e-6)


@pytest.mark.parametrize("offset_ms", [20.0, -20.0])
@pytest.mark.parametrize("heading_deg", [0.0, 225.0])
def test_clock_offset_shifts_positions_by_v_times_offset(offset_ms, heading_deg):
    """Samples stamped ``offset`` after their true time are placed
    v·offset further along the line (+20 ms) or back along it (−20 ms), within 1 mm."""
    line = Line(start_north_m=5.0, start_east_m=0.0, heading_deg=heading_deg, length_m=20.0)
    psi = math.radians(heading_deg)
    hist = PoseHistory(max_gap_s=0.2)
    for t in range(1_000_000, 6_000_001, POSE_DT_US):
        s = SPEED * (t - 1_000_000) / 1e6
        hist.add(Pose(t, 5.0 + s * math.cos(psi), s * math.sin(psi), -0.4, psi))
    true_us = np.array([2_000_000 + k * 1350 + 17 for k in range(37)])
    stamps = true_us + round(offset_ms * 1000)
    georef, _ = georeference(raw(0, stamps), hist, line, PARAMS)
    true_s = SPEED * (true_us - 1_000_000) / 1e6
    shift = np.asarray(georef.sample_s) - true_s
    expected = SPEED * offset_ms / 1000.0
    assert shift == pytest.approx(np.full(37, expected), abs=1e-3)
    assert float(np.mean(shift)) == pytest.approx(expected, abs=1e-3)
    assert np.sign(np.mean(shift)) == np.sign(offset_ms)
    assert georef.sample_d == pytest.approx(np.zeros(37), abs=1e-6)
    assert georef.usable and georef.reason == "ok"


@pytest.mark.parametrize("stamps, pose_kw, saturated, reason", [
    ([1_500_000, 9_000_000], {}, False, "no_pose"),
    ([500_000], {}, False, "no_pose"),
    ([200_000], {}, False, "outside_line"),         # s = −0.8 m (poses from t = 0)
    ([30_000_000], {}, False, "no_pose"),
    ([2_000_000], {"d": 0.6}, False, "off_line"),
    ([2_000_000], {"h": 0.6}, False, "off_height"),
    ([2_000_000], {}, True, "saturated"),
    ([2_000_000], {"d": 0.6, "h": 0.6}, True, "off_line"),
    ([2_000_000], {"h": 0.6}, True, "off_height"),
])
def test_each_reason_and_precedence(stamps, pose_kw, saturated, reason):
    start = 0 if reason == "outside_line" else 1_000_000
    hist = history_of(range(start, 4_000_001, POSE_DT_US), **pose_kw)
    georef, _ = georeference(raw(0, stamps, saturated), hist, LINE, PARAMS)
    assert georef.reason == reason
    assert georef.usable is False


def test_outside_line_beyond_the_end_and_no_pose_wins_over_it():
    hist = history_of(range(1_000_000, 30_000_001, POSE_DT_US))
    georef, _ = georeference(raw(0, [21_500_000]), hist, LINE, PARAMS)
    assert georef.reason == "outside_line" and georef.s == pytest.approx(20.5)
    georef, _ = georeference(raw(0, [21_500_000, 40_000_000]), hist, LINE, PARAMS)
    assert georef.reason == "no_pose"
    assert georef.s == pytest.approx(20.5)        # mean over the samples with a pose
    assert list(georef.sample_has_pose) == [True, False]
    assert math.isnan(georef.sample_s[1])


def test_no_sample_with_a_pose_gives_nan_position():
    georef, used = georeference(raw(0, [5, 6]), PoseHistory(max_gap_s=0.2), LINE, PARAMS)
    assert georef.reason == "no_pose" and math.isnan(georef.s) and used == []


# coverage


def test_coverage_virtual_end_points():
    cov = coverage([], [], 20.0, 0.25)
    assert cov.fraction == 0.0 and cov.max_gap_m == 20.0
    cov = coverage([10.0], [True], 20.0, 5.0, gap_factor=2.0)
    assert cov.fraction == 1.0 and cov.max_gap_m == 10.0


def test_coverage_gap_of_exactly_1_5_ds_counts_as_covered():
    positions = np.arange(0, 161) * 0.125            # 0 … 20 m
    keep = [s for s in positions if not 5.0 < s < 5.375]    # gap 5.0 → 5.375 = 1.5·Δs
    cov = coverage(keep, [True] * len(keep), 20.0, 0.25)
    assert cov.fraction == 1.0 and cov.max_gap_m == 0.375


def test_coverage_hole_lowers_the_fraction():
    positions = np.arange(0, 161) * 0.125
    keep = [s for s in positions if not 5.0 < s < 5.5]      # a 0.5 m hole
    cov = coverage(keep, [True] * len(keep), 20.0, 0.25)
    assert cov.fraction == pytest.approx(1.0 - 0.5 / 20.0)
    assert cov.max_gap_m == 0.5


def test_coverage_ignores_unusable_and_outside_and_nan():
    positions = [-1.0, 0.0, 10.0, 20.0, 21.0, float("nan")]
    cov = coverage(positions, [True, True, False, True, True, True], 20.0, 0.25)
    assert cov.max_gap_m == 20.0 and cov.fraction == 0.0


# HoldQueue


def test_hold_queue_releases_fifo_when_covered():
    hist = history_of(range(1_000_000, 2_000_001, POSE_DT_US))
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(0, [2_005_000, 2_010_000]), hist.newest.t_us, 0.0)
    queue.push(raw(1, [2_015_000]), hist.newest.t_us, 0.0)
    assert queue.release(hist, hist.newest.t_us, 0.1) == []
    hist.add(flight_pose(2_010_000))
    out = queue.release(hist, hist.newest.t_us, 0.1)
    assert [(r.georef.raw.capture_id, r.release) for r in out] == [(0, "covered")]
    hist.add(flight_pose(2_030_000))
    out = queue.release(hist, hist.newest.t_us, 0.1)
    assert [(r.georef.raw.capture_id, r.release) for r in out] == [(1, "covered")]
    assert out[0].georef.usable and len(queue) == 0


def test_hold_queue_head_blocks_the_rest():
    hist = history_of(range(1_000_000, 2_000_001, POSE_DT_US))
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(0, [3_000_000]), hist.newest.t_us, 0.0)
    queue.push(raw(1, [1_500_000]), hist.newest.t_us, 0.0)
    assert queue.release(hist, hist.newest.t_us, 0.1) == []
    assert len(queue) == 2


def test_hold_queue_timeout_in_simulated_time():
    hist = history_of(range(1_000_000, 2_000_001, POSE_DT_US))
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(0, [10_000_000]), 2_000_000, 0.0)   # stamp far ahead of PX4 time
    for t in range(2_020_000, 2_500_000, POSE_DT_US):
        hist.add(flight_pose(t))
        queue.pose_arrived(0.0)
        assert queue.release(hist, t, 0.0) == []
    hist.add(flight_pose(2_500_000))
    out = queue.release(hist, 2_500_000, 0.0)
    assert [(r.release, r.georef.reason) for r in out] == [("timeout", "no_pose")]


def test_hold_queue_wall_fallback():
    hist = history_of(range(1_000_000, 2_000_001, POSE_DT_US))
    queue = HoldQueue(LINE, PARAMS)
    queue.pose_arrived(100.0)
    queue.push(raw(0, [2_010_000]), 2_000_000, 100.5)
    assert queue.release(hist, 2_000_000, 101.9) == []
    out = queue.release(hist, 2_000_000, 102.0)
    assert [(r.release, r.georef.reason) for r in out] == [("wall", "no_pose")]


def test_hold_queue_wall_fallback_without_any_pose():
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(0, [10]), None, 50.0)
    hist = PoseHistory(max_gap_s=0.2)
    assert queue.release(hist, None, 51.0) == []
    assert [r.release for r in queue.release(hist, None, 52.0)] == ["wall"]


def test_flush_marks_uncovered_samples_no_pose():
    hist = history_of(range(1_000_000, 2_000_001, POSE_DT_US))
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(0, [1_990_000, 2_010_000]), 2_000_000, 0.0)
    queue.push(raw(1, [2_020_000]), 2_000_000, 0.0)
    out = queue.flush(hist)
    assert [(r.release, r.georef.reason) for r in out] == [("flush", "no_pose")] * 2
    assert list(out[0].georef.sample_has_pose) == [True, False]
    assert len(queue) == 0


def test_height_band_from_the_ground_reference():
    # The vehicle 0.4 m above a ground at z0 = −0.2 m (z = −0.6 m): ok only from z0.
    hist = PoseHistory(max_gap_s=0.2)
    for t in range(1_000_000, 3_000_001, POSE_DT_US):
        p = flight_pose(t)
        hist.add(Pose(p.t_us, p.x, p.y, -0.6, p.heading_rad))
    georef, _ = georeference(raw(0, [2_000_000]), hist, LINE, PARAMS, ground_z=-0.2)
    assert georef.reason == "ok" and georef.h == pytest.approx(0.4)
    georef, _ = georeference(raw(0, [2_000_000]), hist, LINE, PARAMS)
    assert georef.reason == "off_height" and georef.h == pytest.approx(0.6)
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(1, [2_000_000]), 3_000_000, 0.0)
    out = queue.release(hist, 3_000_000, 0.0, ground_z=-0.2)
    assert out[0].georef.reason == "ok" and out[0].ground_z == -0.2


# used_poses and replay


def run_queue(events, max_gap_s=0.2, ground=None, params=PARAMS, bag_order=False):
    """Feed ("pose", Pose) and ("raw", capture) events in order, flush at the end.

    ``ground``: live, a GroundReference fed by ("home", z) events; replay (None), z0
    fixed from each republished ("pose", (Pose, z0)) event.
    Returns the decisions {capture_id: (usable, reason, release, h)} and the republished
    poses in publication order, each before its capture, with the z0 of its release;
    with ``bag_order`` each raw capture at its arrival instead (as the bag records it:
    the raw capture when it arrived, its new poses when it was released).
    """
    replay = ground is None
    ground = GroundReference() if replay else ground
    hist = PoseHistory(max_gap_s=max_gap_s)
    queue = HoldQueue(LINE, params)
    decisions, published, stream = {}, [], []

    def take(released):
        for r in released:
            published.extend(r.new_poses)
            stream.extend(("pose", (p, r.ground_z)) for p in r.new_poses)
            if not bag_order:
                stream.append(("raw", r.georef.raw))
            decisions[r.georef.raw.capture_id] = (r.georef.usable, r.georef.reason, r.release,
                                                r.georef.h)

    for kind, item in events:
        if kind == "home":
            ground.update(item, valid=True)
            continue
        if kind == "pose":
            if replay:
                item, z0 = item
                ground.fix(z0)
            hist.add(item)
            queue.pose_arrived(0.0)
        else:
            queue.push(item, hist.newest.t_us if hist.newest else None, 0.0)
            if bag_order:
                stream.append(("raw", item))
        now = hist.newest.t_us if hist.newest else None
        take(queue.release(hist, now, 0.0, ground.value))
    take(queue.flush(hist, ground.value))
    return decisions, published, stream


def test_replaying_the_republished_poses_reproduces_the_decisions():
    # 50 Hz poses with a 0.3 s gap (above max_pose_gap_s) in the middle; a capture every
    # 50 ms with three samples; the last two captures lie beyond the last pose (flush).
    pose_times = [t for t in range(1_000_000, 4_000_001, POSE_DT_US)
                  if not 2_500_000 < t < 2_800_000]
    capture_stamps = [[t0 + k * 1350 for k in range(3)] for t0 in range(1_100_000, 4_100_000, 50_000)]
    # Home at z = 0.04 m, reset to z = −0.03 m (arming) before the first pose; the line
    # is flown 0.4 m above the reset home.
    events = [("home", 0.04), ("home", -0.03)]
    events += [("pose", flight_pose(t, h=0.43)) for t in pose_times]
    events += [("raw", raw(i, st)) for i, st in enumerate(capture_stamps)]
    # Each capture arrives right after the pose following its last sample (as live).
    events.sort(key=lambda e: -1 if e[0] == "home" else e[1].t_us if e[0] == "pose"
                else int(e[1].sample_stamps_us[-1]) + 1)
    ground = GroundReference()
    live, published, stream = run_queue(events, ground=ground)
    assert ground.z0 == pytest.approx(-0.03)
    replay, _, _ = run_queue(stream)
    assert set(live) == set(replay) == set(range(len(capture_stamps)))
    reasons = {v[1] for v in live.values()}
    assert {"ok", "no_pose"} <= reasons
    gap_captures = [i for i, st in enumerate(capture_stamps) if 2_500_000 < st[0] < 2_780_000]
    assert gap_captures and all(live[i][1] == "no_pose" for i in gap_captures)
    assert {v[2] for v in live.values()} == {"covered", "flush"}
    for i in live:
        assert live[i][:2] == replay[i][:2], f"capture {i}"
        assert live[i][3] == replay[i][3] or math.isnan(live[i][3]) and math.isnan(replay[i][3])
    ok_h = [v[3] for v in live.values() if v[1] == "ok"]
    assert ok_h == pytest.approx([0.4] * len(ok_h))            # from z0, not from z = 0
    assert len({p.t_us for p in published}) == len(published)      # each pose at most once
    assert len(published) < len(pose_times)
    assert {z0 for _, (_, z0) in [e for e in stream if e[0] == "pose"]} == {ground.z0}


def test_replay_in_bag_order_needs_no_hold_timeout():
    # As recorded in a CI run: each raw capture before the poses republished with
    # its release, the poses one at a time. After a 1 s pause of the sensor no pose was
    # republished, so the replay's "now" is 1 s old when the next capture arrives; its
    # first pose then passes the 0.5 s hold timeout before the rest arrive.
    pose_times = list(range(1_000_000, 4_000_001, POSE_DT_US))
    capture_t0 = [t for t in range(1_100_000, 3_900_000, 50_000)
                if not 2_000_000 <= t < 3_000_000]
    capture_stamps = [[t0 + k * 1350 for k in range(3)] for t0 in capture_t0]
    events = [("pose", flight_pose(t)) for t in pose_times]
    events += [("raw", raw(i, st)) for i, st in enumerate(capture_stamps)]
    events.sort(key=lambda e: e[1].t_us if e[0] == "pose"
                else int(e[1].sample_stamps_us[-1]) + 1)
    live, _, stream = run_queue(events, ground=GroundReference(), bag_order=True)
    assert {v[2] for v in live.values()} == {"covered"}
    timed_out, _, _ = run_queue(stream)
    after_pause = capture_t0.index(3_000_000)
    assert timed_out[after_pause][1:3] == ("no_pose", "timeout")
    replay, _, _ = run_queue(stream, params=replay_params(PARAMS))
    assert replay_params(PARAMS).hold_timeout_s == math.inf
    for i in live:
        assert live[i][:3] == replay[i][:3], f"capture {i}"


# decisions.jsonl and status


def test_decisions_jsonl_line_format(tmp_path):
    hist = history_of(range(1_000_000, 3_000_001, POSE_DT_US))
    queue = HoldQueue(LINE, PARAMS)
    queue.push(raw(7, [2_000_000]), 3_000_000, 0.0)
    queue.push(raw(8, [5_000_000]), 3_000_000, 0.0)
    log = DecisionLog(tmp_path / "out" / "decisions.jsonl")
    released = queue.release(hist, 3_000_000, 0.0) + queue.flush(hist)
    for r in released:
        log.append(r)
    lines = (tmp_path / "out" / "decisions.jsonl").read_text().splitlines()
    assert [json.loads(line) for line in lines] == [
        {"capture_id": 7, "usable": True, "reason": "ok", "s": pytest.approx(1.0), "release": "covered"},
        {"capture_id": 8, "usable": False, "reason": "no_pose", "s": None, "release": "flush"},
    ]
    assert list(json.loads(decision_line(released[0]))) == ["capture_id", "usable", "reason", "s",
                                                           "release"]


def test_status_counters():
    hist = history_of(range(1_000_000, 3_000_001, POSE_DT_US))
    status = StatusCounters(line_length_m=20.0, ds_m=0.05, gap_factor=1.5)
    queue = HoldQueue(LINE, PARAMS)
    status.on_pose(3_000_000)
    for i, stamp in enumerate([2_000_000, 2_050_000, 9_000_000]):
        status.on_raw(i)
        queue.push(raw(i, [stamp]), 3_000_000, 0.0)
    for r in queue.release(hist, 3_000_000, 0.0) + queue.flush(hist):
        status.on_release(r)
    snap = status.snapshot(queue_empty=len(queue) == 0)
    assert snap["reasons"] == list(REASONS)
    assert dict(zip(snap["reasons"], snap["counts"])) == {
        "ok": 2, "no_pose": 1, "outside_line": 0, "off_line": 0, "off_height": 0, "saturated": 0}
    assert (snap["captures_seen"], snap["released"], snap["usable"]) == (3, 3, 2)
    assert snap["has_capture"] and snap["last_capture_id"] == 2 and snap["queue_empty"]
    assert snap["now_us"] == snap["last_pose_stamp_us"] == 3_000_000
    assert snap["poses_used"] == 3                         # 2.0 s (exact), 2.04 and 2.06 s
    assert snap["max_gap_m"] == pytest.approx(18.95)


# The pure modules must not need ROS


def test_pure_modules_import_neither_rclpy_nor_px4_msgs():
    code = (
        "import sys\n"
        "import sensor_bridge.pose, sensor_bridge.line, sensor_bridge.coverage\n"
        "import sensor_bridge.georeference, sensor_bridge.px4_pose, sensor_bridge.ground\n"
        "bad = [m for m in sys.modules if m.split('.')[0] in ('rclpy', 'px4_msgs')]\n"
        "assert not bad, bad\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True, cwd=SENSOR_BRIDGE)
    for name in ("pose", "line", "coverage", "georeference", "ground"):
        text = (SENSOR_BRIDGE / "sensor_bridge" / f"{name}.py").read_text()
        assert "rclpy" not in text and "px4_msgs" not in text, name
