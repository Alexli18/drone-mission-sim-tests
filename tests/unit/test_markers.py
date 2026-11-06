"""The toy sensor's reading model, the marker detector and the simulated sensor (no ROS).

The last tests run the whole chain in process (poses -> sensor -> georeferencing ->
detector), which is what the flight test does with PX4 and ROS: they pin down what the
flight test may expect (marker positions, the clock-offset shift, the lever arm) before
a simulator is involved.
"""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_bridge.georeference import GeorefParams, HoldQueue
from sensor_bridge.line import Line
from sensor_bridge.pose import Pose, PoseHistory
from survey_demo.markers import GroundMarker, detect_markers, reading
from survey_demo.sensor_sim import SensorParams, SensorSim

pytestmark = pytest.mark.unit

LINE = Line(start_north_m=5.0, start_east_m=0.0, heading_deg=0.0, length_m=12.0)
SENSOR = SensorParams(capture_rate_hz=20.0, samples_per_capture=4, sample_spacing_s=0.01,
                      footprint_sigma_m=0.2, noise=0.02)
MARKERS = [GroundMarker(2.5, 0.0, 0.8), GroundMarker(6.0, 0.15, 0.7), GroundMarker(9.5, 0.0, 0.8)]
SPEED = 1.0
POSE_DT_US = 20_000
OFFSET = (0.15, 0.0, 0.0)       # the sensor sits 15 cm ahead of the vehicle origin


# The reading model


def test_reading_peaks_on_the_marker_and_falls_off_with_distance():
    m = [GroundMarker(3.0, 0.0, 0.8)]
    assert reading(3.0, 0.0, m, 0.2) == pytest.approx(0.8)
    assert reading(3.2, 0.0, m, 0.2) == pytest.approx(0.8 * math.exp(-0.5))
    assert reading(3.0, 0.2, m, 0.2) == pytest.approx(reading(3.2, 0.0, m, 0.2))
    assert reading(8.0, 0.0, m, 0.2) == pytest.approx(0.0, abs=1e-12)


def test_readings_add_up_and_stop_at_full_scale():
    m = [GroundMarker(3.0, 0.0, 0.8), GroundMarker(3.0, 0.0, 0.8)]
    assert reading(3.0, 0.0, m, 0.2) == 1.0


# The detector


def captures_over(s_marker, amplitude=0.8, sigma=0.2, ds=0.05):
    grid = np.arange(0.0, 12.0, ds)
    return [(float(s), amplitude * math.exp(-0.5 * ((s - s_marker) / sigma) ** 2)) for s in grid]


def test_detector_finds_the_centroid_of_a_marker():
    found = detect_markers(captures_over(6.01), threshold=0.3, min_captures=3, max_gap_m=0.15)
    assert [m.s_m for m in found] == pytest.approx([6.01], abs=0.01)   # grid and threshold truncation
    assert found[0].strength == pytest.approx(0.8, abs=0.02)
    assert found[0].n_captures >= 8


def test_detector_separates_markers_and_ignores_a_blip():
    data = sorted(captures_over(2.5) + captures_over(6.0))
    data = [(s, max(r, 0.0)) for s, r in data]
    data.append((9.0, 0.9))                                  # a single capture above threshold
    found = detect_markers(data, threshold=0.3, min_captures=3, max_gap_m=0.15)
    assert len(found) == 2
    assert [m.s_m for m in found] == pytest.approx([2.5, 6.0], abs=0.01)


def test_detector_on_a_clean_line_finds_nothing():
    rng = np.random.default_rng(1)
    data = [(float(s), float(abs(rng.normal(0, 0.02)))) for s in np.arange(0, 12, 0.05)]
    assert detect_markers(data, 0.3, 3, 0.15) == []


def test_detector_skips_nan_positions():
    data = [(math.nan, 0.9)] * 5 + captures_over(3.0)
    assert len(detect_markers(data, 0.3, 3, 0.15)) == 1


# The simulated sensor


def pose_at(t_us, d=0.0, h=0.4):
    """The vehicle on LINE (heading north) at 1 m/s, s = 0 at t = 1 s."""
    return Pose(t_us, 5.0 + SPEED * (t_us - 1_000_000) / 1e6, d, -h, 0.0)


def history(t0=0, t1=15_000_000, **kw):
    hist = PoseHistory(max_gap_s=0.5, max_age_s=1000.0)
    for t in range(t0, t1 + 1, POSE_DT_US):
        hist.add(pose_at(t, **kw))
    return hist


def make_sim(seed=0, markers=MARKERS, offset_ms=0.0, jitter_ms=0.0, sensor_offset=OFFSET,
             start_us=1_000_000):
    return SensorSim(SENSOR, markers, LINE, sensor_offset, offset_ms, jitter_ms,
                     np.random.default_rng([seed, 0]), np.random.default_rng([seed, 1]),
                     start_us)


def test_a_capture_is_emitted_only_when_its_last_sample_is_covered():
    sim = make_sim()
    hist = history()
    # capture 0: samples at 1.000 .. 1.030 s
    assert sim.advance_to(1_029_999, hist) == []
    out = sim.advance_to(1_030_000, hist)
    assert [c.capture_id for c, _ in out] == [0]
    assert out[0][1].sample_true_us == [1_000_000, 1_010_000, 1_020_000, 1_030_000]


def test_captures_do_not_depend_on_how_the_poses_arrive():
    one_go = make_sim(seed=3).advance_to(6_000_000, history())
    sim = make_sim(seed=3)
    hist = history()
    stepwise = []
    for newest in range(1_000_000, 6_000_001, 7_000):          # a different batching
        stepwise += sim.advance_to(newest, hist)
    stepwise += sim.advance_to(6_000_000, hist)
    assert [c.capture_id for c, _ in stepwise] == [c.capture_id for c, _ in one_go]
    for (a, ta), (b, tb) in zip(one_go, stepwise, strict=True):
        assert a.readings == b.readings and a.sample_stamps_us == b.sample_stamps_us
        assert ta.sample_s == tb.sample_s


def test_same_seed_same_captures_other_seed_other_noise():
    a = make_sim(seed=1).advance_to(3_000_000, history())
    b = make_sim(seed=1).advance_to(3_000_000, history())
    c = make_sim(seed=2).advance_to(3_000_000, history())
    assert [x.readings for x, _ in a] == [x.readings for x, _ in b]
    assert [x.readings for x, _ in a] != [x.readings for x, _ in c]


def test_clock_offset_moves_the_stamps_but_not_the_truth():
    plain = make_sim().advance_to(3_000_000, history())
    shifted = make_sim(offset_ms=20.0).advance_to(3_000_000, history())
    for (a, ta), (b, tb) in zip(plain, shifted, strict=True):
        assert [s + 20_000 for s in a.sample_stamps_us] == b.sample_stamps_us
        assert ta.sample_true_us == tb.sample_true_us and ta.sample_s == tb.sample_s


def test_samples_without_bracketing_poses_get_nan_truth_and_are_counted():
    hist = history(t0=2_000_000, t1=6_000_000)                 # no poses before 2 s
    sim = make_sim()
    out = sim.advance_to(3_000_000, hist)
    first = out[0][1]
    assert all(math.isnan(v) for v in first.sample_s)
    assert sim.without_pose >= 4
    assert not any(math.isnan(v) for v in out[-1][1].sample_s)


def test_stop_ends_the_stream_and_reports_the_last_id():
    sim = make_sim()
    hist = history()
    sim.advance_to(2_000_000, hist)
    last = sim.stop()
    assert last == sim.next_id - 1
    assert sim.advance_to(9_000_000, hist) == []
    assert make_sim().stop() is None


# The whole chain without ROS


GEOREF = GeorefParams(sensor_height_m=0.4, sensor_offset_m=OFFSET, off_line_m=0.5,
                      height_band_m=0.15, hold_timeout_s=0.5, wall_fallback_s=2.0)


def fly(offset_ms=0.0, jitter_ms=0.0, seed=0, markers=MARKERS, georef=GEOREF):
    """Poses -> sensor -> georeferencing -> detector; returns (found markers, decisions)."""
    hist = history()
    sim = make_sim(seed=seed, markers=markers, offset_ms=offset_ms, jitter_ms=jitter_ms)
    queue = HoldQueue(LINE, georef)
    released = []
    for newest in range(1_000_000, 15_000_001, POSE_DT_US):
        for capture, _ in sim.advance_to(newest, hist):
            raw = SimpleNamespace(
                capture_id=capture.capture_id, saturated=capture.saturated,
                sample_stamps_us=np.array(capture.sample_stamps_us, dtype=np.uint64),
                readings=capture.readings)
            queue.push(raw, newest, 0.0)
        released += queue.release(hist, newest, 0.0, ground_z=0.0)
    released += queue.flush(hist)
    usable = [(r.georef.s, float(np.mean(r.georef.raw.readings)))
              for r in released if r.georef.usable]
    return detect_markers(usable, 0.3, 3, 1.5 * 3 * SPEED / 20.0), released


def test_chain_recovers_the_markers_within_2_cm():
    found, released = fly()
    assert [m.s_m for m in found] == pytest.approx([2.5, 6.0, 9.5], abs=0.02)
    reasons = {r.georef.reason for r in released}
    assert "ok" in reasons and reasons <= {"ok", "outside_line", "no_pose"}


@pytest.mark.parametrize("offset_ms", [20.0, -20.0])
def test_chain_shifts_every_marker_by_speed_times_clock_offset(offset_ms):
    found, _ = fly(offset_ms=offset_ms, jitter_ms=1.0)
    errors = [m.s_m - s for m, s in zip(found, [2.5, 6.0, 9.5], strict=True)]
    assert errors == pytest.approx([SPEED * offset_ms / 1000.0] * 3, abs=0.015)


def test_chain_with_a_wrong_lever_arm_is_caught_by_the_marker_position():
    """The georeferencing ignores the 15 cm offset: every marker is 15 cm off (tolerance
    10 cm), which is why the flight scenarios use a non-zero offset."""
    wrong = GeorefParams(sensor_height_m=0.4, sensor_offset_m=(0.0, 0.0, 0.0), off_line_m=0.5,
                         height_band_m=0.15, hold_timeout_s=0.5, wall_fallback_s=2.0)
    found, _ = fly(georef=wrong)
    errors = [m.s_m - s for m, s in zip(found, [2.5, 6.0, 9.5], strict=True)]
    assert errors == pytest.approx([-0.15] * 3, abs=0.02)


def test_chain_on_a_clean_line_reports_no_marker():
    found, _ = fly(markers=[])
    assert found == []
