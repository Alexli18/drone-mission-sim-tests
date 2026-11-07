"""Line-survey helpers without ROS: node parameters, the sensor-lag guard, the figures
the test computes from the recordings, and the marker verdict (docs/line-survey.md).

Pure: the standard library, numpy-free, and ``sensor_bridge.coverage`` (pure). Unit
tested in ``tests/unit/test_survey.py``.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from itertools import pairwise

from sensor_bridge.coverage import coverage

WORK = "/work"              # the repository inside the ROS container (read-only)
OUT = "/out"                # the run's output directory inside the container
MODES = ("live", "replay")
STATUS_PERIOD_S = 1.0
FINALIZE_MARGIN_S = 10.0    # finalize_wait_s = timeouts_wall_s.finalize - this
REPLAY_POSE_TOPIC = "/survey/pose_used"

# ready_gate lists: what must exist before the sensor starts.
_READY = {
    "live": {
        "nodes": ["toy_sensor_node", "georeference_node", "marker_detector_node",
                  "rosapi", "rosbridge_websocket"],
        "services": ["/survey/stop", "/survey/flush", "/markers/finalize",
                     "/rosapi/node_details"],
        "topics_min_subscribers": ["/survey/capture:2", "/survey/pose_used:1",
                                   "/survey/georef_capture:2", "/survey/ready:1"],
        # Both nodes have PX4 poses before the sensor starts. Nothing needs to subscribe
        # to /sim/capture_truth: the test subscribes after the gate, and the figures only
        # use the captures from the flight.
        "flag_topics": ["/sim/pose_received", "/survey/pose_received"],
        "exit_when_ready": False,
    },
    "replay": {
        "nodes": ["georeference_node", "marker_detector_node", "rosapi", "rosbridge_websocket"],
        "services": ["/survey/flush", "/markers/finalize", "/rosapi/node_details"],
        "topics_min_subscribers": ["/survey/capture:1", "/survey/pose_used:1"],
        "exit_when_ready": True,
    },
}


def ds_m(cfg, speed_m_s: float) -> float:
    """Distance between two captures along the line (m)."""
    return speed_m_s / cfg.survey.sensor.capture_rate_hz


def node_params(scenario, cfg, seed_offset: int, mode: str, *,
                finalize_timeout_s: float) -> dict:
    """The ROS 2 parameter file: one ``<node>: {ros__parameters: ...}`` entry per node.

    ``mode``: ``live`` (all nodes) or ``replay`` (georeferencing node, detector, gate).
    A line without markers leaves the marker lists out: a ROS 2 Humble parameter file
    cannot hold an empty list.
    """
    if mode not in MODES:
        raise ValueError(f"mode: expected one of {MODES}, got {mode!r}")
    survey, ros_cfg = scenario.survey, cfg.ros
    geo, sensor, det = cfg.survey.georeference, cfg.survey.sensor, cfg.survey.detector
    line = survey.line
    ds = ds_m(cfg, scenario.speed_m_s)
    line_params = {
        "line_start_north_m": float(line.start_north_m),
        "line_start_east_m": float(line.start_east_m),
        "line_heading_deg": float(line.heading_deg),
        "line_length_m": float(line.length_m),
    }
    georef = {
        **line_params,
        "sensor_height_m": float(survey.sensor_height_m),
        "sensor_offset_m": [float(v) for v in survey.sensor_offset_m],
        "off_line_m": geo.off_line_m,
        "height_band_m": geo.height_band_m,
        "hold_timeout_s": geo.hold_timeout_s,
        "wall_fallback_s": geo.wall_fallback_s,
        "max_pose_gap_s": geo.max_pose_gap_s,
        "coverage_gap_factor": geo.coverage_gap_factor,
        "ds_m": ds,
        "status_period_s": STATUS_PERIOD_S,
        "decisions_path": f"{OUT}/decisions.jsonl",
        "home_topic": ros_cfg.home_topic,      # unused in replay (z0 from the recording)
    }
    if mode == "live":
        georef.update(pose_topic=ros_cfg.pose_topic, pose_type="px4", republish_poses=True)
    else:
        georef.update(pose_topic=REPLAY_POSE_TOPIC, pose_type="sample", republish_poses=False)
    nodes = {"georeference_node": georef}
    if mode == "live":
        toy = {
            "seed": int(survey.seed) + int(seed_offset),
            "clock_offset_ms": float(survey.clock.offset_ms),
            "clock_jitter_ms": float(survey.clock.jitter_ms),
            **line_params,
            "sensor_offset_m": [float(v) for v in survey.sensor_offset_m],
            "capture_rate_hz": sensor.capture_rate_hz,
            "samples_per_capture": sensor.samples_per_capture,
            "sample_spacing_s": sensor.sample_spacing_s,
            "footprint_sigma_m": sensor.footprint_sigma_m,
            "noise": sensor.noise,
            "max_pose_gap_s": sensor.max_pose_gap_s,
            "pose_topic": ros_cfg.pose_topic,
            "wait_for_ready": True,
        }
        if survey.markers:
            toy["markers_s_m"] = [m.s_m for m in survey.markers]
            toy["markers_d_m"] = [m.d_m for m in survey.markers]
            toy["markers_amplitude"] = [m.amplitude for m in survey.markers]
        nodes["toy_sensor_node"] = toy
    nodes["marker_detector_node"] = {
        "ds_m": ds,
        "threshold": det.threshold,
        "min_captures": det.min_captures,
        "max_gap_factor": det.max_gap_factor,
        "markers_path": f"{OUT}/markers.json",
        "finalize_wait_s": float(finalize_timeout_s) - FINALIZE_MARGIN_S,
    }
    nodes["ready_gate"] = {key: (list(value) if isinstance(value, list) else value)
                           for key, value in _READY[mode].items()}
    return {name: {"ros__parameters": params} for name, params in nodes.items()}


# The sensor-lag guard ---------------------------------------------------------------


def sensor_lag_s(status: dict | None, truth: dict | None) -> float | None:
    """The georeferencing node's newest pose time minus the newest true sample time (s).

    Large when the sensor node falls behind the poses: its captures then reach the
    georeferencing node after the poses they need are gone (``no_pose``).
    """
    if not status or not truth or not truth.get("sample_true_us"):
        return None
    newest_true = max(int(t) for t in truth["sample_true_us"])
    return (int(status["now_us"]) - newest_true) / 1e6


def reason_counts(status: dict) -> dict:
    return dict(zip(status["reasons"], status["counts"], strict=True))


def takeoff_sim_s(recorder) -> float | None:
    """Simulated time at which MAVSDK first reported PX4's TAKEOFF mode (None: never)."""
    return next((sim_t for sim_t, _, mode in recorder.mode_history() if mode == "TAKEOFF"),
                None)


# Truth figures from the two recordings -----------------------------------------------


def _floats(values) -> list[float]:
    """A recorded float array: JSON null (a NaN on the topic) becomes NaN."""
    return [math.nan if v is None else float(v) for v in values]


def _nanmean(values) -> float:
    kept = [v for v in values if not math.isnan(v)]
    return sum(kept) / len(kept) if kept else math.nan


@dataclass
class TruthFigures:
    pairs: int
    usable_pairs: int
    coverage: float
    max_gap_m: float
    clock_shift_mean_m: float
    usable_without_truth: int
    samples_without_pose: int


def truth_figures(truth_msgs, georef_msgs, length_m: float, ds: float,
                  gap_factor: float) -> TruthFigures:
    """Georeferenced captures against where the sensor really was (the PX4 estimate at
    the samples' true times), paired by capture id.

    * coverage: ``sensor_bridge.coverage`` over the true mean positions of the usable
      captures (the georeferencing node's own figure uses its positions);
    * clock shift: mean over the usable captures' samples of the georeferenced
      position minus the true one (v x offset for a clock offset);
    * usable without truth: usable captures with a sample the sensor had no pose for.

    Only the ids present in both recordings are paired, and a repeated id keeps its last
    message, so a lost or duplicated capture does not show here: ``capture_stream_failures``
    checks the streams themselves.
    """
    truth = {int(m["capture_id"]): m for m in truth_msgs}
    georef = {int(m["raw"]["capture_id"]): m for m in georef_msgs}
    ids = sorted(set(truth) & set(georef))
    true_s, usable, shifts, without_truth = [], [], [], 0
    for i in ids:
        t, g = truth[i], georef[i]
        ts = _floats(t["sample_s"])
        true_s.append(_nanmean(ts))
        usable.append(bool(g["usable"]))
        if not g["usable"]:
            continue
        without_truth += any(math.isnan(v) for v in ts)
        shifts.extend(gs - v for gs, v in zip(_floats(g["sample_s"]), ts, strict=True))
    cov = coverage(true_s, usable, length_m, ds, gap_factor)
    return TruthFigures(
        pairs=len(ids), usable_pairs=sum(usable), coverage=cov.fraction, max_gap_m=cov.max_gap_m,
        clock_shift_mean_m=_nanmean(shifts), usable_without_truth=without_truth,
        samples_without_pose=sum(sum(math.isnan(v) for v in _floats(m["sample_s"]))
                                 for m in truth.values()))


# Capture stream integrity ------------------------------------------------------------


def _id_text(ids, limit: int = 10) -> str:
    ids = sorted(ids)
    text = ", ".join(str(i) for i in ids[:limit])
    return text + (f", ... ({len(ids)} in all)" if len(ids) > limit else "")


def _stream_problems(ids: list[int], start: int, last: int) -> list[str]:
    """What is wrong with one recorded stream of capture ids, in the window start..last."""
    problems = []
    repeated = [i for i, n in Counter(ids).items() if n > 1]
    if repeated:
        problems.append(f"repeated ids {_id_text(repeated)}")
    missing = set(range(start, last + 1)) - set(ids)
    if missing:
        problems.append(f"missing ids {_id_text(missing)}")
    beyond = [i for i in ids if i > last]
    if beyond:
        problems.append(f"ids after the last published one ({last}): {_id_text(beyond)}")
    inversion = next(((a, b) for a, b in pairwise(ids) if b < a), None)
    if inversion:
        problems.append(f"out of order (id {inversion[1]} arrived after {inversion[0]})")
    return problems


def capture_window_start(truth_msgs, georef_msgs) -> int | None:
    """First capture id both recordings can hold: the test subscribes after the sensor has
    started, so each recording begins at the first capture it received. None: a stream is
    empty."""
    truth = [int(m["capture_id"]) for m in truth_msgs]
    georef = [int(m["raw"]["capture_id"]) for m in georef_msgs]
    if not truth or not georef:
        return None
    return max(min(truth), min(georef))


def capture_stream_failures(truth_msgs, georef_msgs, status: dict,
                            last: int) -> list[tuple[str, str]]:
    """Every capture the sensor published reached the georeferencing node and the test.

    ``last``: the id the sensor reported at its stop (ids run 0, 1, 2, ... without gaps).
    Returns (expected, actual) failures:

    * both recordings hold every id from the window start (``capture_window_start``) to
      ``last`` exactly once and in order, nothing after ``last``;
    * the georeferencing node counted ``last + 1`` raw captures and released as many (it
      counts from the first capture, so a capture lost before the test subscribed shows
      here).
    """
    failures = []
    start = capture_window_start(truth_msgs, georef_msgs)
    streams = (("truth", [int(m["capture_id"]) for m in truth_msgs]),
               ("georeferenced", [int(m["raw"]["capture_id"]) for m in georef_msgs]))
    for name, ids in streams:
        if not ids:
            failures.append((f"the {name} recording holds the captures of the flight",
                             "no message recorded"))
            continue
        problems = _stream_problems(ids, min(ids) if start is None else start, last)
        if problems:
            first = min(ids) if start is None else start
            failures.append(
                (f"the {name} recording holds every capture id {first}..{last} once, in order",
                 "; ".join(problems)))
    expected = last + 1
    for key, what in (("captures_seen", "raw captures counted by the georeferencing node"),
                      ("released", "captures released by the georeferencing node")):
        if int(status[key]) != expected:
            failures.append((f"{what}: {expected} (ids 0..{last})", f"{status[key]}"))
    return failures


# Marker verdict ----------------------------------------------------------------------


@dataclass
class MarkerVerdict:
    errors_m: list[float | None]    # per expected marker: found - expected, None if missed
    false_markers: list[float]      # positions of found markers matching no expected one
    failures: list[tuple[str, str]]  # (expected, actual) texts

    @property
    def mean_error_m(self) -> float | None:
        found = [e for e in self.errors_m if e is not None]
        return sum(found) / len(found) if found else None


def evaluate_markers(expected_s, found_s, tol_m: float) -> MarkerVerdict:
    """Each expected marker needs the nearest unused found marker within ``tol_m``; found
    markers left over are false markers."""
    unused = sorted(found_s)
    errors, failures = [], []
    for s in sorted(expected_s):
        best = min(unused, key=lambda f: abs(f - s), default=None)
        if best is not None and abs(best - s) <= tol_m:
            unused.remove(best)
            errors.append(best - s)
        else:
            errors.append(None)
            failures.append((f"a marker found within {tol_m} m of s = {s:.2f} m",
                             "none" if best is None else f"nearest {best:.3f} m"))
    if unused:
        failures.append(("no marker where none is painted",
                         "false markers at s = " + ", ".join(f"{f:.2f} m" for f in unused)))
    return MarkerVerdict(errors_m=errors, false_markers=unused, failures=failures)
