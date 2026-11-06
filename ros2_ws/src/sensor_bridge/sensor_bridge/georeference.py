"""Georeferencing of raw captures: per-sample positions, usable flag, hold queue (pure).

No ROS: the node (``georeference_node``) is a thin wrapper. A raw capture is any object
with the ``SensorCapture`` fields used here (``capture_id``, ``sample_stamps_us``,
``saturated``): the ROS message or a plain data class.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from sensor_bridge.coverage import coverage
from sensor_bridge.line import offset_xyz, sensor_position
from sensor_bridge.pose import interpolate_pair

REASON_OK = "ok"
REASON_NO_POSE = "no_pose"
REASON_OUTSIDE_LINE = "outside_line"
REASON_OFF_LINE = "off_line"
REASON_OFF_HEIGHT = "off_height"
REASON_SATURATED = "saturated"
REASONS = (REASON_OK, REASON_NO_POSE, REASON_OUTSIDE_LINE, REASON_OFF_LINE, REASON_OFF_HEIGHT,
           REASON_SATURATED)

RELEASE_COVERED = "covered"
RELEASE_TIMEOUT = "timeout"
RELEASE_WALL = "wall"
RELEASE_FLUSH = "flush"


@dataclass(frozen=True)
class GeorefParams:
    """The node parameters the georeferencing logic needs (``framework.survey.node_params``)."""

    sensor_height_m: float
    sensor_offset_m: tuple[float, float, float]
    off_line_m: float
    height_band_m: float
    hold_timeout_s: float
    wall_fallback_s: float


def replay_params(cfg: GeorefParams) -> GeorefParams:
    """The replay's parameters: ``cfg`` without the simulated-time hold timeout.

    In a replay the node's "now" is the newest *republished* pose, not PX4's newest
    pose: poses come only with the live releases, after the raw capture (as recorded),
    and one at a time, so "now" can pass a held capture's arrival by the hold timeout
    before the capture's own poses arrive. In a CI replay this released 90 captures by
    timeout as ``no_pose`` that the live run had released ``covered`` (``outside_line``).
    Without the timeout a capture is released once it is covered, as live; the wall
    fallback and the flush still release the tail. A capture the live run released by
    timeout can still differ; the replay check compares only captures from takeoff on.
    """
    return replace(cfg, hold_timeout_s=math.inf)


@dataclass
class GeorefCaptureData:
    """The fields of ``sensor_bridge_msgs/GeorefCapture``."""

    raw: object
    sample_s: np.ndarray           # float32 [n_samples]; NaN without a pose
    sample_d: np.ndarray
    sample_h: np.ndarray
    sample_has_pose: np.ndarray    # bool [n_samples]
    s: float                        # mean over samples with a pose; NaN if none
    d: float
    h: float
    usable: bool
    reason: str


def georeference(raw, history, line, cfg: GeorefParams, ground_z: float = 0.0):
    """(GeorefCaptureData, used_poses) of ``raw`` with the poses in ``history``.

    Per sample: the pose interpolated at the sample stamp, the sensor offset
    applied, projected onto ``line``; heights from the ground reference ``ground_z``
    (z0, ``sensor_bridge.ground``). ``used_poses`` are the bracketing poses of each
    sample that has a pose, in time order, without repeats.
    """
    offset = offset_xyz(cfg.sensor_offset_m)
    stamps = [int(t) for t in raw.sample_stamps_us]
    n = len(stamps)
    cs, cd, ch = (np.full(n, np.nan, dtype=np.float32) for _ in range(3))
    has_pose = np.zeros(n, dtype=bool)
    used = {}
    for k, t in enumerate(stamps):
        pair = history.bracketing(t)
        if pair is None:
            continue
        for p in pair:
            used[p.t_us] = p
        north, east, h = sensor_position(interpolate_pair(pair[0], pair[1], t), offset,
                                       ground_z)
        cs[k], cd[k] = line.project(north, east)
        ch[k] = h
        has_pose[k] = True
    if has_pose.any():
        s, d, h = (float(np.mean(v[has_pose].astype(np.float64))) for v in (cs, cd, ch))
    else:
        s = d = h = math.nan
    if not has_pose.all():
        reason = REASON_NO_POSE
    elif s < 0.0 or s > line.length_m:
        reason = REASON_OUTSIDE_LINE
    elif abs(d) > cfg.off_line_m:
        reason = REASON_OFF_LINE
    elif abs(h - cfg.sensor_height_m) > cfg.height_band_m:
        reason = REASON_OFF_HEIGHT
    elif bool(raw.saturated):
        reason = REASON_SATURATED
    else:
        reason = REASON_OK
    georef = GeorefCaptureData(raw=raw, sample_s=cs, sample_d=cd, sample_h=ch,
                             sample_has_pose=has_pose, s=s, d=d, h=h,
                             usable=reason == REASON_OK, reason=reason)
    return georef, [used[t] for t in sorted(used)]


@dataclass
class Released:
    """One released capture: its georeferenced data, the cause, and the poses it used.

    ``new_poses`` are the used poses not republished before (each pose at most once);
    ``ground_z`` is the z0 its heights were measured from (republished with the poses).
    """

    georef: GeorefCaptureData
    release: str
    used_poses: list
    new_poses: list
    ground_z: float = 0.0


class HoldQueue:
    """FIFO of raw captures, each with the node's "now" (newest pose time) at its arrival."""

    def __init__(self, line, cfg: GeorefParams):
        self.line = line
        self.cfg = cfg
        self._items: deque = deque()
        self._last_pose_wall = None
        self._published: set = set()

    def __len__(self) -> int:
        return len(self._items)

    def push(self, raw, now_us: int | None, now_wall: float) -> None:
        self._items.append((raw, now_us, now_wall))

    def pose_arrived(self, now_wall: float) -> None:
        """Note the wall-clock time of the newest pose (for the wall fallback)."""
        self._last_pose_wall = now_wall

    def _cause(self, item, history, now_us, now_wall):
        raw, arrival_us, arrival_wall = item
        if history.covers(max(int(t) for t in raw.sample_stamps_us)):
            return RELEASE_COVERED
        if (now_us is not None and arrival_us is not None
                and now_us - arrival_us >= self.cfg.hold_timeout_s * 1e6):
            return RELEASE_TIMEOUT
        since = self._last_pose_wall if self._last_pose_wall is not None else arrival_wall
        if now_wall - since >= self.cfg.wall_fallback_s:
            return RELEASE_WALL
        return None

    def _release(self, raw, history, cause, ground_z) -> Released:
        georef, used = georeference(raw, history, self.line, self.cfg, ground_z)
        new = [p for p in used if p.t_us not in self._published]
        self._published.update(p.t_us for p in new)
        return Released(georef=georef, release=cause, used_poses=used, new_poses=new,
                        ground_z=float(ground_z))

    def release(self, history, now_us: int | None, now_wall: float,
                ground_z: float = 0.0) -> list:
        """Release from the head while it is covered, timed out, or past the wall fallback."""
        out = []
        while self._items:
            cause = self._cause(self._items[0], history, now_us, now_wall)
            if cause is None:
                break
            raw = self._items.popleft()[0]
            out.append(self._release(raw, history, cause, ground_z))
        return out

    def flush(self, history, ground_z: float = 0.0) -> list:
        """Release every held capture (``flush``); samples without a pose become no_pose."""
        out = []
        while self._items:
            raw = self._items.popleft()[0]
            out.append(self._release(raw, history, RELEASE_FLUSH, ground_z))
        return out


def decision_line(released: Released) -> str:
    """One ``decisions.jsonl`` line (Shared conventions); a NaN s is written as null."""
    g = released.georef
    s = None if math.isnan(g.s) else float(g.s)
    return json.dumps({"capture_id": int(g.raw.capture_id), "usable": bool(g.usable),
                       "reason": g.reason, "s": s, "release": released.release})


class DecisionLog:
    """Appends one line per released capture to ``decisions.jsonl``, in release order."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("")

    def append(self, released: Released) -> None:
        with open(self.path, "a") as f:
            f.write(decision_line(released) + "\n")


class StatusCounters:
    """The fields of ``sensor_bridge_msgs/SurveyStatus`` (see ``snapshot``)."""

    def __init__(self, line_length_m: float, ds_m: float, gap_factor: float):
        self.line_length_m = line_length_m
        self.ds_m = ds_m
        self.gap_factor = gap_factor
        self.now_us = 0
        self.pose_received = False
        self.last_pose_stamp_us = 0
        self.captures_seen = 0
        self.has_capture = False
        self.last_capture_id = 0
        self.released = 0
        self.usable = 0
        self.counts = {r: 0 for r in REASONS}
        self.poses_used = 0
        self._s: list = []
        self._usable: list = []

    def on_pose(self, t_us: int) -> None:
        self.pose_received = True
        self.last_pose_stamp_us = max(self.last_pose_stamp_us, int(t_us))
        self.now_us = self.last_pose_stamp_us

    def on_raw(self, capture_id: int) -> None:
        self.captures_seen += 1
        self.last_capture_id = max(self.last_capture_id, int(capture_id)) if self.has_capture \
            else int(capture_id)
        self.has_capture = True

    def on_release(self, released: Released, republished: bool = True) -> None:
        """Count a released capture; ``republished``: its new poses went to /survey/pose_used."""
        g = released.georef
        self.released += 1
        self.usable += int(g.usable)
        self.counts[g.reason] += 1
        if republished:
            self.poses_used += len(released.new_poses)
        self._s.append(g.s)
        self._usable.append(g.usable)

    def snapshot(self, queue_empty: bool) -> dict:
        cov = coverage(self._s, self._usable, self.line_length_m, self.ds_m, self.gap_factor)
        return {
            "now_us": self.now_us, "pose_received": self.pose_received,
            "last_pose_stamp_us": self.last_pose_stamp_us, "captures_seen": self.captures_seen,
            "has_capture": self.has_capture, "last_capture_id": self.last_capture_id,
            "released": self.released, "usable": self.usable, "reasons": list(REASONS),
            "counts": [self.counts[r] for r in REASONS], "poses_used": self.poses_used,
            "coverage": cov.fraction, "max_gap_m": cov.max_gap_m, "queue_empty": queue_empty,
        }
