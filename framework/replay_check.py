"""Replay check: compare a live run with its replay, without ROS.

The runner reads the files both runs wrote (``decisions.jsonl``, ``markers.json``) and
the live bag's ``metadata.yaml``; the functions here return lists of mismatch texts
(empty = equal), so the test can report every difference at once.
"""

from __future__ import annotations

import json
import sqlite3
import struct
from pathlib import Path

import yaml

CAPTURE_TOPIC = "/survey/capture"


def bag_message_counts(bag_dir: Path) -> dict[str, int]:
    """Message count per topic from a rosbag2 ``metadata.yaml``."""
    meta = yaml.safe_load((Path(bag_dir) / "metadata.yaml").read_text())
    info = meta["rosbag2_bagfile_information"]
    return {entry["topic_metadata"]["name"]: int(entry["message_count"])
            for entry in info.get("topics_with_message_count", [])}


def capture_stamp(cdr: bytes) -> tuple[int, int]:
    """(capture_id, header stamp in µs) of a CDR-serialized ``sensor_bridge_msgs/SensorCapture``.

    Reads only the leading fields: the 4-byte encapsulation header, ``header``
    (``stamp`` sec and nanosec, ``frame_id``), ``sensor_id`` and ``capture_id``. The
    stamp is the sensor clock of the capture's first sample (simulated time).
    """
    if cdr[1] != 1:
        raise ValueError(f"expected little-endian CDR, got encapsulation {cdr[:2].hex()}")
    o = 4
    sec, nanosec = struct.unpack_from("<iI", cdr, o)
    o += 8
    for _ in range(2):                       # header.frame_id, sensor_id
        o += (-(o - 4)) % 4
        (n,) = struct.unpack_from("<I", cdr, o)
        o += 4 + n
    o += (-(o - 4)) % 8
    (capture_id,) = struct.unpack_from("<Q", cdr, o)
    return int(capture_id), sec * 1_000_000 + nanosec // 1000


def capture_stamps_us(bag_dir: Path) -> dict[int, int]:
    """Header stamp (µs) per capture id of the recorded ``/survey/capture`` (sqlite3 bag)."""
    bag_dir = Path(bag_dir)
    meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
    out = {}
    for name in meta["rosbag2_bagfile_information"]["relative_file_paths"]:
        con = sqlite3.connect(f"file:{bag_dir / name}?mode=ro", uri=True)
        try:
            rows = con.execute("SELECT m.data FROM messages m JOIN topics t "
                               "ON t.id = m.topic_id WHERE t.name = ?", (CAPTURE_TOPIC,))
            for (data,) in rows:
                capture_id, stamp_us = capture_stamp(bytes(data))
                out[capture_id] = stamp_us
        finally:
            con.close()
    return out


def compared_captures(stamps_us: dict[int, int], start_us: int) -> set[int]:
    """The replay check compares the captures whose first sample is at or after ``start_us``."""
    return {k for k, t in stamps_us.items() if t >= start_us}


def read_decisions(path: Path) -> list[dict]:
    """``decisions.jsonl``: one dict per released capture, in release order."""
    text = Path(path).read_text() if Path(path).exists() else ""
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def read_markers(path: Path) -> list[dict]:
    return json.loads(Path(path).read_text())["markers"]


def recording_errors(counts: dict[str, int], status: dict, decisions: list[dict]) -> list[str]:
    """Precheck: the bag holds what the live georeferencing node saw and published.

    ``status`` is the live run's final ``/survey/status``: ``captures_seen`` raw
    captures, ``poses_used`` republished poses, ``released`` decisions.
    """
    errors = []
    for topic, field in ((CAPTURE_TOPIC, "captures_seen"), ("/survey/pose_used", "poses_used")):
        if counts.get(topic, 0) != int(status[field]):
            errors.append(f"bag {topic} {counts.get(topic, 0)} messages, live status "
                          f"{field} {status[field]}")
    if len(decisions) != int(status["released"]):
        errors.append(f"live decisions.jsonl {len(decisions)} lines, live status released "
                      f"{status['released']}")
    return errors


def decision_mismatches(live: list[dict], replay: list[dict], limit: int = 10,
                        compared: set[int] | None = None) -> list[str]:
    """For every capture id the live run released (in ``compared``, if given): the same
    ``usable`` and ``reason``."""
    replayed = {d["capture_id"]: d for d in replay}
    out = []
    for d in live:
        if compared is not None and d["capture_id"] not in compared:
            continue
        r = replayed.get(d["capture_id"])
        if r is None:
            out.append(f"capture {d['capture_id']}: not released in the replay")
        elif (r["usable"], r["reason"]) != (d["usable"], d["reason"]):
            out.append(f"capture {d['capture_id']}: live {d['reason']} "
                       f"({'usable' if d['usable'] else 'unusable'}), replay {r['reason']} "
                       f"({'usable' if r['usable'] else 'unusable'})")
    if len(out) > limit:
        out = out[:limit] + [f"... {len(out) - limit} more"]
    return out


def marker_mismatches(live: list[dict], replay: list[dict], s_tol_m: float) -> list[str]:
    """Same number of markers; paired in s order, each within ``s_tol_m`` along the line."""
    if len(live) != len(replay):
        return [(f"{len(live)} live markers at s = "
                f"{[round(m['s_m'], 3) for m in live]}, {len(replay)} replayed at s = "
                f"{[round(m['s_m'], 3) for m in replay]}")]
    out = []
    pairs = zip(sorted(live, key=lambda m: m["s_m"]), sorted(replay, key=lambda m: m["s_m"]),
                strict=True)
    for k, (a, b) in enumerate(pairs):
        if abs(a["s_m"] - b["s_m"]) > s_tol_m:
            out.append(f"marker {k}: live at {a['s_m']:.3f} m, replay at {b['s_m']:.3f} m")
    return out
