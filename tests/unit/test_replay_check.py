"""Replay check helpers (``framework.replay_check``)."""

import json
import sqlite3
import struct

import pytest
import yaml

from framework.replay_check import (
    bag_message_counts,
    capture_stamp,
    capture_stamps_us,
    compared_captures,
    decision_mismatches,
    marker_mismatches,
    read_decisions,
    read_markers,
    recording_errors,
)

pytestmark = pytest.mark.unit

STATUS = {"captures_seen": 3, "poses_used": 7, "released": 3}


def decision(capture_id, usable=True, reason="ok", s=1.0, release="covered"):
    return {"capture_id": capture_id, "usable": usable, "reason": reason, "s": s,
            "release": release}


def marker(s_m, strength=0.8):
    return {"s_m": s_m, "strength": strength}


def write_metadata(bag, counts):
    bag.mkdir()
    meta = {"rosbag2_bagfile_information": {"version": 5, "topics_with_message_count": [
        {"topic_metadata": {"name": name, "type": "x/msg/Y"}, "message_count": n}
        for name, n in counts.items()]}}
    (bag / "metadata.yaml").write_text(yaml.safe_dump(meta))


def test_bag_message_counts(tmp_path):
    write_metadata(tmp_path / "bag", {"/survey/capture": 3, "/survey/pose_used": 7})
    assert bag_message_counts(tmp_path / "bag") == {"/survey/capture": 3,
                                                    "/survey/pose_used": 7}


def test_read_files(tmp_path):
    lines = [decision(0), decision(1, False, "outside_line", None)]
    (tmp_path / "d.jsonl").write_text("".join(json.dumps(d) + "\n" for d in lines))
    assert read_decisions(tmp_path / "d.jsonl") == lines
    assert read_decisions(tmp_path / "missing.jsonl") == []
    (tmp_path / "markers.json").write_text(json.dumps({"markers": [marker(2.0)]}))
    assert read_markers(tmp_path / "markers.json") == [marker(2.0)]


def test_recording_errors():
    decisions = [decision(k) for k in range(3)]
    counts = {"/survey/capture": 3, "/survey/pose_used": 7}
    assert recording_errors(counts, STATUS, decisions) == []
    errors = recording_errors({"/survey/capture": 2}, STATUS, decisions[:2])
    assert len(errors) == 3
    assert "bag /survey/capture 2 messages, live status captures_seen 3" in errors[0]
    assert "/survey/pose_used 0" in errors[1]
    assert "decisions.jsonl 2 lines" in errors[2]


def test_decisions_compared_per_live_capture_id():
    live = [decision(0), decision(1, False, "outside_line"), decision(2)]
    # Order and extra replayed captures do not matter; only the live ids are compared.
    assert decision_mismatches(live, [decision(2), decision(1, False, "outside_line"),
                                      decision(0), decision(9)]) == []
    out = decision_mismatches(live, [decision(0), decision(1, False, "no_pose")])
    assert out == ["capture 1: live outside_line (unusable), replay no_pose (unusable)",
                   "capture 2: not released in the replay"]


def test_decision_mismatches_are_capped():
    live = [decision(k) for k in range(15)]
    out = decision_mismatches(live, [], limit=10)
    assert len(out) == 11 and out[-1] == "... 5 more"


def test_markers_paired_in_s_order():
    live = [marker(6.0), marker(3.0)]
    assert marker_mismatches(live, [marker(3.009), marker(5.995)], 0.01) == []
    out = marker_mismatches(live, [marker(3.0), marker(6.02)], 0.01)
    assert out == ["marker 1: live at 6.000 m, replay at 6.020 m"]


def test_marker_count_differs():
    out = marker_mismatches([marker(1.0)], [], 0.01)
    assert out == ["1 live markers at s = [1.0], 0 replayed at s = []"]


# The start of the comparison (takeoff) from the recorded raw captures


def cdr_string(buf: bytearray, text: str) -> None:
    buf.extend(b"\0" * ((-(len(buf) - 4)) % 4))
    data = text.encode() + b"\0"
    buf.extend(struct.pack("<I", len(data)) + data)


def capture_cdr(capture_id: int, stamp_us: int, frame_id: str = "survey0",
                  sensor_id: str = "survey0") -> bytes:
    """The leading fields of a CDR SensorCapture, then a float64 sequence (as ROS writes it)."""
    buf = bytearray(b"\x00\x01\x00\x00")
    buf.extend(struct.pack("<iI", stamp_us // 1_000_000, (stamp_us % 1_000_000) * 1000))
    cdr_string(buf, frame_id)
    cdr_string(buf, sensor_id)
    buf.extend(b"\0" * ((-(len(buf) - 4)) % 8))
    buf.extend(struct.pack("<Q", capture_id))
    buf.extend(struct.pack("<I", 2) + b"\0" * 4 + struct.pack("<2d", 1e9, 2e9))
    return bytes(buf)


@pytest.mark.parametrize("frame_id", ["", "a", "survey0", "sensor_123"])
def test_capture_stamp_across_string_paddings(frame_id):
    assert capture_stamp(capture_cdr(276, 41_940_693, frame_id, frame_id[:3])) == \
        (276, 41_940_693)


def test_capture_stamps_from_a_sqlite_bag_and_the_compared_set(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(yaml.safe_dump(
        {"rosbag2_bagfile_information": {"relative_file_paths": ["bag_0.db3"]}}))
    con = sqlite3.connect(bag / "bag_0.db3")
    con.executescript("CREATE TABLE topics (id INTEGER, name TEXT);"
                      "CREATE TABLE messages (topic_id INTEGER, timestamp INTEGER, data BLOB);"
                      "INSERT INTO topics VALUES (1, '/survey/capture'), (2, '/survey/pose_used');")
    stamps = {0: 28_140_693, 1: 28_190_693, 2: 53_428_000, 3: 53_478_100}
    con.executemany("INSERT INTO messages VALUES (1, 0, ?)",
                    [(capture_cdr(k, t),) for k, t in stamps.items()])
    con.execute("INSERT INTO messages VALUES (2, 0, ?)", (b"\x00\x01\x00\x00" + bytes(40),))
    con.commit()
    con.close()
    assert capture_stamps_us(bag) == stamps
    assert compared_captures(stamps, 53_428_000) == {2, 3}


def test_only_the_compared_captures_are_checked():
    live = [decision(0, False, "no_pose", release="timeout"), decision(1), decision(2)]
    replay = [decision(0, False, "outside_line"), decision(1), decision(2, False, "no_pose")]
    assert decision_mismatches(live, replay, compared={1}) == []
    assert decision_mismatches(live, replay, compared={1, 2}) == [
        "capture 2: live ok (usable), replay no_pose (unusable)"]
    assert len(decision_mismatches(live, replay)) == 2
