"""Unit tests for the ready gate's conditions (survey_demo.gate; no ROS)."""

import pytest
from survey_demo.gate import missing_items, parse_topic_minimum

pytestmark = pytest.mark.unit


def test_parse_topic_minimum():
    assert parse_topic_minimum("/survey/capture:2") == ("/survey/capture", 2)
    for bad in ("/survey/capture", ":2", "/x:two"):
        with pytest.raises(ValueError):
            parse_topic_minimum(bad)


def gate(flags_received, counts=None, names=("toy_sensor_node", "georeference_node")):
    counts = counts or {"/survey/capture": 2}
    return missing_items(set(names), {"/survey/stop"}, lambda t: counts.get(t, 0),
                         ["toy_sensor_node", "georeference_node"], ["/survey/stop"],
                         [("/survey/capture", 2)], ["/sim/pose_received", "/survey/pose_received"],
                         flags_received)


def test_waits_until_both_nodes_have_a_pose():
    assert gate(set()) == ["message on /sim/pose_received", "message on /survey/pose_received"]
    assert gate({"/sim/pose_received"}) == ["message on /survey/pose_received"]
    assert gate({"/sim/pose_received", "/survey/pose_received"}) == []


def test_lists_every_missing_item():
    both = {"/sim/pose_received", "/survey/pose_received"}
    assert gate(both, counts={"/survey/capture": 1}, names=("toy_sensor_node",)) == [
        "node georeference_node", "topic /survey/capture (1/2 subscriptions)"]
