"""The helpers behind scripts/flight_figures.py that do not draw."""

import pytest

from framework.flight_figures import (
    along_line_m,
    mode_name,
    mode_segments,
    reason_counts,
)

pytestmark = pytest.mark.unit


def test_segments_split_where_the_state_changes():
    times = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    states = [17, 3, 3, 4, 5, 5]
    assert mode_segments(times, states) == [
        (0.0, 1.0, "Takeoff"), (1.0, 3.0, "Mission"), (3.0, 4.0, "Hold"), (4.0, 5.0, "Return")]


def test_one_state_gives_one_segment_and_nothing_gives_none():
    assert mode_segments([0.0, 2.0, 4.0], [3, 3, 3]) == [(0.0, 4.0, "Mission")]
    assert mode_segments([], []) == []


def test_unknown_states_are_named_by_number():
    assert mode_name(18) == "Land" and mode_name(99) == "state 99"


@pytest.mark.parametrize("heading, north, east, expected", [
    (0.0, 8.0, 0.0, 3.0),       # 5 m of line start, 3 m along north
    (0.0, 5.0, 2.0, 0.0),       # sideways offset does not count along the line
    (90.0, 5.0, 4.0, 4.0),      # a line heading east
    (0.0, 3.0, 0.0, -2.0),      # before the line start
])
def test_distance_along_the_line(heading, north, east, expected):
    line = {"start_north_m": 5.0, "start_east_m": 0.0, "heading_deg": heading}
    assert along_line_m([north], [east], line) == pytest.approx([expected])


def test_reason_counts():
    rows = [{"reason": "ok"}, {"reason": "outside_line"}, {"reason": "ok"}]
    assert reason_counts(rows) == {"ok": 2, "outside_line": 1}
