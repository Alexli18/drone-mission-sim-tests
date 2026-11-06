"""Unit tests for the failsafe pass rule (no simulator), including CUJ 3, and the line
helpers."""

import math
from types import SimpleNamespace

import pytest

from framework.assertions import (
    FlightAssertionError,
    Moment,
    Reference,
    assert_cross_track,
    failsafe_verdict,
    latlon_to_offset,
    line_speed,
    line_window,
)
from framework.mission import Expected, SurveyLine, offset_to_latlon

pytestmark = pytest.mark.unit

START = Moment(10.0, 100.0)
TRIGGER = Moment(40.0, 110.0)
REF = Reference(start=START, trigger=TRIGGER)


class FakeRecorder:
    """Mode changes as (wall_s, mode); latest values for armed, landed, progress."""

    def __init__(self, modes, armed=True, landed="IN_AIR", progress=(3, 22)):
        self.cfg = SimpleNamespace(min_effective_speed_factor=5.0)
        self._modes = [(None, wall, mode) for wall, mode in modes]
        self._latest = {
            "armed": armed,
            "landed_state": SimpleNamespace(name=landed),
            "mission_progress": SimpleNamespace(current=progress[0], total=progress[1]),
        }

    def mode_history(self):
        return self._modes

    def latest(self, stream):
        return self._latest.get(stream)

    def effective_speed_factor(self):
        return None

    def snapshot(self):
        return {}


def expect(**kw):
    base = {"mode_reached": "RETURN_TO_LAUNCH", "leave_mode": None, "allowed_before": ("HOLD",),
            "then_mode": None, "landed": True, "disarmed": True, "max_dist_from_home_m": 5.0}
    base.update(kw)
    return Expected(**base)


LANDED = {"armed": False, "landed": "ON_GROUND"}


def verdict(modes, expected=None, **state):
    return failsafe_verdict(FakeRecorder(modes, **state), expected or expect(), REF, "MISSION")


def test_hold_then_return_then_landed_passes():
    assert verdict([(100, "MISSION"), (120, "HOLD"), (125, "RETURN_TO_LAUNCH")], **LANDED)


def test_waits_while_return_is_still_flying():
    assert verdict([(100, "MISSION"), (125, "RETURN_TO_LAUNCH")]) is None


def test_cuj3_land_instead_of_return_fails():
    with pytest.raises(FlightAssertionError, match="first failsafe mode RETURN_TO_LAUNCH"):
        verdict([(100, "MISSION"), (120, "HOLD"), (125, "LAND")])


def test_failsafe_before_the_fault_fails():
    with pytest.raises(FlightAssertionError, match="failsafe before fault"):
        verdict([(100, "MISSION"), (105, "HOLD"), (108, "MISSION")])


def test_mission_finished_without_failsafe_fails():
    with pytest.raises(FlightAssertionError, match="failsafe never fired"):
        verdict([(100, "MISSION"), (150, "HOLD")], progress=(22, 22))


def test_no_mode_change_yet_keeps_waiting():
    assert verdict([(100, "MISSION")]) is None


def test_then_mode_must_follow_before_disarm():
    expected = expect(then_mode="LAND", max_dist_from_home_m=None)
    modes = [(100, "MISSION"), (120, "HOLD"), (125, "RETURN_TO_LAUNCH")]
    with pytest.raises(FlightAssertionError, match="LAND after RETURN_TO_LAUNCH"):
        verdict(modes, expected, **LANDED)
    assert verdict(modes + [(140, "LAND")], expected, **LANDED)


def test_leave_mode_accepts_any_next_mode():
    expected = expect(mode_reached=None, leave_mode="MISSION", allowed_before=())
    assert verdict([(100, "MISSION"), (115, "LAND")], expected, **LANDED)


def test_mission_after_disarm_is_ignored():
    # PX4 reports MISSION again after disarming (seen in spikes S6-S8).
    modes = [(100, "MISSION"), (120, "HOLD"), (125, "RETURN_TO_LAUNCH"), (160, "MISSION")]
    assert verdict(modes, **LANDED)


def test_not_landed_keeps_waiting():
    assert verdict([(100, "MISSION"), (125, "RETURN_TO_LAUNCH")], armed=False) is None


def test_offboard_flow_uses_offboard_as_active_mode():
    rec = FakeRecorder([(110, "OFFBOARD"), (130, "HOLD"), (135, "RETURN_TO_LAUNCH")], **LANDED)
    ref = Reference(start=TRIGGER, trigger=TRIGGER)
    assert failsafe_verdict(rec, expect(), ref, "OFFBOARD")


HOME_LAT, HOME_LON = 47.397742, 8.545594


def test_latlon_to_offset_inverts_offset_to_latlon_within_0_1_m():
    for north, east in [(0, 0), (5, 0), (50, 70), (-20, 130), (110, -110)]:
        lat, lon = offset_to_latlon(HOME_LAT, HOME_LON, north, east)
        back = latlon_to_offset(HOME_LAT, HOME_LON, lat, lon)
        assert math.dist(back, (north, east)) < 0.1


class TrackRecorder(FakeRecorder):
    """Positions as (sim_s, lat, lon, relative altitude) from (sim_s, north, east) offsets."""

    def __init__(self, track):
        super().__init__([])
        self._positions = [(t, *offset_to_latlon(HOME_LAT, HOME_LON, n, e), 0.4)
                           for t, n, e in track]

    def positions(self):
        return self._positions


LINE = SurveyLine(start_north_m=5, start_east_m=0, heading_deg=0, length_m=20)


def synthetic_track():
    """Hover at home, fly north at 0.5 m/s from s = -1.75 m to s = 22.25 m, weaving by 0.1 m.

    No sample lies on s = 0 or s = L, where the lat/lon round trip could put it either side.
    """
    track = [(None, 0.0, 0.0), (1.0, 0.0, 0.0)]  # the first sample has no simulated time yet
    for i in range(49):
        t = 10.0 + i
        track.append((t, 3.25 + 0.5 * i, 0.1 * math.sin(i)))
    return track


def test_line_window_is_first_sample_at_s0_to_last_at_length():
    t_in, t_out = line_window(TrackRecorder(synthetic_track()), LINE, HOME_LAT, HOME_LON)
    assert (t_in, t_out) == (14.0, 53.0)  # north 5.25 m and 24.75 m


def test_assert_cross_track_returns_max_and_fails_beyond_tolerance():
    rec = TrackRecorder(synthetic_track())
    worst = assert_cross_track(rec, LINE, HOME_LAT, HOME_LON, 0.3)
    assert worst == pytest.approx(0.1, abs=0.01)
    with pytest.raises(FlightAssertionError, match="cross-track within ± 0.05 m"):
        assert_cross_track(rec, LINE, HOME_LAT, HOME_LON, 0.05)


def test_line_speed_in_simulated_time():
    speed = line_speed(TrackRecorder(synthetic_track()), LINE, HOME_LAT, HOME_LON)
    assert speed == pytest.approx(0.5, abs=0.01)


def test_line_window_without_the_line_fails():
    rec = TrackRecorder([(1.0, 0.0, 0.0), (2.0, 1.0, 0.0)])
    with pytest.raises(FlightAssertionError, match="no such window"):
        line_window(rec, LINE, HOME_LAT, HOME_LON)
