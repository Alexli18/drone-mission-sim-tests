"""Unit tests for the failsafe pass rule (no simulator), including CUJ 3."""

from types import SimpleNamespace

import pytest

from framework.assertions import FlightAssertionError, Moment, Reference, failsafe_verdict
from framework.mission import Expected

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
