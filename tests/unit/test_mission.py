"""Unit tests for scenario loading (no simulator)."""

import pytest
import yaml

from framework.assertions import horizontal_distance_m
from framework.config import load_baseline_params
from framework.mission import (
    SCENARIOS_DIR,
    Area,
    ScenarioError,
    lawnmower_waypoints,
    list_scenarios,
    load_scenario,
    offset_to_latlon,
)

pytestmark = pytest.mark.unit


def write(tmp_path, data, name="s.yaml"):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return path


HOVER = {"name": "h", "type": "hover", "description": "d", "altitude_m": 10, "hover_s": 5}


def test_every_scenario_file_loads():
    files = [p for p in SCENARIOS_DIR.glob("*.yaml") if not p.name.startswith("_")]
    assert files
    for path in files:
        load_scenario(path)


def test_scenario_overrides_baseline_and_keeps_the_rest():
    scenario = load_scenario("hover_and_land.yaml")
    baseline = load_baseline_params()
    assert baseline["MIS_TAKEOFF_ALT"] == 15
    assert scenario.px4_params["MIS_TAKEOFF_ALT"] == 10
    for name, value in baseline.items():
        if name != "MIS_TAKEOFF_ALT":
            assert scenario.px4_params[name] == value


def test_missing_field_is_named(tmp_path):
    data = dict(HOVER)
    del data["hover_s"]
    with pytest.raises(ScenarioError, match="missing field: hover_s"):
        load_scenario(write(tmp_path, data))


def test_unknown_key_is_named(tmp_path):
    with pytest.raises(ScenarioError, match="unknown key: hovr_s"):
        load_scenario(write(tmp_path, dict(HOVER, hovr_s=3)))


def test_unknown_type_is_rejected(tmp_path):
    with pytest.raises(ScenarioError, match="unknown type: survey"):
        load_scenario(write(tmp_path, dict(HOVER, type="survey")))


def test_mission_type_requires_area(tmp_path):
    data = dict(HOVER, type="mission", speed_m_s=5, duration_bounds_s={"min_s": 1, "max_s": 2})
    del data["hover_s"]
    with pytest.raises(ScenarioError, match="missing field: area"):
        load_scenario(write(tmp_path, data))


def test_list_scenarios_skips_baseline():
    names = [p.name for p in list_scenarios("hover")]
    assert "hover_and_land.yaml" in names
    assert all(not n.startswith("_") for n in names)


AREA = Area(origin_north_m=10, origin_east_m=10, width_m=60, height_m=40, line_spacing_m=10)


def test_lawnmower_line_count_and_order():
    points = lawnmower_waypoints(AREA)
    assert len(points) == 2 * 7  # ceil(60 / 10) + 1 lines, both ends each
    assert points[0] == (10, 10) and points[1] == (50, 10)   # first line runs north
    assert points[2] == (50, 20) and points[3] == (10, 20)   # second line runs south
    assert points[-1][1] == 70


def test_lawnmower_points_stay_inside_the_area():
    for north, east in lawnmower_waypoints(AREA):
        assert 10 <= north <= 50 and 10 <= east <= 70


def test_offset_round_trip_within_0_1_m():
    home_lat, home_lon = 47.397742, 8.545594
    for north, east in [(0, 0), (50, 70), (-20, 130), (110, 110)]:
        lat, lon = offset_to_latlon(home_lat, home_lon, north, east)
        expected = (north**2 + east**2) ** 0.5
        assert abs(horizontal_distance_m(home_lat, home_lon, lat, lon) - expected) < 0.1


def test_mission_scenario_fields():
    scenario = load_scenario("survey_lawnmower.yaml")
    assert scenario.area == AREA
    assert scenario.duration_bounds_s.min_s < scenario.duration_bounds_s.max_s


def test_bad_area_keys_are_rejected(tmp_path):
    data = yaml.safe_load((SCENARIOS_DIR / "survey_lawnmower.yaml").read_text())
    data["area"]["spacing"] = 5
    with pytest.raises(ScenarioError, match="area must have exactly the keys"):
        load_scenario(write(tmp_path, data))


FAILSAFE = {
    "name": "f", "type": "failsafe", "description": "d", "altitude_m": 15, "speed_m_s": 5,
    "area": {"origin_north_m": 10, "origin_east_m": 10, "width_m": 100, "height_m": 100,
             "line_spacing_m": 10},
    "px4_params": {"SYS_FAILURE_EN": 1, "EKF2_NOAID_TOUT": 5000000, "COM_POS_FS_EPH": 5},
    "failure": {"kind": "gps_loss", "trigger": {"type": "after_s", "value": 60}},
    "expected": {"leave_mode": "MISSION", "landed": True, "disarmed": True},
}


def test_failsafe_scenarios_load_with_their_mechanism():
    kinds = {load_scenario(p).failure.kind for p in list_scenarios("failsafe")}
    assert kinds == {"gps_loss", "link_loss", "rc_loss", "low_battery", "geofence_breach"}


def test_both_mode_reached_and_leave_mode_is_rejected(tmp_path):
    data = dict(FAILSAFE, expected={"leave_mode": "MISSION", "mode_reached": "LAND"})
    with pytest.raises(ScenarioError, match="exactly one of mode_reached and leave_mode"):
        load_scenario(write(tmp_path, data))


def test_unknown_failure_kind_is_rejected(tmp_path):
    with pytest.raises(ScenarioError, match="unknown failure.kind: wind"):
        load_scenario(write(tmp_path, dict(FAILSAFE, failure={"kind": "wind"})))


def test_missing_required_param_is_named(tmp_path):
    data = dict(FAILSAFE, px4_params={"SYS_FAILURE_EN": 1})
    with pytest.raises(ScenarioError, match="lacks COM_POS_FS_EPH, EKF2_NOAID_TOUT"):
        load_scenario(write(tmp_path, data))


def test_geofence_must_contain_home(tmp_path):
    data = yaml.safe_load((SCENARIOS_DIR / "geofence_breach.yaml").read_text())
    data["geofence"]["inclusion_polygon"] = [[10, 10], [30, 10], [30, 30], [10, 30]]
    with pytest.raises(ScenarioError, match="must contain home"):
        load_scenario(write(tmp_path, data))


def test_geofence_only_for_geofence_breach(tmp_path):
    data = dict(FAILSAFE, geofence={"inclusion_polygon": [[-1, -1], [1, -1], [1, 1]]})
    with pytest.raises(ScenarioError, match="only allowed for failure.kind geofence_breach"):
        load_scenario(write(tmp_path, data))
