"""Unit tests for scenario loading (no simulator)."""

import pytest
import yaml

from framework.config import load_baseline_params
from framework.mission import SCENARIOS_DIR, ScenarioError, list_scenarios, load_scenario

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


def test_list_scenarios_skips_baseline():
    names = [p.name for p in list_scenarios("hover")]
    assert "hover_and_land.yaml" in names
    assert all(not n.startswith("_") for n in names)
