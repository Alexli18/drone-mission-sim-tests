"""Unit tests for framework.config (no simulator)."""

import subprocess
import sys

import pytest
import yaml

from framework.config import CONFIG_PATH, REPO_ROOT, ConfigError, load_baseline_params, load_config

pytestmark = pytest.mark.unit


def test_config_loads():
    cfg = load_config()
    assert cfg.px4.speed_factor > 0
    assert cfg.mavlink.api_port == 14540
    assert cfg.min_effective_speed_factor == cfg.min_effective_speed_ratio * cfg.px4.speed_factor


def test_missing_key_is_named(tmp_path):
    data = yaml.safe_load(CONFIG_PATH.read_text())
    del data["timeouts_wall_s"]["armable"]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ConfigError, match=r"timeouts_wall_s\.armable"):
        load_config(path)


def test_unknown_key_is_named(tmp_path):
    data = yaml.safe_load(CONFIG_PATH.read_text())
    data["px4"]["colour"] = "blue"
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ConfigError, match=r"px4\.colour"):
        load_config(path)


@pytest.mark.parametrize("command", ["image", "digest", "local-tag", "ros-base-tag", "ros-tag",
                                     "ros-domain"])
def test_cli_prints_pin(command):
    cfg = load_config()
    expected = {
        "image": cfg.px4.image_ref,
        "digest": cfg.px4.digest,
        "local-tag": cfg.px4.local_tag,
        "ros-base-tag": cfg.ros.base_image_tag,
        "ros-tag": cfg.ros.image_tag,
        "ros-domain": str(cfg.ros.domain_id),
    }[command]
    out = subprocess.run([sys.executable, "-m", "framework.config", command], cwd=REPO_ROOT,
                         capture_output=True, text=True, check=True).stdout.strip()
    assert out == expected


def test_local_tag_is_derived_from_digest():
    cfg = load_config()
    assert cfg.px4.local_tag == "px4-sitl-patched:" + cfg.px4.digest.removeprefix("sha256:")[:12]


def test_baseline_params_load():
    params = load_baseline_params()
    assert params["COM_FAIL_ACT_T"] == 5
    assert "COM_RC_OVERRIDE" not in params  # not in PX4 v1.18.0-rc1 (spike S1)


def test_ros_runs_use_a_lower_speed_factor():
    px4 = load_config().px4
    assert 0 < px4.speed_factor_ros < px4.speed_factor


def test_survey_section_is_complete_and_strict(tmp_path):
    data = yaml.safe_load(CONFIG_PATH.read_text())
    assert load_config().survey.pass_rule.coverage_min == 0.95
    del data["survey"]["detector"]["threshold"]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ConfigError, match=r"survey\.detector\.threshold"):
        load_config(path)
