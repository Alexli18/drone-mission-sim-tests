"""Allure identity of the flight tests (framework/reporting.py) and the CI wiring (no simulator)."""

import pytest
import yaml

from framework import reporting
from framework.config import REPO_ROOT, load_config
from framework.mission import load_scenario

pytestmark = pytest.mark.unit

WORKFLOW = REPO_ROOT / ".github" / "workflows" / "sim-tests.yml"
CFG = load_config()
SURVEY = load_scenario("line_survey_markers.yaml")
FAILSAFE = load_scenario("rc_loss.yaml")


def params(scenario, **env):
    return reporting.run_parameters(CFG, scenario, env)


def test_matrix_cells_get_different_parameters():
    cells = [{"SIM_SUITE": "failsafes", "SIM_RUN": "1"}, {"SIM_SUITE": "failsafes", "SIM_RUN": "2"},
             {"SIM_SUITE": "basic", "SIM_RUN": "1"}]
    assert len({tuple(params(FAILSAFE, **env).items()) for env in cells}) == 3


def test_survey_seed_offset_tells_runs_apart_and_other_tests_have_none():
    seeds = [params(SURVEY, SIM_SUITE="ros-survey", SIM_RUN="1", SURVEY_SEED_OFFSET=s)
             for s in ("0", "1")]
    assert seeds[0] != seeds[1]
    assert "seed_offset" not in params(FAILSAFE, SURVEY_SEED_OFFSET="1")


def test_px4_image_comes_from_the_config():
    assert params(FAILSAFE)["px4_image"] == CFG.px4.image
    assert params(None)["scenario"] == "none"


def test_description_matches_a_result_to_the_artifact():
    text = reporting.description("test_failsafe_rc_loss_", {
        "GITHUB_RUN_ID": "42", "SIM_SUITE": "failsafes", "SIM_RUN": "3"})
    assert "results-failsafes-3" in text and "run 42" in text and "test_failsafe_rc_loss_" in text


def workflow_steps(job: str) -> list[dict]:
    return yaml.safe_load(WORKFLOW.read_text())["jobs"][job]["steps"]


@pytest.mark.parametrize("job, prefix", [("sim", "allure-results-${{ matrix.suite }}-"),
                                         ("ros-survey", "allure-results-ros-survey-")])
def test_matrix_job_uploads_a_uniquely_named_allure_artifact(job, prefix):
    names = [s["with"]["name"] for s in workflow_steps(job)
             if str(s.get("uses", "")).startswith("actions/upload-artifact")]
    assert prefix + "${{ matrix.run }}" in names
    assert len(names) == len(set(names))


def test_matrix_jobs_pass_their_cell_to_the_tests():
    jobs = yaml.safe_load(WORKFLOW.read_text())["jobs"]
    assert jobs["sim"]["env"] == {"SIM_SUITE": "${{ matrix.suite }}",
                                  "SIM_RUN": "${{ matrix.run }}"}
    assert jobs["ros-survey"]["env"]["SIM_SUITE"] == "ros-survey"
    assert jobs["ros-survey"]["env"]["SIM_RUN"] == "${{ matrix.run }}"
    for job in ("sim", "ros-survey"):
        pytest_runs = [s["run"] for s in workflow_steps(job) if "pytest" in str(s.get("run"))]
        assert len(pytest_runs) == 1 and "--alluredir=allure-results" in pytest_runs[0]
        assert "--junitxml=" in pytest_runs[0]


def test_allure_job_collects_only_result_artifacts():
    download = next(s for s in workflow_steps("allure")
                    if str(s.get("uses", "")).startswith("actions/download-artifact"))
    assert download["with"]["pattern"] == "allure-results-*"
    assert download["with"]["merge-multiple"] is True
    # The report artifact must not match the pattern (a re-run would merge it back in).
    upload = next(s for s in workflow_steps("allure")
                  if str(s.get("uses", "")).startswith("actions/upload-artifact"))
    assert not upload["with"]["name"].startswith("allure-results-")
