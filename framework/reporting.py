"""Allure details for the flight tests (tests/conftest.py).

Allure tells two results of one test apart by the test name plus its parameters
(``historyId``). The matrix jobs run the same tests with the same scenario, so the
parameters below carry what differs between them; without them the report would show
one test with "retries". Recorded metrics are attachments, never parameters: a value
that changes from run to run would make every run a new test.
"""

from __future__ import annotations

import json
import os

import allure

SUITE_ENV = "SIM_SUITE"  # matrix suite, set by the workflow (ros-survey for the ROS job)
RUN_ENV = "SIM_RUN"  # matrix run, set by the workflow
SEED_ENV = "SURVEY_SEED_OFFSET"  # read by the ROS fixtures as well
LOCAL = "local"


def run_parameters(cfg, scenario, env=None) -> dict[str, str]:
    """The parameters that make a flight test's identity: where and with what it ran."""
    env = os.environ if env is None else env
    params = {
        "suite": env.get(SUITE_ENV, LOCAL),
        "run": env.get(RUN_ENV, LOCAL),
        "scenario": scenario.name if scenario else "none",
    }
    if scenario is not None and scenario.survey is not None:
        params["seed_offset"] = env.get(SEED_ENV, "0")
    params["px4_image"] = cfg.px4.image
    return params


def description(test_id: str, env=None) -> str:
    """Text that matches an Allure result to the files in the ``results-*`` artifact."""
    env = os.environ if env is None else env
    lines = []
    if env.get("GITHUB_RUN_ID"):
        lines.append(f"Workflow run {env['GITHUB_RUN_ID']} (attempt "
                     f"{env.get('GITHUB_RUN_ATTEMPT', '1')}), job artifact "
                     f"results-{env.get(SUITE_ENV, LOCAL)}-{env.get(RUN_ENV, LOCAL)}.")
    lines.append(f"Test id: {test_id}. Telemetry, logs, ULog and rosbag are in the results "
                 f"artifact under artifacts/*/{test_id}*; they are not attached here.")
    return "\n".join(lines)


def apply_parameters(cfg, scenario) -> None:
    """Set the identity parameters of the running test (during fixture setup)."""
    for name, value in run_parameters(cfg, scenario).items():
        allure.dynamic.parameter(name, value)


def apply_description(test_id: str) -> None:
    """Set the description; allure-pytest overwrites an earlier one at the end of setup."""
    allure.dynamic.description(description(test_id))


def attach_metrics(properties) -> None:
    """The values the test recorded with ``record_property`` (also in the JUnit file)."""
    if properties:
        text = "\n".join(f"{name}: {value}" for name, value in properties)
        allure.attach(text, name="recorded metrics", attachment_type=allure.attachment_type.TEXT)


def attach_failure(recorder) -> None:
    """PX4 mode history and the last telemetry snapshot, as the failure message shows them."""
    text = (f"mode history (sim_s, wall_s, mode):\n{json.dumps(recorder.mode_history())}\n"
            f"last snapshot:\n{json.dumps(recorder.snapshot(), indent=2)}")
    allure.attach(text, name="PX4 modes and last telemetry",
                  attachment_type=allure.attachment_type.TEXT)
