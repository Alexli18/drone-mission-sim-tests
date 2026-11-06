"""scripts/ci_plan.sh decides which CI jobs run; this pins its decisions (no CI needed).

Each test builds a throw-away git repository with a base commit and a change, then runs
the script the way the workflow does.
"""

import os
import subprocess
from pathlib import Path

import pytest

from framework.config import REPO_ROOT

pytestmark = pytest.mark.unit

SCRIPT = REPO_ROOT / "scripts" / "ci_plan.sh"
NIGHTLY, WEEKLY = "17 2 * * *", "41 4 * * 1"


def git(repo: Path, *args: str, env: dict | None = None) -> str:
    return subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@example.invalid", *args],
        cwd=repo, check=True, capture_output=True, text=True, env=env).stdout.strip()


def make_repo(tmp_path: Path, changed: list[str], commit_date: str | None = None):
    """A repo whose base commit is old and whose HEAD changes the given files."""
    git(tmp_path, "init", "-q")
    (tmp_path / "README.txt").write_text("base")
    git(tmp_path, "add", "-A")
    git(tmp_path, "commit", "-q", "-m", "base")
    base = git(tmp_path, "rev-parse", "HEAD")
    for name in changed:
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("changed")
    git(tmp_path, "add", "-A")
    env = None
    if commit_date:
        env = dict(os.environ, GIT_COMMITTER_DATE=commit_date, GIT_AUTHOR_DATE=commit_date)
    git(tmp_path, "commit", "-q", "--allow-empty", "-m", "change", env=env)
    return base


def plan(repo: Path, event: str, base: str = "", schedule: str = "", label: str = "false"):
    out = subprocess.run([str(SCRIPT), event, base, schedule, label], cwd=repo, check=True,
                         capture_output=True, text=True).stdout
    return dict(line.split("=") for line in out.split())


def decided(result):
    return {k: v for k, v in result.items() if v == "true"}.keys()


@pytest.mark.parametrize("changed, expected", [
    (["framework/mission.py"], {"run", "sim", "ros"}),
    (["scenarios/gps_loss.yaml"], {"run", "sim", "ros"}),
    (["scripts/summarize.py"], {"run", "sim", "ros"}),
    (["ros2_ws/src/sensor_bridge/sensor_bridge/pose.py"], {"run", "ros"}),
    (["docker/ros2-survey-base/Dockerfile"], {"run", "ros"}),
    (["docker/px4-sitl/Dockerfile"], {"run", "sim", "ros"}),
    (["tests/test_line_survey.py"], {"run", "ros"}),
    (["tests/test_failsafes.py"], {"run", "sim", "ros"}),
    (["tests/unit/test_config.py"], {"run", "ros"}),
    (["README.txt"], {"run"}),
])
def test_pull_request_runs_only_what_its_paths_touch(tmp_path, changed, expected):
    base = make_repo(tmp_path, changed)
    assert set(decided(plan(tmp_path, "pull_request", base))) == expected


def test_the_full_ci_label_runs_everything_but_not_the_weekly_extras(tmp_path):
    base = make_repo(tmp_path, ["README.txt"])
    result = plan(tmp_path, "pull_request", base, label="true")
    assert set(decided(result)) == {"run", "sim", "ros", "full"}


def test_a_pull_request_without_a_base_is_an_error(tmp_path):
    make_repo(tmp_path, ["README.txt"])
    done = subprocess.run([str(SCRIPT), "pull_request", "", "", "false"], check=False, cwd=tmp_path,
                          capture_output=True, text=True)
    assert done.returncode == 2 and "base SHA" in done.stderr


def test_nightly_runs_everything_when_main_changed_in_24_hours(tmp_path):
    make_repo(tmp_path, ["README.txt"])
    result = plan(tmp_path, "schedule", schedule=NIGHTLY)
    assert set(decided(result)) == {"run", "sim", "ros", "full"}


def test_nightly_skips_a_quiet_night(tmp_path):
    make_repo(tmp_path, ["README.txt"], commit_date="2020-01-01T00:00:00")
    result = plan(tmp_path, "schedule", schedule=NIGHTLY)
    assert set(decided(result)) == set() and result["run"] == "false"


def test_weekly_runs_the_ros_survey_with_the_weekly_scenarios_even_when_quiet(tmp_path):
    make_repo(tmp_path, ["README.txt"], commit_date="2020-01-01T00:00:00")
    result = plan(tmp_path, "schedule", schedule=WEEKLY)
    assert set(decided(result)) == {"run", "ros", "full", "weekly"}


def test_unknown_schedule_is_an_error(tmp_path):
    make_repo(tmp_path, ["README.txt"])
    done = subprocess.run([str(SCRIPT), "schedule", "", "0 0 * * *", "false"], check=False, cwd=tmp_path,
                          capture_output=True, text=True)
    assert done.returncode == 2 and "unknown schedule" in done.stderr


def test_manual_dispatch_runs_everything(tmp_path):
    make_repo(tmp_path, ["README.txt"])
    assert set(decided(plan(tmp_path, "workflow_dispatch"))) == {"run", "sim", "ros", "full"}


def test_the_cron_strings_match_the_workflow():
    text = (REPO_ROOT / ".github" / "workflows" / "sim-tests.yml").read_text()
    assert f'cron: "{NIGHTLY}"' in text and f'cron: "{WEEKLY}"' in text


def test_the_workflow_ignores_documentation_only_changes():
    text = (REPO_ROOT / ".github" / "workflows" / "sim-tests.yml").read_text()
    assert text.count('- "**/*.md"') == 2        # push and pull_request
    assert "cancel-in-progress: ${{ github.event_name == 'pull_request' }}" in text
