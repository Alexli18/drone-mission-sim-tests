"""Unit tests for scripts/summarize.py (no simulator)."""

import importlib.util

import pytest

from framework.config import REPO_ROOT

pytestmark = pytest.mark.unit

_spec = importlib.util.spec_from_file_location("summarize", REPO_ROOT / "scripts" / "summarize.py")
summarize = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(summarize)


def junit(cases: str) -> str:
    return f'<?xml version="1.0"?><testsuites><testsuite name="pytest">{cases}</testsuite></testsuites>'


PASS = ('<testcase name="test_failsafe[rc_loss]" time="20.5"><properties>'
        '<property name="landing_distance_from_home_m" value="0.5"/></properties></testcase>')
FAIL = '<testcase name="test_failsafe[rc_loss]" time="30.0"><failure message="x"/></testcase>'
SKIP = '<testcase name="test_failsafe[link_loss]" time="1.0"><skipped message="Linux"/></testcase>'


def test_single_run_has_no_pass_rate_table(tmp_path):
    (tmp_path / "failsafes-1.xml").write_text(junit(PASS + SKIP))
    text = summarize.render(summarize.read_results(tmp_path))
    assert "| `test_failsafe[rc_loss]` | failsafes | 1 | ✅ passed | 20.5 |" in text
    assert "landing_distance_from_home_m: 0.5" in text
    assert "⏭ skipped" in text
    assert "Pass rate" not in text


def test_nightly_runs_give_pass_rate_with_warning(tmp_path):
    for run, case in ((1, PASS), (2, PASS), (3, FAIL)):
        sub = tmp_path / f"results-failsafes-{run}" / "junit"
        sub.mkdir(parents=True)
        (sub / f"failsafes-{run}.xml").write_text(junit(case))
    text = summarize.render(summarize.read_results(tmp_path))
    assert "| `test_failsafe[rc_loss]` | failsafes | 2/3 | 67% ⚠ |" in text


def test_no_results_still_writes_a_summary(tmp_path):
    assert "No JUnit results found" in summarize.render(summarize.read_results(tmp_path))
