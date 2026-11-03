"""Write a Markdown summary of every JUnit file under a directory.

Usage: python scripts/summarize.py <dir>  (output goes to stdout). JUnit files
are named <suite>-<run>.xml by the workflow. Pass/fail is decided by the sim
jobs; this script only reports and exits 0 whenever it could write the summary.
"""

import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


def read_results(root: Path) -> list[dict]:
    results = []
    for path in sorted(root.rglob("*.xml")):
        suite, _, run = path.stem.rpartition("-")
        if not suite:
            suite, run = path.stem, "1"
        for case in ET.parse(path).iter("testcase"):
            if case.find("skipped") is not None:
                outcome = "skipped"
            elif case.find("failure") is not None or case.find("error") is not None:
                outcome = "failed"
            else:
                outcome = "passed"
            results.append({
                "test": case.get("name"),
                "suite": suite,
                "run": run,
                "outcome": outcome,
                "time_s": float(case.get("time") or 0),
                "properties": {p.get("name"): p.get("value") for p in case.iter("property")},
            })
    return results


def render(results: list[dict]) -> str:
    if not results:
        return "## Simulation test results\n\nNo JUnit results found.\n"
    icon = {"passed": "✅", "failed": "❌", "skipped": "⏭"}
    lines = ["## Simulation test results", "",
             "| Test | Suite | Run | Result | Wall time (s) | Metrics |",
             "|---|---|---|---|---|---|"]
    for r in results:
        metrics = "<br>".join(f"{k}: {v}" for k, v in r["properties"].items())
        lines.append(f"| `{r['test']}` | {r['suite']} | {r['run']} | {icon[r['outcome']]} "
                     f"{r['outcome']} | {r['time_s']:.1f} | {metrics} |")

    runs = defaultdict(list)
    for r in results:
        if r["outcome"] != "skipped":
            runs[(r["suite"], r["test"])].append(r["outcome"] == "passed")
    if any(len(v) > 1 for v in runs.values()):
        lines += ["", "### Pass rate", "", "| Test | Suite | Passed | Rate |", "|---|---|---|---|"]
        for (suite, test), passes in sorted(runs.items()):
            rate = sum(passes) / len(passes)
            warn = " ⚠" if rate < 1 else ""
            lines.append(f"| `{test}` | {suite} | {sum(passes)}/{len(passes)} | "
                         f"{rate:.0%}{warn} |")
    return "\n".join(lines) + "\n"


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    sys.stdout.write(render(read_results(Path(argv[1]))))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
