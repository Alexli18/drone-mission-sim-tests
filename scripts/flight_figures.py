#!/usr/bin/env python3
"""Draw the flight figures of docs/evidence.md from a CI run's artifacts.

    scripts/flight_figures.py failsafe ULOG OUT.png --title TEXT [--source TEXT]
    scripts/flight_figures.py survey RESULTS_DIR SCENARIO OUT.png --title TEXT [--source TEXT]

``RESULTS_DIR`` is an unpacked ``results-ros-survey-<n>`` artifact. Needs ``pip install -e ".[plots]"``.
"""

import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from framework import flight_figures


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest="kind", required=True)
    fs = sub.add_parser("failsafe")
    fs.add_argument("ulog", type=Path)
    fs.add_argument("out", type=Path)
    sv = sub.add_parser("survey")
    sv.add_argument("results", type=Path)
    sv.add_argument("scenario", help="scenario name, e.g. line_survey_markers")
    sv.add_argument("out", type=Path)
    for p in (fs, sv):
        p.add_argument("--title", required=True)
        p.add_argument("--source", default="")
    args = parser.parse_args()

    if args.kind == "failsafe":
        figures = flight_figures.failsafe_figure(args.ulog, args.out, args.title, args.source)
    else:
        test_dir = f"test_line_survey_{args.scenario}_"
        ulogs = sorted((args.results / "ulog" / test_dir).glob("px4/rootfs/log/*/*.ulg"),
                       key=lambda p: p.stat().st_size)
        if not ulogs:
            sys.exit(f"no ULog under {args.results}/ulog/{test_dir}")
        live = args.results / "ros" / test_dir / "live"
        scenario = yaml.safe_load((Path(__file__).resolve().parents[1] / "scenarios"
                                   / f"{args.scenario}.yaml").read_text())
        figures = flight_figures.survey_figure(ulogs[-1], live / "decisions.jsonl", live / "markers.json",
                                               scenario, args.out, args.title, args.source)
    print(figures)


if __name__ == "__main__":
    main()
