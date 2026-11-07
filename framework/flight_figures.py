"""Figures of a flight from its PX4 log (ULog) and, for the line survey, the node's recordings.

Two figures, both drawn from files a CI run keeps in its ``results-*`` artifact:

*   ``failsafe_figure``: the track seen from above and the height over time, shaded by flight
    mode, for one failsafe test;
*   ``survey_figure``: the captures along the line against the painted and the found markers, the
    height over the line against its limits, and when the range finder joined the height estimate.

The helpers that do not draw are plain functions and unit tested; matplotlib, numpy and pyulog are
imported when a figure is drawn (``pip install -e ".[plots]"``).
"""

from __future__ import annotations

import json
from pathlib import Path

# PX4 v1.18.0-rc1, msg/versioned/VehicleStatus.msg, NAVIGATION_STATE_*.
NAV_STATE_NAMES = {
    0: "Manual", 2: "Position", 3: "Mission", 4: "Hold", 5: "Return", 12: "Descend",
    14: "Offboard", 17: "Takeoff", 18: "Land",
}
NAV_STATE_COLORS = {
    "Mission": "#3b7dd8", "Hold": "#d9a21b", "Return": "#d1495b", "Land": "#6b8e23",
    "Takeoff": "#7f7f7f", "Offboard": "#8e5ea2", "Position": "#4fa3a5",
}
LIFTOFF_HEIGHT_M = 0.05


def mode_name(nav_state: int) -> str:
    return NAV_STATE_NAMES.get(int(nav_state), f"state {int(nav_state)}")


def mode_segments(times_s, nav_states) -> list[tuple[float, float, str]]:
    """``(start_s, end_s, mode)`` for each stretch of one navigation state; the last one ends at
    the last time."""
    segments: list[tuple[float, float, str]] = []
    if len(times_s) == 0:
        return segments
    start, current = float(times_s[0]), int(nav_states[0])
    for t, state in zip(times_s[1:], nav_states[1:]):
        if int(state) != current:
            segments.append((start, float(t), mode_name(current)))
            start, current = float(t), int(state)
    segments.append((start, float(times_s[-1]), mode_name(current)))
    return segments


def along_line_m(north_m, east_m, line: dict) -> list[float]:
    """Distance along the survey line for vehicle positions (north, east from home).

    ``line``: ``start_north_m``, ``start_east_m``, ``heading_deg`` as in the scenario file.
    """
    import math

    heading = math.radians(float(line.get("heading_deg", 0.0)))
    n0, e0 = float(line["start_north_m"]), float(line.get("start_east_m", 0.0))
    return [(n - n0) * math.cos(heading) + (e - e0) * math.sin(heading)
            for n, e in zip(north_m, east_m)]


def reason_counts(decisions: list[dict]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for decision in decisions:
        counts[decision["reason"]] = counts.get(decision["reason"], 0) + 1
    return counts


def read_decisions(path: Path) -> list[dict]:
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def _load(ulg: Path, topics: list[str]) -> dict:
    from pyulog import ULog

    data = {d.name: d.data for d in ULog(str(ulg), topics).data_list}
    missing = [t for t in topics if t not in data]
    if missing:
        raise SystemExit(f"{Path(ulg).name} has no {', '.join(missing)}")
    return data


def failsafe_figure(ulg: Path, out: Path, title: str, source: str = "") -> dict:
    """Track from above and height over time for one failsafe flight; returns the figures shown."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.collections import LineCollection

    data = _load(ulg, ["vehicle_local_position_groundtruth", "vehicle_status"])
    truth, status = data["vehicle_local_position_groundtruth"], data["vehicle_status"]
    t_truth = truth["timestamp"] / 1e6
    t_status = status["timestamp"] / 1e6
    nav = status["nav_state"]
    in_air = np.flatnonzero(-truth["z"] > LIFTOFF_HEIGHT_M)
    t_start = float(t_truth[in_air[0]]) if len(in_air) else float(t_truth[0])
    t_end = float(t_truth[in_air[-1]]) if len(in_air) else float(t_truth[-1])
    keep = (t_truth >= t_start - 2.0) & (t_truth <= t_end + 2.0)
    t, north, east, height = t_truth[keep] - t_start, truth["x"][keep], truth["y"][keep], -truth["z"][keep]
    nav_at = nav[np.clip(np.searchsorted(t_status, t_truth[keep]) - 1, 0, len(nav) - 1)]
    segments = mode_segments(t_status[(t_status >= t_start - 2.0) & (t_status <= t_end + 2.0)] - t_start,
                             nav[(t_status >= t_start - 2.0) & (t_status <= t_end + 2.0)])

    fig, (ax_map, ax_alt) = plt.subplots(1, 2, figsize=(11, 4.4), gridspec_kw={"width_ratios": [1, 1.5]})
    points = np.column_stack([east, north])
    pieces = np.stack([points[:-1], points[1:]], axis=1)
    colors = [NAV_STATE_COLORS.get(mode_name(s), "#999999") for s in nav_at[:-1]]
    ax_map.add_collection(LineCollection(pieces, colors=colors, linewidths=2.2))
    ax_map.plot(0, 0, marker="s", color="black", markersize=7, label="home")
    ax_map.plot(east[-1], north[-1], marker="X", color="black", markersize=9, label="landing")
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.autoscale()
    ax_map.set_xlabel("east of home (m)")
    ax_map.set_ylabel("north of home (m)")
    ax_map.set_title("Track seen from above (simulator truth)")
    ax_map.legend(loc="best", fontsize=8)
    ax_map.grid(alpha=0.3)

    ax_alt.plot(t, height, color="black", linewidth=1.6)
    seen = set()
    for start, end, mode in segments:
        label = mode if mode not in seen else None
        seen.add(mode)
        ax_alt.axvspan(start, end, color=NAV_STATE_COLORS.get(mode, "#999999"), alpha=0.18, label=label)
    ax_alt.set_xlabel("seconds since lift-off")
    ax_alt.set_ylabel("height (m)")
    ax_alt.set_title("Height and flight mode")
    ax_alt.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17), fontsize=8, ncol=4, frameon=False)
    ax_alt.grid(alpha=0.3)
    ax_alt.set_xlim(t[0], t[-1])

    landing_m = float(np.hypot(north[-1], east[-1]))
    reaction = next((s for s in segments if s[2] not in ("Mission", "Takeoff", "Offboard")), None)
    note = f"landed {landing_m:.1f} m from home"
    if reaction:
        ax_alt.axvline(reaction[0], color="black", linestyle="--", linewidth=1)
        note = f"{reaction[2]} {reaction[0]:.0f} s after lift-off; " + note
    fig.suptitle(title, fontsize=12)
    fig.text(0.01, 0.01, (note + ("   |   " + source if source else "")), fontsize=8, color="#444444")
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return {"landing_distance_m": landing_m, "reaction": reaction, "segments": segments}


def survey_figure(ulg: Path, decisions: Path, markers: Path, scenario: dict, out: Path, title: str,
                  source: str = "", rng_fusion_limit_s: float = 3.0) -> dict:
    """Captures against markers, height over the line, and the range-finder fusion of one flight."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    line = scenario["line"]
    length = float(line["length_m"])
    height_target = float(scenario["sensor_height_m"]) + float(scenario.get("sensor_offset_m", {}).get("z", 0.0))
    band = 0.15
    data = _load(ulg, ["vehicle_local_position_groundtruth", "estimator_aid_src_rng_hgt"])
    truth, aid = data["vehicle_local_position_groundtruth"], data["estimator_aid_src_rng_hgt"]
    t_truth = truth["timestamp"] / 1e6
    height = -truth["z"]
    s_truth = np.array(along_line_m(truth["x"], truth["y"], line))
    in_air = np.flatnonzero(height > LIFTOFF_HEIGHT_M)
    liftoff = float(t_truth[in_air[0]])
    airborne = (t_truth >= liftoff) & (height > LIFTOFF_HEIGHT_M)
    last_air = float(t_truth[in_air[-1]])

    rows = read_decisions(decisions)
    found = json.loads(Path(markers).read_text())
    found = found if isinstance(found, list) else found.get("markers", [])
    painted = scenario.get("markers", [])
    counts = reason_counts(rows)

    fig, (ax_cap, ax_h, ax_f) = plt.subplots(3, 1, figsize=(10, 9.2),
                                             gridspec_kw={"height_ratios": [1.1, 1.3, 0.9]})
    ok = [r["s"] for r in rows if r["reason"] == "ok" and r["s"] is not None]
    out_of_line = [r["s"] for r in rows if r["reason"] != "ok" and r["s"] is not None]
    ax_cap.scatter(out_of_line, [0.0] * len(out_of_line), s=6, color="#b5b5b5", label="not usable")
    ax_cap.scatter(ok, [0.0] * len(ok), s=6, color="#2e8b57", label=f"usable ({len(ok)})")
    for k, m in enumerate(painted):
        ax_cap.axvline(m["s_m"], color="#d1495b", linestyle="--", linewidth=1.2, label="painted marker" if k == 0 else None)
    errors = []
    for k, m in enumerate(found):
        ax_cap.plot(m["s_m"], 0.6, marker="v", color="#3b7dd8", markersize=9, label="marker found" if k == 0 else None)
        if painted:
            nearest = min(painted, key=lambda p: abs(p["s_m"] - m["s_m"]))
            errors.append(m["s_m"] - nearest["s_m"])
            ax_cap.annotate(f"{errors[-1] * 100:+.1f} cm", (m["s_m"], 0.6), textcoords="offset points",
                            xytext=(0, 9), ha="center", fontsize=9, color="#3b7dd8")
    ax_cap.set_xlim(-1, length + 1)
    ax_cap.set_ylim(-0.4, 1.45)
    ax_cap.set_yticks([])
    ax_cap.set_xlabel("position along the line (m)")
    ax_cap.set_title("Captures, painted markers and the markers the detector found (position error in cm)")
    ax_cap.legend(loc="upper center", fontsize=8, ncol=4, frameon=False)

    ax_h.axhspan(height_target - band, height_target + band, color="#2e8b57", alpha=0.15,
                 label=f"limit {height_target:.2f} ± {band:.2f} m")
    ax_h.plot(s_truth[airborne], height[airborne], color="#999999", linewidth=1.0, label="whole flight")
    on_line = airborne & (s_truth >= 0) & (s_truth <= length)
    ax_h.plot(s_truth[on_line], height[on_line], color="black", linewidth=1.8, label="over the line")
    ax_h.set_xlim(-1, length + 1)
    ax_h.set_xlabel("position along the line (m)")
    ax_h.set_ylabel("height (m)")
    ax_h.legend(loc="lower right", fontsize=8)
    ax_h.grid(alpha=0.3)
    worst = float(np.max(np.abs(height[on_line] - height_target))) if on_line.any() else float("nan")
    ax_h.set_title(f"Height over the line (simulator truth); largest deviation from {height_target:.2f} m: "
                   f"{worst:.2f} m")

    t_aid = aid["timestamp"] / 1e6 - liftoff
    fused_idx = np.flatnonzero((t_aid >= 0) & aid["fused"].astype(bool))
    delay = float(t_aid[fused_idx[0]]) if len(fused_idx) else float("nan")
    ax_f.step(t_aid, aid["fused"].astype(int), where="post", color="black")
    ax_f.axvline(rng_fusion_limit_s, color="#d1495b", linestyle="--", linewidth=1.2, label=f"limit {rng_fusion_limit_s:.0f} s")
    if len(fused_idx):
        ax_f.axvline(delay, color="#2e8b57", linewidth=1.5, label=f"range finder fused after {delay:.2f} s")
    ax_f.set_xlim(-1, min(last_air - liftoff, 12.0))
    ax_f.set_yticks([0, 1])
    ax_f.set_yticklabels(["not fused", "fused"])
    ax_f.set_xlabel("seconds since lift-off")
    ax_f.set_title("When the range finder joined the height estimate (from the PX4 log)")
    ax_f.legend(loc="center right", fontsize=8)
    ax_f.grid(alpha=0.3)

    reasons = ", ".join(f"{k} {v}" for k, v in counts.items())
    fig.suptitle(title, fontsize=12)
    fig.text(0.01, 0.005, f"captures: {reasons}   |   " + source, fontsize=8, color="#444444")
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return {"fusion_delay_s": delay, "worst_height_error_m": worst, "reasons": counts,
            "markers_found": len(found)}
