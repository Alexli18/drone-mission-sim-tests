"""Checks that read the PX4 flight log (ULog) after the flight.

The telemetry the tests watch live does not say whether the range finder feeds the height
estimate; the estimator's own record of it does (``estimator_aid_src_rng_hgt.fused``).
docs/qa-notes.md ("Height at the start of the line") explains why the delay matters.

The calculation works on arrays, so it is unit tested without a log; ``load_fusion_inputs``
is the only part that touches pyulog.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

LIFTOFF_HEIGHT_M = 0.05  # simulator truth above the ground: the vehicle has left it
TRUTH_TOPIC = "vehicle_local_position_groundtruth"
AID_TOPIC = "estimator_aid_src_rng_hgt"


class UlogError(Exception):
    """The log is missing or lacks what the check needs."""


def find_ulog(data_dir: Path) -> Path:
    """The flight's log under a SITL data directory (the largest, if there are several)."""
    logs = sorted(data_dir.glob("px4/rootfs/log/*/*.ulg"), key=lambda p: p.stat().st_size)
    if not logs:
        raise UlogError(f"no .ulg file under {data_dir}/px4/rootfs/log")
    return logs[-1]


def load_fusion_inputs(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """``(t_truth_s, height_m, t_aid_s, fused)`` from a ULog, times in seconds of PX4 time."""
    from pyulog import ULog
    from pyulog.exceptions import ULogException

    try:
        ulog = ULog(str(path), [TRUTH_TOPIC, AID_TOPIC])
    except (OSError, ULogException) as exc:
        raise UlogError(f"{path.name} cannot be parsed: {exc}") from exc
    topics = {d.name: d for d in ulog.data_list}
    missing = [t for t in (TRUTH_TOPIC, AID_TOPIC) if t not in topics]
    if missing:
        raise UlogError(f"{path.name} has no {', '.join(missing)}")
    truth, aid = topics[TRUTH_TOPIC].data, topics[AID_TOPIC].data
    return (truth["timestamp"] / 1e6, -truth["z"], aid["timestamp"] / 1e6,
            aid["fused"].astype(bool))


def rng_fusion_delay_s(t_truth_s, height_m, t_aid_s, fused) -> float | None:
    """Seconds from lift-off to the first sample at or after it where the range finder is fused.

    Lift-off is the first truth sample above ``LIFTOFF_HEIGHT_M``. ``None`` when the range
    finder is never fused after that (or there is no lift-off).
    """
    t_truth_s, height_m = np.asarray(t_truth_s), np.asarray(height_m)
    t_aid_s, fused = np.asarray(t_aid_s), np.asarray(fused, dtype=bool)
    above = np.flatnonzero(height_m > LIFTOFF_HEIGHT_M)
    if not len(above):
        return None
    liftoff = float(t_truth_s[above[0]])
    fused_after = np.flatnonzero((t_aid_s >= liftoff) & fused)
    if not len(fused_after):
        return None
    return float(t_aid_s[fused_after[0]]) - liftoff


def rng_fusion_check(data_dir: Path, limit_s: float) -> tuple[float | None, tuple[str, str] | None]:
    """The delay (``None`` if unknown) and the ``(expected, actual)`` failure, if any.

    A log that is missing or unreadable fails the check: a rule that quietly skips would
    pass in exactly the case it was written for.
    """
    expected = (f"the range finder fused into the height estimate within {limit_s} s of "
                "lift-off (PX4 log)")
    try:
        delay = rng_fusion_delay_s(*load_fusion_inputs(find_ulog(data_dir)))
    except UlogError as exc:
        return None, (expected, f"the log could not be read: {exc}")
    if delay is None:
        return None, (expected, "the range finder was never fused after lift-off")
    if delay > limit_s:
        return delay, (expected, f"fused {delay:.2f} s after lift-off")
    return delay, None
