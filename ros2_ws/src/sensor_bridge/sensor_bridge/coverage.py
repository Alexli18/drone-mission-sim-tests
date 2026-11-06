"""Coverage of a line by usable captures (pure; no ROS).

The georeferencing node calls it with its estimated positions; the test computes the
same figure from the simulated sensor's own positions and compares the two.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise


@dataclass(frozen=True)
class Coverage:
    fraction: float
    max_gap_m: float


def coverage(capture_positions_s, usable, length_m: float, ds_m: float,
             gap_factor: float = 1.5) -> Coverage:
    """Fraction of the line not inside a gap above ``gap_factor``·Δs, and the largest gap.

    1. positions of usable captures with 0 ≤ s ≤ L, sorted; 2. virtual points at 0 and L;
    3. gaps between consecutive points; 4. fraction = 1 − Σ(gaps > gap_factor·Δs) / L.
    """
    points = sorted(float(s) for s, ok in zip(capture_positions_s, usable)
                    if ok and not math.isnan(float(s)) and 0.0 <= float(s) <= length_m)
    points = [0.0] + points + [float(length_m)]
    gaps = [b - a for a, b in pairwise(points)]
    limit = gap_factor * ds_m
    holes = sum(g for g in gaps if g > limit)
    return Coverage(fraction=1.0 - holes / length_m, max_gap_m=max(gaps))
