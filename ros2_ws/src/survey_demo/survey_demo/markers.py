"""The toy sensor's ground markers: the reading they cause, and the detector (pure; no ROS).

A marker is a painted target at line-frame position (s, d). The sensor looks straight
down: a reading is the sum of Gaussian footprints of the markers around the sensor,
clipped at full scale (1.0). The detector knows nothing of the markers; it sees only
usable georeferenced captures, ``(s, reading)`` pairs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GroundMarker:
    s_m: float
    d_m: float
    amplitude: float


def reading(s: float, d: float, markers, sigma_m: float) -> float:
    """Noise-free reading at line-frame position (s, d), 0 to 1."""
    total = 0.0
    for m in markers:
        r2 = (s - m.s_m) ** 2 + (d - m.d_m) ** 2
        total += m.amplitude * math.exp(-r2 / (2.0 * sigma_m**2))
    return min(total, 1.0)


@dataclass(frozen=True)
class FoundMarker:
    s_m: float          # reading-weighted centroid along the line
    strength: float     # largest reading
    n_captures: int


def detect_markers(captures, threshold: float, min_captures: int,
                   max_gap_m: float) -> list[FoundMarker]:
    """Markers from ``(s, reading)`` pairs of usable captures.

    Captures with a reading above ``threshold``, sorted by s, split into groups wherever
    two neighbours are more than ``max_gap_m`` apart; a group with at least
    ``min_captures`` captures is one marker at its reading-weighted centroid.
    """
    hits = sorted((s, r) for s, r in captures if r > threshold and not math.isnan(s))
    groups: list[list[tuple[float, float]]] = []
    for hit in hits:
        if groups and hit[0] - groups[-1][-1][0] <= max_gap_m:
            groups[-1].append(hit)
        else:
            groups.append([hit])
    found = []
    for group in groups:
        if len(group) < min_captures:
            continue
        weight = sum(r for _, r in group)
        found.append(FoundMarker(s_m=sum(s * r for s, r in group) / weight,
                                 strength=max(r for _, r in group), n_captures=len(group)))
    return found

