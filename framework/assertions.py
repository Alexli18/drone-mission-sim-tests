"""Waits and assertions with deadlines and readable failure messages.

Every wait in the framework goes through one of these helpers, so no test
synchronizes with a bare sleep (design, G7).
"""

from __future__ import annotations

import asyncio
import inspect
import json
import math
import time

EARTH_RADIUS_M = 6_371_000.0


class FlightAssertionError(AssertionError):
    """Failure with expected vs. actual and, if available, recent telemetry."""

    def __init__(self, expected: str, actual: str, recorder=None):
        lines = [f"expected: {expected} | actual: {actual}"]
        if recorder is not None:
            factor = recorder.effective_speed_factor()
            floor = recorder.cfg.min_effective_speed_factor
            if factor is not None and factor < floor:
                lines.append(f"effective speed factor {factor:.1f} below floor {floor:.1f}")
            lines.append("mode history: " + json.dumps(recorder.mode_history()))
            lines.append("last snapshot:\n" + json.dumps(recorder.snapshot(), indent=2))
        super().__init__("\n".join(lines))


async def wait_until(predicate, timeout_s, description, recorder=None, poll_interval_s=0.1):
    """Poll ``predicate`` (plain or async) until it is true or ``timeout_s`` passes."""
    deadline = time.monotonic() + timeout_s
    while True:
        if recorder is not None:
            recorder.check_alive()
        result = predicate()
        if inspect.isawaitable(result):
            result = await result
        if result:
            return result
        if time.monotonic() >= deadline:
            raise FlightAssertionError(description, f"not true after {timeout_s} s", recorder)
        await asyncio.sleep(poll_interval_s)


async def wait_for_stream(stream, predicate, timeout_s, description):
    """Read a MAVSDK stream until ``predicate`` holds; the error shows the last value."""
    last = {}

    async def read():
        async for value in stream:
            last["value"] = value
            if predicate(value):
                return value

    try:
        return await asyncio.wait_for(read(), timeout_s)
    except TimeoutError:
        actual = f"last value after {timeout_s} s: {last.get('value', 'none received')}"
        raise FlightAssertionError(description, actual) from None


async def call(coro, cfg, description):
    """Run one direct MAVSDK call with the ``mavsdk_call`` timeout."""
    timeout_s = cfg.timeouts_wall_s.mavsdk_call
    try:
        return await asyncio.wait_for(coro, timeout_s)
    except TimeoutError:
        raise FlightAssertionError(description, f"no reply within {timeout_s} s") from None


def horizontal_distance_m(lat1, lon1, lat2, lon2) -> float:
    """Equirectangular distance; error below 0.1 m within 200 m of home."""
    mean_lat = math.radians((lat1 + lat2) / 2)
    dx = math.radians(lon2 - lon1) * math.cos(mean_lat)
    dy = math.radians(lat2 - lat1)
    return EARTH_RADIUS_M * math.hypot(dx, dy)


async def wait_for_mode(recorder, mode: str, timeout_s, cfg):
    await wait_until(lambda: recorder.latest("flight_mode") is not None
                     and recorder.latest("flight_mode").name == mode,
                     timeout_s, f"flight mode {mode}", recorder, cfg.poll_interval_s)


async def wait_altitude(recorder, target_m, tol_m, timeout_s, cfg):
    def reached():
        pos = recorder.latest("position")
        return pos is not None and abs(pos.relative_altitude_m - target_m) <= tol_m

    await wait_until(reached, timeout_s, f"altitude {target_m} ± {tol_m} m", recorder,
                     cfg.poll_interval_s)


async def wait_landed_and_disarmed(recorder, timeout_s, cfg):
    def done():
        landed = recorder.latest("landed_state")
        return (landed is not None and landed.name == "ON_GROUND"
                and recorder.latest("armed") is False)

    await wait_until(done, timeout_s, "landed and disarmed", recorder, cfg.poll_interval_s)


def assert_altitude(recorder, target_m, tol_m, since_sim_s, until_sim_s) -> float:
    """Every recorded altitude in the window is within tolerance; returns the max error."""
    window = [p for p in recorder.positions()
              if p[0] is not None and since_sim_s <= p[0] <= until_sim_s]
    if not window:
        raise FlightAssertionError("position samples during the hover", "none recorded",
                                   recorder)
    errors = [abs(p[3] - target_m) for p in window]
    worst = max(errors)
    if worst > tol_m:
        raise FlightAssertionError(f"altitude within {target_m} ± {tol_m} m",
                                   f"max error {worst:.2f} m over {len(window)} samples",
                                   recorder)
    return worst


def assert_no_unexpected_modes(recorder, allowed: set, since_wall_s: float) -> None:
    """No flight mode outside ``allowed`` appeared after ``since_wall_s`` (monotonic)."""
    seen = [mode for _, wall, mode in recorder.mode_history() if wall >= since_wall_s]
    unexpected = [mode for mode in seen if mode not in allowed]
    if unexpected:
        raise FlightAssertionError(f"only modes {sorted(allowed)}",
                                   f"also saw {unexpected}", recorder)


def assert_waypoints_reached(recorder, waypoints_latlon, tol_m) -> float:
    """Each waypoint was passed within ``tol_m``; returns the largest miss distance."""
    positions = recorder.positions()
    misses = []
    for index, (lat, lon) in enumerate(waypoints_latlon):
        closest = min((horizontal_distance_m(lat, lon, p[1], p[2]) for p in positions),
                      default=math.inf)
        misses.append((index, closest))
    worst = max(distance for _, distance in misses)
    missed = [f"#{i}: {d:.1f} m" for i, d in misses if d > tol_m]
    if missed:
        raise FlightAssertionError(f"every waypoint passed within {tol_m} m",
                                   "missed " + ", ".join(missed), recorder)
    return worst


def assert_duration(start_sim_s, end_sim_s, bounds, recorder=None) -> float:
    duration = end_sim_s - start_sim_s
    if not bounds.min_s <= duration <= bounds.max_s:
        raise FlightAssertionError(f"duration {bounds.min_s}–{bounds.max_s} s (simulated)",
                                   f"{duration:.1f} s", recorder)
    return duration
