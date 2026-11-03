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
