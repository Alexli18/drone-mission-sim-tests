"""Record telemetry per test: JSON lines on disk, latest values and history in memory."""

from __future__ import annotations

import asyncio
import enum
import json
import time

from framework.assertions import call, wait_until
from framework.sitl import ARTIFACTS


def _jsonable(value):
    if isinstance(value, enum.Enum):
        return value.name
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "__dict__"):
        return {k: _jsonable(v) for k, v in vars(value).items() if not k.startswith("_")}
    return str(value)


class TelemetryRecorder:
    def __init__(self, drone, cfg, test_id: str):
        self.drone = drone
        self.cfg = cfg
        self.path = ARTIFACTS / "telemetry" / f"{test_id}.jsonl"
        self._latest: dict[str, object] = {}
        self._modes: list[tuple[float | None, float, str]] = []
        self._positions: list[tuple[float | None, float, float, float]] = []
        self._tasks: dict[str, asyncio.Task] = {}
        self._clock_first: tuple[float, float] | None = None  # (wall, sim)
        self._clock_last: tuple[float, float] | None = None
        self._file = None

    async def start(self) -> None:
        t = self.drone.telemetry
        # PX4 counts stream rates in simulated time (spike S2), so divide by the factor.
        factor = self.cfg.px4.speed_factor
        rates = self.cfg.telemetry_wall_hz
        # IMU carries the simulated clock, so it runs as fast as position (0.2 s steps at x10).
        for setter in (t.set_rate_position, t.set_rate_imu):
            await call(setter(rates.position / factor), self.cfg, f"{setter.__name__}")
        for setter in (t.set_rate_battery, t.set_rate_landed_state, t.set_rate_in_air):
            await call(setter(rates.other / factor), self.cfg, f"{setter.__name__}")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Open for the recorder's lifetime; closed in stop().
        self._file = open(self.path, "w")  # noqa: SIM115, ASYNC230
        streams = {
            "imu": t.imu,
            "position": t.position,
            "flight_mode": t.flight_mode,
            "landed_state": t.landed_state,
            "armed": t.armed,
            "in_air": t.in_air,
            "battery": t.battery,
            "health": t.health,
            # mission_progress is added in M3, when a mission exists to report on.
        }
        for name, factory in streams.items():
            self._tasks[name] = asyncio.create_task(self._read(name, factory), name=name)
        # Tests read sim_time_s() from their first line, so wait for the clock.
        await wait_until(lambda: self._clock_last is not None, self.cfg.timeouts_wall_s.health,
                         "first IMU sample (simulated clock)", self, self.cfg.poll_interval_s)

    async def _read(self, name, factory) -> None:
        async for value in factory():
            self._record(name, value)

    def _record(self, name, value) -> None:
        wall = time.monotonic()
        if name == "imu":
            sim = value.timestamp_us / 1e6  # simulated clock (spike S4)
            self._clock_first = self._clock_first or (wall, sim)
            self._clock_last = (wall, sim)
        sim_t = self.sim_time_s()
        self._latest[name] = value
        if name == "flight_mode" and (not self._modes or self._modes[-1][2] != value.name):
            self._modes.append((sim_t, wall, value.name))
        if name == "position":
            self._positions.append(
                (sim_t, value.latitude_deg, value.longitude_deg, value.relative_altitude_m)
            )
        line = {"wall_t": wall, "sim_t": sim_t, "stream": name, "value": _jsonable(value)}
        self._file.write(json.dumps(line) + "\n")

    def check_alive(self) -> None:
        for name, task in self._tasks.items():
            if task.done():
                if not task.cancelled() and task.exception() is not None:
                    raise RuntimeError(f"telemetry stream {name} crashed") from task.exception()
                raise RuntimeError(f"telemetry stream {name} ended unexpectedly")

    async def stop(self) -> None:
        for task in self._tasks.values():
            task.cancel()
        await asyncio.gather(*self._tasks.values(), return_exceptions=True)
        if self._file is not None:
            self._file.close()

    def sim_time_s(self) -> float | None:
        return self._clock_last[1] if self._clock_last else None

    def effective_speed_factor(self) -> float | None:
        if not self._clock_first or not self._clock_last:
            return None
        wall = self._clock_last[0] - self._clock_first[0]
        if wall < 1.0:
            return None
        return (self._clock_last[1] - self._clock_first[1]) / wall

    def latest(self, stream: str):
        return self._latest.get(stream)

    def snapshot(self) -> dict:
        return {name: _jsonable(value) for name, value in self._latest.items()}

    def mode_history(self) -> list:
        return list(self._modes)

    def positions(self) -> list:
        return list(self._positions)
