"""Load config.yaml into typed, read-only values.

Also a small CLI so shell steps read the image pin from the same file:
``python -m framework.config image|digest|local-tag``.
"""

from __future__ import annotations

import dataclasses
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "config.yaml"
BASELINE_PATH = REPO_ROOT / "scenarios" / "_baseline.yaml"


class ConfigError(ValueError):
    """config.yaml is missing a key or has an unexpected one."""


@dataclass(frozen=True)
class Home:
    lat: float
    lon: float
    alt: float


@dataclass(frozen=True)
class Px4Config:
    image: str
    digest: str
    model: str
    speed_factor: int
    home: Home

    @property
    def image_ref(self) -> str:
        return f"{self.image}@{self.digest}"

    @property
    def local_tag(self) -> str:
        return f"px4-sitl-pinned:{self.digest.split(':', 1)[1][:12]}"


@dataclass(frozen=True)
class MavlinkConfig:
    api_url: str

    @property
    def api_port(self) -> int:
        return int(self.api_url.rsplit(":", 1)[1])


@dataclass(frozen=True)
class TelemetryRates:
    position: float
    other: float


@dataclass(frozen=True)
class Timeouts:
    container_start: float
    connect: float
    mavsdk_call: float
    health: float
    armable: float
    takeoff: float
    hover: float
    mission_start: float
    land: float
    mission: float
    failsafe: float


@dataclass(frozen=True)
class Tolerances:
    altitude_m: float
    waypoint_m: float
    home_m: float


@dataclass(frozen=True)
class Config:
    px4: Px4Config
    mavlink: MavlinkConfig
    telemetry_wall_hz: TelemetryRates
    timeouts_wall_s: Timeouts
    tolerances: Tolerances
    poll_interval_s: float
    min_effective_speed_ratio: float

    @property
    def min_effective_speed_factor(self) -> float:
        return self.min_effective_speed_ratio * self.px4.speed_factor


def _build(cls, data, path: str):
    """Build dataclass ``cls`` from ``data``, naming the first missing or extra key."""
    if not isinstance(data, dict):
        raise ConfigError(f"{path or 'config'}: expected a mapping")
    fields = {f.name: f for f in dataclasses.fields(cls)}
    for name in fields:
        if name not in data:
            raise ConfigError(f"missing key: {path}{name}")
    for name in data:
        if name not in fields:
            raise ConfigError(f"unknown key: {path}{name}")
    values = {}
    for name, field in fields.items():
        value = data[name]
        # Annotations are strings (``from __future__ import annotations``).
        if field.type in _NESTED:
            value = _build(_NESTED[field.type], value, f"{path}{name}.")
        values[name] = value
    return cls(**values)


_NESTED = {
    "Home": Home,
    "Px4Config": Px4Config,
    "MavlinkConfig": MavlinkConfig,
    "TelemetryRates": TelemetryRates,
    "Timeouts": Timeouts,
    "Tolerances": Tolerances,
}


def load_config(path: Path = CONFIG_PATH) -> Config:
    with open(path) as f:
        return _build(Config, yaml.safe_load(f), "")


def load_baseline_params(path: Path = BASELINE_PATH) -> dict:
    with open(path) as f:
        return dict(yaml.safe_load(f)["px4_params"])


def main(argv: list[str]) -> int:
    commands = {
        "image": lambda c: c.px4.image_ref,
        "digest": lambda c: c.px4.digest,
        "local-tag": lambda c: c.px4.local_tag,
    }
    if len(argv) != 1 or argv[0] not in commands:
        print(f"usage: python -m framework.config {{{'|'.join(commands)}}}", file=sys.stderr)
        return 2
    print(commands[argv[0]](load_config()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
