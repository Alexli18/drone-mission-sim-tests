"""Load config.yaml into typed, read-only values.

Also a small CLI so shell steps read the image pins from the same file:
``python -m framework.config image|digest|local-tag|ros-base-tag|ros-tag|ros-domain``.
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
    speed_factor_ros: int
    home: Home

    @property
    def image_ref(self) -> str:
        return f"{self.image}@{self.digest}"

    @property
    def local_tag(self) -> str:
        # The image the tests run: the pinned image with PX4's lockstep fix
        # (scripts/build_px4_image.sh, docker/px4-sitl/).
        return f"px4-sitl-patched:{self.digest.split(':', 1)[1][:12]}"


@dataclass(frozen=True)
class MavlinkConfig:
    api_url: str

    @property
    def api_port(self) -> int:
        return int(self.api_url.rsplit(":", 1)[1])


@dataclass(frozen=True)
class RosConfig:
    """The ROS container (images, ports, DDS domain, PX4 pose and home topics)."""

    base_image_tag: str
    image_tag: str
    dds_agent_port: int
    rosbridge_port: int
    rosbridge_address: str
    localhost_only: bool
    domain_id: int
    pose_topic: str
    home_topic: str


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
    ros_start: float
    rosbridge_connect: float
    first_dds_position: float
    sensor_drain: float
    finalize: float
    replay: float


@dataclass(frozen=True)
class Tolerances:
    altitude_m: float
    waypoint_m: float
    home_m: float


@dataclass(frozen=True)
class SensorModel:
    """The toy survey sensor (simulation only)."""

    capture_rate_hz: float
    samples_per_capture: int
    sample_spacing_s: float      # between the samples of one capture
    footprint_sigma_m: float     # Gaussian footprint of a marker on the ground
    noise: float                 # reading noise (standard deviation)
    max_pose_gap_s: float        # widest PX4 pose gap the sensor interpolates across


@dataclass(frozen=True)
class GeorefConfig:
    off_line_m: float
    height_band_m: float
    hold_timeout_s: float        # simulated time
    wall_fallback_s: float
    max_pose_gap_s: float
    coverage_gap_factor: float


@dataclass(frozen=True)
class DetectorConfig:
    threshold: float             # reading above which a capture sees a marker
    min_captures: int
    max_gap_factor: float        # captures further apart than this x ds start a new marker


@dataclass(frozen=True)
class PassRules:
    position_tol_m: float
    coverage_min: float
    coverage_agreement: float
    flight_height_m: float
    cross_track_m: float
    speed_tolerance: float
    clock_shift_m: float
    replay_s_m: float
    acceptance_radius_m: float
    run_m: float
    settle_hover_s: float
    sensor_lag_max_s: float


@dataclass(frozen=True)
class SurveyConfig:
    sensor: SensorModel
    georeference: GeorefConfig
    detector: DetectorConfig
    pass_rule: PassRules


@dataclass(frozen=True)
class Config:
    px4: Px4Config
    mavlink: MavlinkConfig
    ros: RosConfig
    survey: SurveyConfig
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
    "RosConfig": RosConfig,
    "SurveyConfig": SurveyConfig,
    "SensorModel": SensorModel,
    "GeorefConfig": GeorefConfig,
    "DetectorConfig": DetectorConfig,
    "PassRules": PassRules,
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
        "ros-base-tag": lambda c: c.ros.base_image_tag,
        "ros-tag": lambda c: c.ros.image_tag,
        "ros-domain": lambda c: c.ros.domain_id,
    }
    if len(argv) != 1 or argv[0] not in commands:
        print(f"usage: python -m framework.config {{{'|'.join(commands)}}}", file=sys.stderr)
        return 2
    print(commands[argv[0]](load_config()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
