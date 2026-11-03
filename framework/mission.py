"""Load scenario YAML files (merged over the parameter baseline) and build missions.

Types: ``hover`` (M2) and ``mission`` (M3); ``failsafe`` (M4) is added with the
test that flies it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml
from mavsdk_grpc.mission import MissionItem, MissionPlan

from framework.assertions import EARTH_RADIUS_M
from framework.config import REPO_ROOT, load_baseline_params

SCENARIOS_DIR = REPO_ROOT / "scenarios"


class ScenarioError(ValueError):
    """A scenario file is missing a field, has an unknown key, or a bad value."""


@dataclass(frozen=True)
class Area:
    """Survey rectangle in metres from home: ``width_m`` east, ``height_m`` north."""

    origin_north_m: float
    origin_east_m: float
    width_m: float
    height_m: float
    line_spacing_m: float


@dataclass(frozen=True)
class Bounds:
    min_s: float
    max_s: float


@dataclass(frozen=True)
class Scenario:
    name: str
    type: str
    description: str
    altitude_m: float
    px4_params: dict  # baseline merged with the file's own px4_params
    hover_s: float | None = None
    speed_m_s: float | None = None
    area: Area | None = None
    duration_bounds_s: Bounds | None = None


# Keys each scenario type requires and allows, besides name/type/description.
_SCHEMA = {
    "hover": {"required": {"altitude_m", "hover_s"}, "optional": {"px4_params"}},
    "mission": {"required": {"altitude_m", "speed_m_s", "area", "duration_bounds_s"},
                "optional": {"px4_params"}},
}
_COMMON = {"name", "type", "description"}


def load_scenario(path: Path) -> Scenario:
    path = Path(path)
    if not path.is_absolute():
        path = SCENARIOS_DIR / path
    with open(path) as f:
        data = yaml.safe_load(f) or {}

    def fail(message: str):
        raise ScenarioError(f"{path.name}: {message}")

    for key in sorted(_COMMON):
        if key not in data:
            fail(f"missing field: {key}")
    schema = _SCHEMA.get(data["type"])
    if schema is None:
        fail(f"unknown type: {data['type']} (supported: {', '.join(sorted(_SCHEMA))})")
    for key in sorted(schema["required"]):
        if key not in data:
            fail(f"missing field: {key}")
    allowed = _COMMON | schema["required"] | schema["optional"]
    for key in sorted(data):
        if key not in allowed:
            fail(f"unknown key: {key}")

    own_params = data.get("px4_params") or {}
    if not isinstance(own_params, dict):
        fail("px4_params must be a mapping")
    params = load_baseline_params()
    params.update(own_params)  # the scenario wins over the baseline

    def build(cls, key):
        if key not in data:
            return None
        value = data[key]
        expected = set(cls.__dataclass_fields__)
        if not isinstance(value, dict) or set(value) != expected:
            fail(f"{key} must have exactly the keys {sorted(expected)}")
        return cls(**{k: float(v) for k, v in value.items()})

    def number(key):
        return float(data[key]) if key in data else None

    return Scenario(
        name=data["name"],
        type=data["type"],
        description=data["description"],
        altitude_m=float(data["altitude_m"]),
        px4_params=params,
        hover_s=number("hover_s"),
        speed_m_s=number("speed_m_s"),
        area=build(Area, "area"),
        duration_bounds_s=build(Bounds, "duration_bounds_s"),
    )


def list_scenarios(scenario_type: str) -> list[Path]:
    """Scenario files of one type, sorted; files starting with ``_`` are skipped."""
    found = []
    for path in sorted(SCENARIOS_DIR.glob("*.yaml")):
        if path.name.startswith("_"):
            continue
        with open(path) as f:
            if (yaml.safe_load(f) or {}).get("type") == scenario_type:
                found.append(path)
    return found


def lawnmower_waypoints(area: Area) -> list[tuple[float, float]]:
    """Lines running north, spaced east by ``line_spacing_m``, alternating direction.

    Returns (north_m, east_m) offsets from home: both ends of each line.
    """
    lines = math.ceil(area.width_m / area.line_spacing_m) + 1
    south, north = area.origin_north_m, area.origin_north_m + area.height_m
    points = []
    for i in range(lines):
        east = area.origin_east_m + min(i * area.line_spacing_m, area.width_m)
        ends = [(south, east), (north, east)]
        points += ends if i % 2 == 0 else ends[::-1]
    return points


def offset_to_latlon(home_lat: float, home_lon: float, north_m: float, east_m: float):
    """Flat-earth conversion of a local offset from home to latitude/longitude."""
    lat = home_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = home_lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(home_lat))))
    return lat, lon


def to_mission_plan(waypoints, home_lat, home_lon, altitude_m, speed_m_s, acceptance_m):
    """Build a MAVSDK mission; also returns the waypoints as (lat, lon) for the checks."""
    latlon = [offset_to_latlon(home_lat, home_lon, n, e) for n, e in waypoints]
    items = [
        MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=altitude_m,
            speed_m_s=speed_m_s,
            is_fly_through=True,
            gimbal_pitch_deg=float("nan"),
            gimbal_yaw_deg=float("nan"),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float("nan"),
            camera_photo_interval_s=float("nan"),
            acceptance_radius_m=acceptance_m,
            yaw_deg=float("nan"),
            camera_photo_distance_m=float("nan"),
            vehicle_action=MissionItem.VehicleAction.NONE,
        )
        for lat, lon in latlon
    ]
    return MissionPlan(items), latlon
