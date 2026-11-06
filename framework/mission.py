"""Load scenario YAML files (merged over the parameter baseline) and build missions.

Types: ``hover`` (M2), ``mission`` (M3), ``failsafe`` (M4) and ``line_survey``
(a survey line flown with a sensor; docs/line-survey.md).
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
class Trigger:
    type: str  # at_waypoint | after_s
    value: float


@dataclass(frozen=True)
class Failure:
    kind: str
    trigger: Trigger | None = None
    breach_north_m_s: float | None = None  # geofence_breach only


@dataclass(frozen=True)
class Expected:
    """Semantics: the design's "Pass rule for failsafe scenarios"."""

    mode_reached: str | None
    leave_mode: str | None
    allowed_before: tuple[str, ...]
    then_mode: str | None
    landed: bool
    disarmed: bool
    max_dist_from_home_m: float | None


@dataclass(frozen=True)
class SurveyLine:
    """A straight survey line in metres from home; ``heading_deg`` clockwise from north.

    Line frame: s along the line from its start, d cross-track, positive to the right
    of the heading.
    """

    start_north_m: float
    start_east_m: float
    heading_deg: float
    length_m: float

    def project(self, north_m: float, east_m: float) -> tuple[float, float]:
        """(s, d) of a (north, east) offset from home."""
        h = math.radians(self.heading_deg)
        dn, de = north_m - self.start_north_m, east_m - self.start_east_m
        return dn * math.cos(h) + de * math.sin(h), -dn * math.sin(h) + de * math.cos(h)

    def point(self, s_m: float, d_m: float = 0.0) -> tuple[float, float]:
        """(north, east) offset from home of the line-frame point (s, d)."""
        h = math.radians(self.heading_deg)
        return (self.start_north_m + s_m * math.cos(h) - d_m * math.sin(h),
                self.start_east_m + s_m * math.sin(h) + d_m * math.cos(h))


@dataclass(frozen=True)
class Clock:
    """Sensor clock against PX4 time: a fixed offset and a Gaussian jitter (σ), in ms."""

    offset_ms: float = 0.0
    jitter_ms: float = 0.0


@dataclass(frozen=True)
class Marker:
    """A ground marker at line-frame position (s, d); ``amplitude`` is the peak reading."""

    s_m: float
    d_m: float = 0.0
    amplitude: float = 0.8


@dataclass(frozen=True)
class LineSurvey:
    """The ``line_survey`` part of a scenario."""

    line: SurveyLine
    sensor_height_m: float
    sensor_offset_m: tuple[float, float, float]   # body frame (x forward, y right, z down)
    markers: tuple[Marker, ...]
    clock: Clock
    seed: int
    schedule: str           # push | weekly
    replay_check: bool


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
    failure: Failure | None = None
    expected: Expected | None = None
    geofence: tuple[tuple[float, float], ...] | None = None  # (north_m, east_m) from home
    survey: LineSurvey | None = None


# Keys each scenario type requires and allows, besides name/type/description.
_SCHEMA = {
    "hover": {"required": {"altitude_m", "hover_s"}, "optional": {"px4_params"}},
    "mission": {"required": {"altitude_m", "speed_m_s", "area", "duration_bounds_s"},
                "optional": {"px4_params"}},
    "failsafe": {"required": {"altitude_m", "failure", "expected"},
                 "optional": {"px4_params", "speed_m_s", "area", "geofence"}},
    "line_survey": {"required": {"line", "sensor_height_m", "speed_m_s", "markers"},
                    "optional": {"px4_params", "sensor_offset_m", "clock", "seed", "schedule",
                                 "replay_check"}},
}
_SCHEDULES = ("push", "weekly")
_EXPECTED_KEYS = {"mode_reached", "leave_mode", "allowed_before", "then_mode", "landed",
                  "disarmed", "max_dist_from_home_m"}
_COMMON = {"name", "type", "description"}
_OPTIONAL_COMMON = {"profile"}   # a parameter overlay: scenarios/_<profile>.yaml


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
    allowed = _COMMON | _OPTIONAL_COMMON | schema["required"] | schema["optional"]
    for key in sorted(data):
        if key not in allowed:
            fail(f"unknown key: {key}")

    own_params = data.get("px4_params") or {}
    if not isinstance(own_params, dict):
        fail("px4_params must be a mapping")
    params = load_baseline_params()
    if "profile" in data:
        params.update(load_profile_params(data["profile"], fail))  # a named overlay
    params.update(own_params)  # the scenario wins over the baseline and the profile

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

    failure = expected = geofence = survey = None
    if data["type"] == "failsafe":
        failure, expected, geofence = _failsafe_parts(data, params, fail)
    if data["type"] == "line_survey":
        if "UXRCE_DDS_DOM_ID" in own_params:
            fail("px4_params must not set UXRCE_DDS_DOM_ID (the fixture sets the DDS domain "
                 "through ROS_DOMAIN_ID)")
        survey = _line_survey(data, fail)
        # The commanded vehicle height: sensor height plus the offset below the vehicle.
        altitude_m = survey.sensor_height_m + survey.sensor_offset_m[2]
    else:
        altitude_m = float(data["altitude_m"])

    return Scenario(
        name=data["name"],
        type=data["type"],
        description=data["description"],
        altitude_m=altitude_m,
        px4_params=params,
        hover_s=number("hover_s"),
        speed_m_s=number("speed_m_s"),
        area=build(Area, "area"),
        duration_bounds_s=build(Bounds, "duration_bounds_s"),
        failure=failure,
        expected=expected,
        geofence=geofence,
        survey=survey,
    )


def load_profile_params(name, fail) -> dict:
    """``px4_params`` of ``scenarios/_<name>.yaml`` (a profile is a named parameter overlay)."""
    if not isinstance(name, str) or not name.isidentifier():
        fail(f"profile must be a name, got {name!r}")
    path = SCENARIOS_DIR / f"_{name}.yaml"
    if not path.is_file():
        fail(f"profile: scenarios/_{name}.yaml does not exist")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if set(data) != {"px4_params"} or not isinstance(data["px4_params"], dict):
        fail(f"scenarios/_{name}.yaml must hold exactly one key, px4_params")
    return dict(data["px4_params"])


def _line_survey(data, fail) -> LineSurvey:
    """The ``line_survey`` keys, each check naming its key."""

    def number(key, value, positive=False):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            fail(f"{key} must be a number, got {value!r}")
        if positive and value <= 0:
            fail(f"{key} must be a number > 0, got {value!r}")
        return float(value)

    def numbers(key, raw, keys):
        if not isinstance(raw, dict) or set(raw) != set(keys):
            fail(f"{key} must have exactly the keys {sorted(keys)}")
        return {name: number(f"{key}.{name}", value) for name, value in raw.items()}

    line = SurveyLine(**numbers("line", data["line"], SurveyLine.__dataclass_fields__))
    if line.length_m <= 0:
        fail(f"line.length_m must be > 0, got {line.length_m}")
    offset = numbers("sensor_offset_m", data.get("sensor_offset_m", {"x": 0, "y": 0, "z": 0}), ("x", "y", "z"))
    clock = Clock(**numbers("clock", data.get("clock", {"offset_ms": 0.0, "jitter_ms": 0.0}),
                            Clock.__dataclass_fields__))
    if clock.jitter_ms < 0:
        fail(f"clock.jitter_ms must be >= 0, got {clock.jitter_ms}")
    raw_markers = data["markers"]
    if not isinstance(raw_markers, list):
        fail("markers must be a list (empty for a clean line)")
    markers = []
    for index, raw in enumerate(raw_markers):
        if (not isinstance(raw, dict) or "s_m" not in raw
                or set(raw) - set(Marker.__dataclass_fields__)):
            fail(f"markers[{index}] must be a mapping with s_m and optionally d_m, amplitude")
        marker = Marker(**numbers(f"markers[{index}]", raw, set(raw)))
        if not 0 <= marker.s_m <= line.length_m:
            fail(f"markers[{index}].s_m must lie on the line (0 to {line.length_m})")
        if not 0 < marker.amplitude <= 1:
            fail(f"markers[{index}].amplitude must be in (0, 1], got {marker.amplitude}")
        markers.append(marker)
    seed = data.get("seed", 0)
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        fail(f"seed must be an integer >= 0, got {seed!r}")
    schedule = data.get("schedule", "push")
    if schedule not in _SCHEDULES:
        fail(f"schedule must be push or weekly, got {schedule!r}")
    replay_check = data.get("replay_check", False)
    if not isinstance(replay_check, bool):
        fail(f"replay_check must be true or false, got {replay_check!r}")
    return LineSurvey(line=line, sensor_height_m=number("sensor_height_m",
                                                         data["sensor_height_m"], positive=True),
                      sensor_offset_m=(offset["x"], offset["y"], offset["z"]),
                      markers=tuple(markers), clock=clock, seed=seed, schedule=schedule,
                      replay_check=replay_check)


def _failsafe_parts(data, params, fail):
    from framework.failures import FAULTS  # imported here: failures.py imports this module

    raw = data["failure"]
    if not isinstance(raw, dict) or "kind" not in raw:
        fail("failure must be a mapping with a kind")
    kind = raw["kind"]
    if kind not in FAULTS:
        fail(f"unknown failure.kind: {kind} (known: {', '.join(sorted(FAULTS))})")
    allowed = {"kind", "trigger"} | ({"breach_north_m_s"} if kind == "geofence_breach" else set())
    for key in sorted(raw):
        if key not in allowed:
            fail(f"unknown key: failure.{key}")
    trigger = None
    if "trigger" in raw:
        t = raw["trigger"]
        if not isinstance(t, dict) or set(t) != {"type", "value"} or t["type"] not in (
                "at_waypoint", "after_s"):
            fail("failure.trigger must be {type: at_waypoint|after_s, value: N}")
        trigger = Trigger(t["type"], float(t["value"]))

    geofence = None
    if kind == "geofence_breach":
        if "breach_north_m_s" not in raw or "geofence" not in data:
            fail("geofence_breach needs failure.breach_north_m_s and a geofence block")
        polygon = data["geofence"].get("inclusion_polygon") if isinstance(
            data["geofence"], dict) else None
        if not polygon or len(polygon) < 3:
            fail("geofence.inclusion_polygon needs at least 3 points")
        geofence = tuple((float(n), float(e)) for n, e in polygon)
        if not _contains(geofence, (0.0, 0.0)):
            fail("geofence.inclusion_polygon must contain home (0, 0)")
    else:
        if "geofence" in data:
            fail("geofence is only allowed for failure.kind geofence_breach")
        for key in ("area", "speed_m_s"):
            if key not in data:
                fail(f"missing field: {key}")

    raw_expected = data["expected"]
    if not isinstance(raw_expected, dict):
        fail("expected must be a mapping")
    for key in sorted(raw_expected):
        if key not in _EXPECTED_KEYS:
            fail(f"unknown key: expected.{key}")
    if ("mode_reached" in raw_expected) == ("leave_mode" in raw_expected):
        fail("expected needs exactly one of mode_reached and leave_mode")
    allowed_before = tuple(raw_expected.get("allowed_before", ()))
    then_mode = raw_expected.get("then_mode")
    if "leave_mode" in raw_expected and (allowed_before or then_mode):
        fail("allowed_before and then_mode must be empty with leave_mode")
    distance = raw_expected.get("max_dist_from_home_m")
    expected = Expected(
        mode_reached=raw_expected.get("mode_reached"),
        leave_mode=raw_expected.get("leave_mode"),
        allowed_before=allowed_before,
        then_mode=then_mode,
        landed=bool(raw_expected.get("landed", False)),
        disarmed=bool(raw_expected.get("disarmed", False)),
        max_dist_from_home_m=None if distance is None else float(distance),
    )

    missing = sorted(FAULTS[kind].required_params - set(params))
    if missing:
        fail(f"px4_params (after the baseline merge) lacks {', '.join(missing)}")
    return Failure(kind, trigger, raw.get("breach_north_m_s")), expected, geofence


def _contains(polygon, point) -> bool:
    """Ray casting point-in-polygon on (north, east) offsets."""
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


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


def line_waypoints(line: SurveyLine, run_m: float) -> list[tuple[float, float]]:
    """Run-in point, line start, line end and run-out point as (north_m, east_m) from home.

    ``run_m`` has no default: the test passes ``survey.pass_rule.run_m`` from config.yaml.
    """
    return [line.point(s) for s in (-run_m, 0.0, line.length_m, line.length_m + run_m)]


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
