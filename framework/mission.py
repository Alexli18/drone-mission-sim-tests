"""Load scenario YAML files (merged over the parameter baseline).

M2 supports type ``hover``; ``mission`` (M3) and ``failsafe`` (M4) are added
with the tests that fly them.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from framework.config import REPO_ROOT, load_baseline_params

SCENARIOS_DIR = REPO_ROOT / "scenarios"


class ScenarioError(ValueError):
    """A scenario file is missing a field, has an unknown key, or a bad value."""


@dataclass(frozen=True)
class Scenario:
    name: str
    type: str
    description: str
    altitude_m: float
    hover_s: float
    px4_params: dict  # baseline merged with the file's own px4_params


# Keys each scenario type requires and allows, besides name/type/description.
_SCHEMA = {
    "hover": {"required": {"altitude_m", "hover_s"}, "optional": {"px4_params"}},
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

    return Scenario(
        name=data["name"],
        type=data["type"],
        description=data["description"],
        altitude_m=float(data["altitude_m"]),
        hover_s=float(data["hover_s"]),
        px4_params=params,
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
