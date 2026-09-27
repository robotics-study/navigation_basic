"""Multi-agent scenario loader for the velocity-obstacle family.

`maps/loader.py`'s `load_scenario`/`Scenario` are single-agent by contract (they
raise on an `agents:` key), so a genuinely different schema -- N agents, each
with its own start/goal/radius and an optional scripted (non-cooperative)
velocity -- gets its own family-owned loader instead of extending that one.
Depends on yaml + `core.types` + this family's own `AgentSpec` only; the
static grid referenced by `map:` is loaded separately by the caller via
`maps.load_map`, exactly like the single-agent demo flow does.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from navigation.core.types import Point, RobotState

from .agent_sim import AgentSpec


@dataclass(frozen=True)
class AgentScenario:
    map_path: str
    agents: tuple[AgentSpec, ...]


def load_agent_scenario(path: str | Path) -> AgentScenario:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    map_path = str((path.parent / raw["map"]).resolve())
    agents: list[AgentSpec] = []
    for entry in raw["agents"]:
        start_xy = entry["start"]
        goal_xy = entry["goal"]
        theta = float(entry.get("theta", 0.0))
        scripted = entry.get("scripted_velocity")
        scripted_velocity: Point | None = (
            (float(scripted[0]), float(scripted[1])) if scripted is not None else None
        )
        agents.append(
            AgentSpec(
                start=RobotState(pose=(float(start_xy[0]), float(start_xy[1]), theta)),
                goal=(float(goal_xy[0]), float(goal_xy[1]), 0.0),
                radius=float(entry["radius"]),
                scripted_velocity=scripted_velocity,
            )
        )
    return AgentScenario(map_path=map_path, agents=tuple(agents))
