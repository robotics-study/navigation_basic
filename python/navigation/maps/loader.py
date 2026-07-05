"""Map / scenario loaders. Dispatch on the yaml `type` field, not the extension."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from navigation.core.capabilities import MapBase
from navigation.core.types import Point

from .occupancy_grid import OccupancyGrid2D
from .pgm import read_pgm


def load_map(path: str | Path, seed: int = 0, connectivity: int = 8) -> MapBase:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    map_type = raw.get("type")
    if map_type != "occupancy_grid":
        raise ValueError(f"unsupported map type {map_type!r} (only occupancy_grid)")
    image_path = (path.parent / raw["image"]).resolve()
    _, _, pixels = read_pgm(str(image_path))
    origin = raw["origin"]
    return OccupancyGrid2D(
        pixels=pixels,
        resolution=float(raw["resolution"]),
        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
        occupied_thresh=float(raw.get("occupied_thresh", 0.65)),
        free_thresh=float(raw.get("free_thresh", 0.196)),
        connectivity=connectivity,
        seed=seed,
    )


@dataclass
class Scenario:
    map_path: str  # resolved absolute path to the map yaml
    start: Point
    goal: Point


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    if "agents" in raw:
        raise ValueError("multi-agent scenarios are out of scope for global_planning")
    map_path = (path.parent / raw["map"]).resolve()
    start = raw["start"]
    goal = raw["goal"]
    return Scenario(
        map_path=str(map_path),
        start=(float(start[0]), float(start[1])),
        goal=(float(goal[0]), float(goal[1])),
    )
