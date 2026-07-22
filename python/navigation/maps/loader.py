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
    # Optional SE(2) start/goal heading (radians, world) for kinodynamic planners.
    # Defaulted so existing scenarios (no theta) load unchanged; discrete/sampling ignore.
    start_theta: float = 0.0
    goal_theta: float = 0.0
    # Optional reference path (world waypoints) for tracking-family local planners.
    # Defaulted to empty so existing scenarios (no field) load unchanged.
    reference_path: tuple[Point, ...] = ()


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    if "agents" in raw:
        # reference_path only extends the single-agent problem definition, so this
        # rejection is unrelated to it and applies to every category, not just one.
        raise ValueError("multi-agent scenarios are not supported by load_scenario")
    map_path = (path.parent / raw["map"]).resolve()
    start = raw["start"]
    goal = raw["goal"]
    raw_reference_path = raw.get("reference_path")
    reference_path: tuple[Point, ...] = (
        tuple((float(pt[0]), float(pt[1])) for pt in raw_reference_path)
        if raw_reference_path is not None
        else ()
    )
    return Scenario(
        map_path=str(map_path),
        start=(float(start[0]), float(start[1])),
        goal=(float(goal[0]), float(goal[1])),
        start_theta=float(raw.get("start_theta", 0.0)),
        goal_theta=float(raw.get("goal_theta", 0.0)),
        reference_path=reference_path,
    )
