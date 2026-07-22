#!/usr/bin/env python3
"""Replay a trace jsonl over its map (matplotlib).

Depends only on the trace/map spec + navigation core/maps — never on algorithm
modules. All planner state needed for visualization arrives via trace events.

Output modes (combinable; all but interactive are headless via the Agg backend):
  (default)          interactive window with the full accumulated frame
  --save out.png     final accumulated frame to a PNG
  --gif out.gif      animated replay of the search progress + final path
  --snapshots dir/   evenly-spaced mid-search PNG snapshots + a final frame
"""

from __future__ import annotations

import argparse
import json
import math
import os
from bisect import bisect_right
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import cache, lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.lines import Line2D

    from navigation.maps.occupancy_grid import OccupancyGrid2D

Point = tuple[float, float]

# Frame budget so a trace with thousands of events yields a watchable GIF.
_DEFAULT_TARGET_FRAMES = 150
_DEFAULT_SNAPSHOTS = 8

# Path is drawn as a start->goal time gradient. These stops are pre-validated for
# CVD separation from the other marks; do not alter them. Deep purple (start) ->
# magenta -> red (goal).
_PATH_RAMP = ["#6f2fb5", "#c2179b", "#e5484d"]
# Sequential single-hue ramps (early = bright, late = dark) encode search-time
# order on each mark type. Edges use sky (not the old blue-violet) to stay clear
# of the path's purple start color. Pre-validated; do not alter.
_EXPANDED_RAMP = ("#fef08a", "#ea580c", "#7c2d12")  # yellow -> orange -> dark brown
_SAMPLE_RAMP = ("#4ade80", "#14532d")  # green
_EDGE_RAMP = ("#7dd3fc", "#075985")  # sky
# Dynamic replanning (D* Lite): robot trail teal (clear of every ramp + the path
# gradient), sensed obstacles fog in as dark squares over the all-free belief. CVD-safe.
_ROBOT_COLOR = "#0d9488"
# Local planning problem definition (Pure Pursuit reference path / every algorithm's
# goal): slate gray + the path ramp's own goal red, so a failed run (no path_found)
# still shows what the robot was chasing.
_REFERENCE_PATH_COLOR = "#94a3b8"
_GOAL_COLOR = _PATH_RAMP[-1]
# Potential Fields (Khatib 1986) force quiver: blue pull toward the goal, red push
# away from obstacles — matches the attractive/repulsive convention used on the docs
# site. Vector length is clamped (see _clamped_vec) so a large gain never draws an
# arrow off the map.
_FORCE_COLOR_ATT = "#2563eb"
_FORCE_COLOR_REP = "#dc2626"
_FORCE_MAX_RADIUS_CELLS = 3.0
# VFH (Borenstein & Koren 1991) polar histogram wedges: open sectors light, sectors
# over threshold (blocked) dark. Bin magnitude has no fixed physical unit, so the
# radius is normalized to the tick's own max bin rather than an absolute scale.
_HIST_COLOR_OPEN = "#94a3b8"
_HIST_COLOR_BLOCKED = "#334155"
_HIST_MAX_RADIUS_CELLS = 6.0
# Candidate markers (Pure Pursuit lookahead point / VFH valley probes): amber, clear
# of every other mark color in this file.
_CANDIDATE_COLOR = "#f59e0b"
# DWA candidate rollouts: unselected arcs in slate (same hue family as the reference
# path, which never co-occurs — DWA takes no reference path), rejected ones fainter,
# and the selected arc in the candidate amber so it reads as "this tick's choice".
_ROLLOUT_COLOR = "#94a3b8"
_ROLLOUT_ALPHA_ADMISSIBLE = 0.35
_ROLLOUT_ALPHA_REJECTED = 0.12
# Kinodynamic (Hybrid A*) path headings + local planning current-pose heading: a dark
# slate distinct from the path gradient + every ramp.
_HEADING_COLOR = "#1e293b"
# Elastic Bands / TEB band overlay: the deformable band's bubble/pose state, latest
# tick only. Warm gold keeps it clear of the marks it always coexists with (robot
# trail teal, path gradient purple/magenta/red, reference-path/heading slate).
_BAND_COLOR = "#ca8a04"


def _clamped_vec(v: Point, max_len: float) -> Point:
    mag = math.hypot(v[0], v[1])
    if mag <= 1e-9 or mag <= max_len:
        return v
    scale = max_len / mag
    return (v[0] * scale, v[1] * scale)


# Snap normalized time to 16 shades per ramp: the gradient still reads smooth, but
# the total distinct mark colors stay well under Pillow's 256-color GIF palette, so
# the GIF re-compresses instead of ballooning. Applied in every mode for color
# consistency between snapshots and GIFs.
_COLOR_LEVELS = 15


def _quantize(t: float) -> float:
    return round(t * _COLOR_LEVELS) / _COLOR_LEVELS


@lru_cache(maxsize=1)
def _path_cmap() -> LinearSegmentedColormap:
    from matplotlib.colors import LinearSegmentedColormap

    return LinearSegmentedColormap.from_list("nav_path", _PATH_RAMP)


@cache
def _ramp_cmap(stops: tuple[str, ...]) -> LinearSegmentedColormap:
    from matplotlib.colors import LinearSegmentedColormap

    return LinearSegmentedColormap.from_list("nav_ramp", list(stops))


@dataclass
class Scene:
    """Draw-ready geometry extracted from a trace, ordered by event sequence.

    Each drawable element carries the running op index at which it appeared, so a
    frame showing "the first N ops" is a prefix cut across the per-type lists.
    """

    grid: OccupancyGrid2D
    extent: tuple[float, float, float, float]
    edges: list[tuple[Point, Point]] = field(default_factory=list)
    edge_orders: list[int] = field(default_factory=list)
    expanded: list[Point] = field(default_factory=list)
    expanded_orders: list[int] = field(default_factory=list)
    samples: list[Point] = field(default_factory=list)
    sample_orders: list[int] = field(default_factory=list)
    # Dynamic replanning (D* Lite): the robot's executed cells and the obstacles it
    # senses. Empty for every static planner, which keeps their frames unchanged.
    robot: list[Point] = field(default_factory=list)
    robot_orders: list[int] = field(default_factory=list)
    # Per-robot-move heading (radians), parallel to robot/robot_orders. D* Lite's
    # 2-element cell state carries no heading, so those ticks store nan (skip-drawn).
    robot_headings: list[float] = field(default_factory=list)
    revealed: list[Point] = field(default_factory=list)
    revealed_orders: list[int] = field(default_factory=list)
    # Local planning (Potential Fields): attractive/repulsive force vectors at the
    # pose in force order, one force_computed event per control tick.
    forces: list[tuple[Point, Point, Point]] = field(default_factory=list)  # (pos, F_att, F_rep)
    force_orders: list[int] = field(default_factory=list)
    # Local planning (VFH): polar obstacle-density histogram at the pose in force
    # order, one histogram_updated event per control tick.
    # each entry: (pos, bins, threshold)
    histograms: list[tuple[Point, list[float], float]] = field(default_factory=list)
    histogram_orders: list[int] = field(default_factory=list)
    # Local planning (Elastic Bands / TEB): the deformable band's bubble/pose state at
    # the pose in band order, one band_updated event per control tick. Each entry is
    # the raw `band` array: item length 3 = [x, y, radius] (EB bubble), length 4 =
    # [x, y, theta, dt] (TEB pose).
    bands: list[list[list[float]]] = field(default_factory=list)
    band_orders: list[int] = field(default_factory=list)
    # Local planning (Pure Pursuit lookahead point / VFH valley candidates). Global
    # search's own candidate_evaluated flood is excluded in build_scene (see there),
    # so this stays empty for every non-local trace.
    candidates: list[Point] = field(default_factory=list)
    candidate_orders: list[int] = field(default_factory=list)
    # Rollout-scoring planners (DWA): per-candidate predicted polyline (trace
    # `rollout`) plus the admissible/selected flags from `data`. Parallel to
    # candidates/candidate_orders; None/default entries for planners that emit no
    # rollout (Pure Pursuit, VFH, ...) keep their render unchanged.
    candidate_rollouts: list[list[Point] | None] = field(default_factory=list)
    candidate_selected: list[bool] = field(default_factory=list)
    candidate_admissible: list[bool] = field(default_factory=list)
    # Local planning problem definition, loaded from planning_started.scenario (not a
    # search/execution event) so tracking planners' reference path and the goal render
    # even on a failed run (no path_found).
    reference_path: list[Point] = field(default_factory=list)
    goal: Point | None = None
    path: list[Point] = field(default_factory=list)
    # Kinodynamic (Hybrid A*): per-path-pose heading (radians). Empty for every 2-element
    # (Cell/Point) planner, so their render is untouched; populated only by SE(2) traces.
    path_headings: list[float] = field(default_factory=list)
    total_ops: int = 0
    algorithm: str = ""


def _read_events(trace_path: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with open(trace_path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _resolve_map(trace_path: str, events: list[dict[str, Any]], override: str | None) -> str:
    if override:
        return override
    for ev in events:
        if ev.get("event") == "planning_started" and ev.get("map"):
            candidate = Path(ev["map"])
            if candidate.exists():
                return str(candidate)
            # trace stores a repo-root-relative path; try relative to the trace too.
            near = Path(trace_path).parent / ev["map"]
            if near.exists():
                return str(near)
            return str(candidate)
    raise ValueError("no map path in trace; pass --map")


def _looks_like_cell(state: list[float], grid: OccupancyGrid2D) -> bool:
    row, col = int(state[0]), int(state[1])
    return 0 <= row < grid.height and 0 <= col < grid.width


def _to_world_fn(grid: OccupancyGrid2D) -> Callable[[list[float]], Point]:
    def to_world(state: list[float]) -> Point:
        # A 3-element state is always an SE(2) Pose [x, y, theta] (world) — Cells/Points
        # are 2-element. Returning world (x, y) directly also fixes a latent bug where an
        # all-integer 3-element Pose would be misread as a Cell by the check below.
        if len(state) >= 3:
            return (float(state[0]), float(state[1]))
        # Discrete states are [row, col] ints; sampling states are [x, y] world.
        if all(float(v).is_integer() for v in state) and _looks_like_cell(state, grid):
            return grid.cell_to_world(int(state[0]), int(state[1]))
        return (float(state[0]), float(state[1]))

    return to_world


def _world_extent(grid: OccupancyGrid2D) -> tuple[float, float, float, float]:
    # Bottom-left of image = world origin; derive from corner cell centers.
    x0, y1 = grid.cell_to_world(0, 0)
    x1, y0 = grid.cell_to_world(grid.height - 1, grid.width - 1)
    res = grid.resolution
    return (x0 - res / 2, x1 + res / 2, y0 - res / 2, y1 + res / 2)


def _resolve_scenario_path(scenario_field: str, trace_path: str | None) -> Path | None:
    # Mirrors _resolve_map's two candidates: repo-root-relative (matches how demos
    # write the field, cwd == repo root) or relative to the trace file's directory.
    candidate = Path(scenario_field)
    if candidate.exists():
        return candidate
    if trace_path is not None:
        near = Path(trace_path).parent / scenario_field
        if near.exists():
            return near
    return None


def build_scene(
    events: list[dict[str, Any]], grid: OccupancyGrid2D, trace_path: str | None = None
) -> Scene:
    to_world = _to_world_fn(grid)
    scene = Scene(grid=grid, extent=_world_extent(grid))
    # Global search planners also emit candidate_evaluated (e.g. A*'s per-neighbor
    # scoring) in bulk; that flood must stay ignored as before. Only local planning's
    # closed-loop tick (Pure Pursuit lookahead / VFH valleys) also emits robot_moved,
    # so gating candidate collection on robot_moved's presence tells the two apart.
    is_local_trace = any(ev.get("event") == "robot_moved" for ev in events)
    order = 0
    for ev in events:
        name = ev.get("event")
        if name == "planning_started":
            scene.algorithm = str(ev.get("algorithm", ""))
            scenario_field = ev.get("scenario")
            if scenario_field:
                path = _resolve_scenario_path(str(scenario_field), trace_path)
                if path is not None:
                    from navigation.maps.loader import load_scenario

                    sc = load_scenario(path)
                    scene.reference_path = list(sc.reference_path)
                    scene.goal = sc.goal
        elif name == "node_expanded" and "state" in ev:
            scene.expanded.append(to_world(ev["state"]))
            scene.expanded_orders.append(order)
            order += 1
        elif name == "sample_drawn" and "state" in ev:
            scene.samples.append(to_world(ev["state"]))
            scene.sample_orders.append(order)
            order += 1
        elif name in ("edge_added", "rewire") and "state" in ev and "parent" in ev:
            scene.edges.append((to_world(ev["parent"]), to_world(ev["state"])))
            scene.edge_orders.append(order)
            order += 1
        elif name == "robot_moved" and "state" in ev:
            state = ev["state"]
            scene.robot.append(to_world(state))
            scene.robot_orders.append(order)
            scene.robot_headings.append(float(state[2]) if len(state) >= 3 else math.nan)
            order += 1
        elif name == "obstacle_revealed" and "state" in ev:
            scene.revealed.append(to_world(ev["state"]))
            scene.revealed_orders.append(order)
            order += 1
        elif name == "force_computed" and "state" in ev:
            data = ev.get("data") or {}
            f_att = (float(data.get("fx_att", 0.0)), float(data.get("fy_att", 0.0)))
            f_rep = (float(data.get("fx_rep", 0.0)), float(data.get("fy_rep", 0.0)))
            scene.forces.append((to_world(ev["state"]), f_att, f_rep))
            scene.force_orders.append(order)
            order += 1
        elif name == "histogram_updated" and "state" in ev:
            data = ev.get("data") or {}
            bins = [float(b) for b in ev.get("bins", [])]
            threshold = float(data.get("threshold", 0.0))
            scene.histograms.append((to_world(ev["state"]), bins, threshold))
            scene.histogram_orders.append(order)
            order += 1
        elif name == "band_updated" and ev.get("band"):
            scene.bands.append([[float(v) for v in item] for item in ev["band"]])
            scene.band_orders.append(order)
            order += 1
        elif name == "candidate_evaluated" and is_local_trace and "state" in ev:
            scene.candidates.append(to_world(ev["state"]))
            scene.candidate_orders.append(order)
            data = ev.get("data") or {}
            rollout = ev.get("rollout")
            scene.candidate_rollouts.append(
                [to_world(p) for p in rollout] if rollout else None
            )
            scene.candidate_selected.append(float(data.get("selected", 0.0)) >= 1.0)
            # Planners without the flag (Pure Pursuit's single lookahead point etc.)
            # default to admissible so their marker keeps full opacity.
            scene.candidate_admissible.append(float(data.get("admissible", 1.0)) >= 1.0)
            order += 1
        elif name in ("path_found", "planning_finished") and ev.get("path"):
            scene.path = [to_world(s) for s in ev["path"]]
            # SE(2) poses carry a heading in state[2]; gate on 3-element states so
            # Cell/Point paths leave path_headings empty (heading overlay skipped).
            scene.path_headings = [float(s[2]) for s in ev["path"] if len(s) >= 3]
    scene.total_ops = order
    return scene


def _draw(
    ax: Any, scene: Scene, cutoff: int, show_path: bool, path_segments: int | None = None
) -> None:
    """Render the search state after the first ``cutoff`` ops.

    ``path_segments`` limits how many leading path segments are drawn (None = all);
    the GIF epilogue uses it to reveal the path start->goal. --save / snapshots /
    interactive leave it None, so the full path is drawn as before.
    """
    import numpy as np
    from matplotlib.collections import LineCollection

    ax.clear()
    # D* Lite starts blind (freespace belief) and reveals obstacles as it senses them,
    # so its background is all-free and the true walls fog in cell by cell. Local
    # planners execute on a KNOWN map, so robot_moved alone (no reveals) must NOT flip
    # the background — only a planner that actually senses obstacles (obstacle_revealed,
    # D* Lite family) has belief != ground truth.
    dynamic = bool(scene.revealed)
    if dynamic:
        ax.imshow(
            np.ones(scene.grid.free_mask().shape, dtype=float),
            cmap="gray", origin="upper", extent=scene.extent, vmin=0.0, vmax=1.0,
            interpolation="nearest",
        )
    else:
        ax.imshow(
            np.where(scene.grid.free_mask(), 1.0, 0.0),
            cmap="gray",
            origin="upper",
            extent=scene.extent,
            interpolation="nearest",
        )
    # Local planning problem definition: drawn right after the background so every
    # search/execution mark below stays on top of it. Present even on a failed run
    # (STALLED/COLLISION/TIMEOUT never emits path_found) because it comes from the
    # scenario, not the execution outcome.
    reference_path_shown = len(scene.reference_path) >= 2
    if reference_path_shown:
        ref = scene.reference_path
        ax.plot(
            [p[0] for p in ref], [p[1] for p in ref], color=_REFERENCE_PATH_COLOR,
            linewidth=1.4, linestyle=(0, (5, 3)), zorder=1.6, alpha=0.9,
        )
    if scene.goal is not None:
        ax.scatter(
            [scene.goal[0]], [scene.goal[1]], marker="*", s=150, color=_GOAL_COLOR,
            edgecolors="white", linewidths=1.0, zorder=6.5,
        )
    n_edges = bisect_right(scene.edge_orders, cutoff)
    n_expanded = bisect_right(scene.expanded_orders, cutoff)
    n_samples = bisect_right(scene.sample_orders, cutoff)
    # Normalize every mark's color by the FINAL op count so a mark's color is fixed
    # for the whole animation (already-drawn marks never recolor as the GIF advances).
    denom = scene.total_ops or 1
    # Marks are rendered without anti-aliasing: with thousands of per-time-colored
    # overlapping edges, AA blends countless boundary colors that wreck the GIF's
    # LZW/palette compression (28 MB vs 5 MB) for no meaningful visual gain at this
    # line width. The color quantization keeps distinct shades bounded on top.
    if n_edges:
        edge_cmap = _ramp_cmap(_EDGE_RAMP)
        edge_colors = [edge_cmap(_quantize(scene.edge_orders[i] / denom)) for i in range(n_edges)]
        edges = LineCollection(
            scene.edges[:n_edges], colors=edge_colors, linewidths=0.4, zorder=2, antialiaseds=False
        )
        ax.add_collection(edges)
    if n_samples:
        pts = scene.samples[:n_samples]
        sample_cmap = _ramp_cmap(_SAMPLE_RAMP)
        colors = [sample_cmap(_quantize(scene.sample_orders[i] / denom)) for i in range(n_samples)]
        sc = ax.scatter(
            [p[0] for p in pts], [p[1] for p in pts], s=4, c=colors, alpha=0.4, zorder=3
        )
        sc.set_antialiased(False)
    if n_expanded:
        pts = scene.expanded[:n_expanded]
        expanded_cmap = _ramp_cmap(_EXPANDED_RAMP)
        colors = [
            expanded_cmap(_quantize(scene.expanded_orders[i] / denom)) for i in range(n_expanded)
        ]
        # Expanded nodes are the time map's protagonist: larger, near-opaque, on top.
        sc = ax.scatter(
            [p[0] for p in pts], [p[1] for p in pts], s=12, c=colors, alpha=0.9, zorder=4
        )
        sc.set_antialiased(False)
    n_revealed = 0
    if dynamic:
        n_revealed = bisect_right(scene.revealed_orders, cutoff)
        if n_revealed:
            # Fog obstacles in as exact dark cells the moment the robot senses them.
            overlay = np.full(scene.grid.free_mask().shape, np.nan)
            for p in scene.revealed[:n_revealed]:
                row, col = scene.grid.world_to_cell(p[0], p[1])
                overlay[row, col] = 0.0
            ax.imshow(
                np.ma.masked_invalid(overlay), cmap="gray", origin="upper",
                extent=scene.extent, vmin=0.0, vmax=1.0, interpolation="nearest", zorder=1.5,
            )
    # Executed trail: D* Lite (dynamic background above) and every local planner (known,
    # static background) both walk one cell/pose per robot_moved, so this no longer
    # nests under `dynamic` — only the fog overlay above is background-mode-specific.
    n_robot = bisect_right(scene.robot_orders, cutoff)
    if n_robot:
        trail = scene.robot[:n_robot]
        ax.plot(
            [p[0] for p in trail], [p[1] for p in trail], color=_ROBOT_COLOR,
            linewidth=1.6, alpha=0.85, zorder=5, solid_capstyle="round",
        )
        ax.scatter(
            [trail[-1][0]], [trail[-1][1]], marker="D", s=55, color=_ROBOT_COLOR,
            edgecolors="white", linewidths=1.1, zorder=8,
        )
        # Local planning (SE(2) pose): heading arrow at the current pose. D* Lite's
        # 2-element cell state stored nan for this tick, so it draws nothing.
        current_heading = scene.robot_headings[n_robot - 1]
        if not math.isnan(current_heading):
            ax.quiver(
                [trail[-1][0]], [trail[-1][1]],
                [math.cos(current_heading)], [math.sin(current_heading)],
                color=_HEADING_COLOR, angles="xy", scale_units="xy", scale=2.2,
                width=0.008, headwidth=4, headlength=5, zorder=9, alpha=0.9,
            )
    n_force = bisect_right(scene.force_orders, cutoff)
    if n_force:
        # Latest tick only: force is an instantaneous quantity, so accumulating every
        # past tick's arrows would just clutter the frame (and bloat the GIF palette).
        pos, f_att, f_rep = scene.forces[n_force - 1]
        max_len = _FORCE_MAX_RADIUS_CELLS * scene.grid.resolution
        f_att = _clamped_vec(f_att, max_len)
        f_rep = _clamped_vec(f_rep, max_len)
        ax.quiver(
            [pos[0], pos[0]], [pos[1], pos[1]], [f_att[0], f_rep[0]], [f_att[1], f_rep[1]],
            color=[_FORCE_COLOR_ATT, _FORCE_COLOR_REP], angles="xy", scale_units="xy",
            scale=1.0, width=0.01, zorder=6, alpha=0.9,
        )
    n_hist = bisect_right(scene.histogram_orders, cutoff)
    if n_hist and scene.histograms[n_hist - 1][1]:
        # Latest tick only, same rationale as the force quiver above.
        pos, bins, threshold = scene.histograms[n_hist - 1]
        max_bin = max(bins) or 1.0
        r_max = _HIST_MAX_RADIUS_CELLS * scene.grid.resolution
        n_bins = len(bins)
        hist_segments: list[tuple[Point, Point]] = []
        hist_colors: list[str] = []
        for k, value in enumerate(bins):
            angle = 2.0 * math.pi * k / n_bins  # sector 0 = world +x, ccw (schema)
            radius = (value / max_bin) * r_max
            hist_segments.append(
                (pos, (pos[0] + radius * math.cos(angle), pos[1] + radius * math.sin(angle)))
            )
            hist_colors.append(_HIST_COLOR_BLOCKED if value > threshold else _HIST_COLOR_OPEN)
        ax.add_collection(
            LineCollection(hist_segments, colors=hist_colors, linewidths=1.6, zorder=6, alpha=0.85)
        )
    n_band = bisect_right(scene.band_orders, cutoff)
    if n_band and scene.bands[n_band - 1]:
        # Latest tick only, same rationale as the force quiver / histogram above.
        band = scene.bands[n_band - 1]
        if len(band[0]) == 3:
            # Elastic Bands: bubble centers as a polyline, each bubble's clearance
            # radius as a translucent disc with a solid outline.
            from matplotlib.colors import to_rgba
            from matplotlib.patches import Circle

            ax.plot(
                [item[0] for item in band], [item[1] for item in band], color=_BAND_COLOR,
                linewidth=1.2, alpha=0.9, zorder=5.2,
            )
            face = to_rgba(_BAND_COLOR, 0.15)
            for x, y, radius in band:
                ax.add_patch(
                    Circle((x, y), radius, facecolor=face, edgecolor=_BAND_COLOR,
                           linewidth=1.0, zorder=5.1)
                )
        else:
            # TEB: per-segment linewidth encodes the segment's ΔT (thicker = slower),
            # plus a small marker at each pose.
            dts = [item[3] for item in band]
            max_dt = max(dts) or 1.0
            for i in range(len(band) - 1):
                dt_i = dts[i + 1]  # entry 0's dt=0 is a placeholder (no prior segment)
                lw = 0.8 + 3.0 * (dt_i / max_dt)
                ax.plot(
                    [band[i][0], band[i + 1][0]], [band[i][1], band[i + 1][1]],
                    color=_BAND_COLOR, linewidth=lw, alpha=0.9, zorder=5.2,
                )
            ax.scatter(
                [item[0] for item in band], [item[1] for item in band],
                s=10, color=_BAND_COLOR, edgecolors="white", linewidths=0.4, zorder=5.3,
            )
    # Current-tick candidates only (those emitted since the previous robot move), so a
    # long PP/VFH run doesn't smear every historical probe on top of each other.
    tick_candidates: list[Point] = []
    rollouts_shown = False
    if scene.candidates:
        tick_start = scene.robot_orders[n_robot - 2] if n_robot >= 2 else -1
        lo = bisect_right(scene.candidate_orders, tick_start)
        hi = bisect_right(scene.candidate_orders, cutoff)
        tick_candidates = scene.candidates[lo:hi]
        # Rollout arcs first (below the endpoint markers): unselected admissible
        # arcs thin slate, rejected/inadmissible ones fainter, the selected arc
        # in accent amber on top. Latest tick only, same rationale as candidates.
        from matplotlib.colors import to_rgba

        rollout_segments: list[list[Point]] = []
        rollout_colors: list[tuple[float, float, float, float]] = []
        selected_rollout: list[Point] | None = None
        for i in range(lo, hi):
            rollout = scene.candidate_rollouts[i]
            if rollout is None or len(rollout) < 2:
                continue
            if scene.candidate_selected[i]:
                selected_rollout = rollout
                continue
            alpha = (
                _ROLLOUT_ALPHA_ADMISSIBLE
                if scene.candidate_admissible[i]
                else _ROLLOUT_ALPHA_REJECTED
            )
            rollout_segments.append(rollout)
            rollout_colors.append(to_rgba(_ROLLOUT_COLOR, alpha))
        if rollout_segments:
            ax.add_collection(
                LineCollection(
                    rollout_segments, colors=rollout_colors, linewidths=0.8, zorder=5.5,
                    antialiaseds=False,
                )
            )
        if selected_rollout is not None:
            ax.plot(
                [p[0] for p in selected_rollout], [p[1] for p in selected_rollout],
                color=_CANDIDATE_COLOR, linewidth=2.0, zorder=5.8, alpha=0.95,
            )
        rollouts_shown = bool(rollout_segments) or selected_rollout is not None
        if tick_candidates:
            ax.scatter(
                [p[0] for p in tick_candidates], [p[1] for p in tick_candidates],
                marker="^", s=40, color=_CANDIDATE_COLOR, edgecolors="white",
                linewidths=0.6, zorder=6, alpha=0.9,
            )
    drew_path = show_path and len(scene.path) >= 2
    if drew_path:
        _draw_path_gradient(ax, scene.path, path_segments)
    # Kinodynamic (Hybrid A*): overlay heading arrows so the SE(2) pose direction is
    # visible. Empty path_headings (every 2-element planner) skips this entirely.
    drew_headings = drew_path and bool(scene.path_headings)
    if drew_headings:
        _draw_headings(ax, scene.path, scene.path_headings, path_segments)
    ax.set_xlim(scene.extent[0], scene.extent[1])
    ax.set_ylim(scene.extent[2], scene.extent[3])
    shown = min(cutoff, scene.total_ops)
    frac = 100.0 if scene.total_ops == 0 else 100.0 * shown / scene.total_ops
    ax.set_title(f"{scene.algorithm}  {frac:.0f}%  ({shown}/{scene.total_ops})")
    _draw_legend(
        ax, expanded_shown=n_expanded > 0, samples_shown=n_samples > 0, path_shown=drew_path,
        robot_shown=n_robot > 0, revealed_shown=n_revealed > 0, headings_shown=drew_headings,
        reference_path_shown=reference_path_shown, force_shown=n_force > 0,
        histogram_shown=n_hist > 0, band_shown=n_band > 0, candidate_shown=bool(tick_candidates),
        rollout_shown=rollouts_shown,
    )


def _draw_path_gradient(ax: Any, path: list[Point], path_segments: int | None = None) -> None:
    """Draw the path as a time-ordered color gradient (start->goal) with a halo.

    A solid line hides ordering and blends with the tree/walls; the gradient
    encodes progression and the white underlay keeps it legible over dark cells.
    Start (circle) / goal (star) markers disambiguate direction, which color alone
    cannot. ``path_segments`` draws only the leading N segments (for the reveal
    epilogue); colors are keyed to the full path so a revealed segment never
    recolors as more of the path appears.
    """
    from matplotlib.collections import LineCollection

    n_seg = len(path) - 1
    if n_seg < 1:
        return
    reveal = n_seg if path_segments is None else max(0, min(path_segments, n_seg))
    if reveal < 1:
        return
    cmap = _path_cmap()
    colors = [cmap(i / (n_seg - 1) if n_seg > 1 else 0.0) for i in range(n_seg)]
    segments = [[path[i], path[i + 1]] for i in range(n_seg)]
    # White halo first, then the gradient main line just above it (revealed prefix).
    ax.add_collection(LineCollection(segments[:reveal], colors="white", linewidths=4.0, zorder=5))
    ax.add_collection(
        LineCollection(segments[:reveal], colors=colors[:reveal], linewidths=2.5, zorder=6)
    )
    # Start marker as soon as the reveal begins; goal marker only when fully revealed.
    ax.scatter(
        [path[0][0]], [path[0][1]],
        marker="o", s=70, color=_PATH_RAMP[0], edgecolors="white", linewidths=1.2, zorder=7,
    )
    if reveal >= n_seg:
        ax.scatter(
            [path[-1][0]], [path[-1][1]],
            marker="*", s=190, color=_PATH_RAMP[-1], edgecolors="white", linewidths=1.2, zorder=7,
        )


def _draw_headings(
    ax: Any, path: list[Point], headings: list[float], path_segments: int | None = None
) -> None:
    """Overlay SE(2) heading arrows along the path (Hybrid A*).

    A dense sub-pose polyline would clutter if every pose got an arrow, so arrows are
    subsampled to at most ~24. ``path_segments`` clips to the revealed prefix so the GIF
    epilogue reveals headings in step with the path gradient.
    """
    import math as _math

    n = min(len(path), len(headings))
    if n < 1:
        return
    reveal = n if path_segments is None else max(0, min(path_segments + 1, n))
    if reveal < 1:
        return
    stride = max(1, reveal // 24)
    idx = list(range(0, reveal, stride))
    xs = [path[i][0] for i in idx]
    ys = [path[i][1] for i in idx]
    us = [_math.cos(headings[i]) for i in idx]
    vs = [_math.sin(headings[i]) for i in idx]
    ax.quiver(
        xs, ys, us, vs, color=_HEADING_COLOR, angles="xy", scale_units="xy", scale=2.2,
        width=0.006, headwidth=4, headlength=5, zorder=7, alpha=0.9,
    )


def _mark_proxy(color: str | tuple[float, float, float, float]) -> Line2D:
    from matplotlib.lines import Line2D

    return Line2D(
        [0], [0], marker="o", linestyle="none", markerfacecolor=color,
        markeredgecolor="none", markersize=6,
    )


def _draw_legend(
    ax: Any, *, expanded_shown: bool, samples_shown: bool, path_shown: bool,
    robot_shown: bool = False, revealed_shown: bool = False, headings_shown: bool = False,
    reference_path_shown: bool = False, force_shown: bool = False,
    histogram_shown: bool = False, band_shown: bool = False, candidate_shown: bool = False,
    rollout_shown: bool = False,
) -> None:
    from matplotlib.lines import Line2D

    # Per-mark colors vary by time, so legend swatches use each ramp's midpoint.
    handles: list[Line2D] = []
    labels: list[str] = []
    if expanded_shown:
        handles.append(_mark_proxy(_ramp_cmap(_EXPANDED_RAMP)(0.5)))
        labels.append("expanded (early→late)")
    if samples_shown:
        handles.append(_mark_proxy(_ramp_cmap(_SAMPLE_RAMP)(0.5)))
        labels.append("samples (early→late)")
    if revealed_shown:
        handles.append(_mark_proxy((0.1, 0.1, 0.1, 1.0)))
        labels.append("sensed obstacle")
    if reference_path_shown:
        handles.append(Line2D([0], [0], color=_REFERENCE_PATH_COLOR, linewidth=1.4, linestyle="--"))
        labels.append("reference path")
    if robot_shown:
        handles.append(Line2D([0], [0], color=_ROBOT_COLOR, linewidth=1.8))
        labels.append("robot trail")
    if force_shown:
        handles.append(Line2D([0], [0], color=_FORCE_COLOR_ATT, linewidth=1.8))
        labels.append("attractive force")
        handles.append(Line2D([0], [0], color=_FORCE_COLOR_REP, linewidth=1.8))
        labels.append("repulsive force")
    if histogram_shown:
        handles.append(Line2D([0], [0], color=_HIST_COLOR_BLOCKED, linewidth=1.8))
        labels.append("histogram (bin ∝ density)")
    if band_shown:
        handles.append(Line2D([0], [0], color=_BAND_COLOR, linewidth=1.8))
        labels.append("band")
    if rollout_shown:
        handles.append(Line2D([0], [0], color=_ROLLOUT_COLOR, linewidth=1.2))
        labels.append("rollout (selected = amber)")
    if candidate_shown:
        handles.append(_mark_proxy(_CANDIDATE_COLOR))
        labels.append("candidate")
    if path_shown:
        handles.append(Line2D([0], [0], color=_PATH_RAMP[1], linewidth=2.5))
        labels.append("path (start→goal)")
    if headings_shown:
        handles.append(Line2D(
            [0], [0], marker=(3, 0, 0), linestyle="none", markerfacecolor=_HEADING_COLOR,
            markeredgecolor="none", markersize=7,
        ))
        labels.append("heading")
    if handles:
        ax.legend(handles, labels, loc="upper left", fontsize=7)


def _use_agg() -> None:
    import matplotlib

    if matplotlib.get_backend().lower() != "agg":
        matplotlib.use("Agg")


def _frame_cutoffs(total: int, events_per_frame: int) -> list[int]:
    cutoffs = list(range(events_per_frame, total + 1, events_per_frame))
    if not cutoffs or cutoffs[-1] < total:
        cutoffs.append(total)
    return cutoffs


# Frame spec for the animation: (cutoff, show_path, path_segments).
_FrameSpec = tuple[int, bool, "int | None"]
_EPILOGUE_FRAMES = 20
_HOLD_SECONDS = 3  # linger on the completed path before the GIF loops


def _gif_frames(total: int, cutoffs: list[int], n_path_segments: int, fps: int) -> list[_FrameSpec]:
    """Search frames (no path), then a start->goal path reveal, then a hold.

    The search phase never draws the path; the path is revealed segment-by-segment
    in the epilogue so the gradient is actually watchable instead of flashing only
    on a single (dedup-collapsed) final frame.
    """
    hold = fps * _HOLD_SECONDS
    frames: list[_FrameSpec] = [(c, False, None) for c in cutoffs]
    if n_path_segments >= 1:
        n_epi = min(_EPILOGUE_FRAMES, n_path_segments)
        reveals = [max(1, round((k + 1) / n_epi * n_path_segments)) for k in range(n_epi)]
        frames += [(total, True, r) for r in reveals]
        frames += [(total, True, None)] * hold  # hold the completed path
    else:
        frames += [(total, False, None)] * hold  # no path: hold the final search state
    return frames


def save_gif(
    scene: Scene,
    path: str,
    fps: int = 10,
    events_per_frame: int | None = None,
    target_frames: int = _DEFAULT_TARGET_FRAMES,
) -> int:
    """Animate the whole replay to a GIF (PillowWriter). Returns the frame count."""
    _use_agg()
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    total = scene.total_ops
    if events_per_frame is None or events_per_frame <= 0:
        # Auto-downsample so long traces stay within the frame budget.
        events_per_frame = max(1, math.ceil(total / target_frames)) if total > 0 else 1
    cutoffs = _frame_cutoffs(total, events_per_frame)
    frames = _gif_frames(total, cutoffs, max(0, len(scene.path) - 1), fps)

    fig, ax = plt.subplots(figsize=(6, 6))

    def update(spec: _FrameSpec) -> None:
        cutoff, show_path, path_segments = spec
        _draw(ax, scene, cutoff, show_path, path_segments)

    anim = FuncAnimation(fig, update, frames=frames, interval=1000.0 / fps)
    # fresh checkout 에서 out/viz/<algo>/ 를 바로 지정해도 실패하지 않도록 (snapshots 와 동일 동작)
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return len(frames)


def save_snapshots(scene: Scene, out_dir: str, count: int = _DEFAULT_SNAPSHOTS) -> list[str]:
    """Save ``count`` evenly-spaced mid-search PNGs plus a final frame with the path."""
    _use_agg()
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)
    total = scene.total_ops
    cutoffs = [max(1, round(total * (i + 1) / (count + 1))) for i in range(count)]
    cutoffs.append(total)  # final frame shows the solution path
    paths: list[str] = []
    for i, cutoff in enumerate(cutoffs):
        fig, ax = plt.subplots(figsize=(6, 6))
        _draw(ax, scene, cutoff, show_path=cutoff >= total)
        out = os.path.join(out_dir, f"snapshot_{i:03d}.png")
        fig.savefig(out, dpi=110, bbox_inches="tight")
        plt.close(fig)
        paths.append(out)
    return paths


def save_static(scene: Scene, path: str) -> None:
    _use_agg()
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6))
    _draw(ax, scene, scene.total_ops, show_path=True)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def _show_interactive(scene: Scene) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6))
    _draw(ax, scene, scene.total_ops, show_path=True)
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="replay a navigation trace over its map")
    parser.add_argument("trace", help="trace jsonl path")
    parser.add_argument("--map", default=None, help="override map yaml path")
    parser.add_argument("--save", default=None, help="render final frame to a PNG (headless)")
    parser.add_argument("--gif", default=None, help="render the replay to an animated GIF")
    parser.add_argument("--fps", type=int, default=10, help="GIF frames per second")
    parser.add_argument(
        "--events-per-frame",
        type=int,
        default=None,
        help="trace ops advanced per GIF frame (default: auto ~150 frames total)",
    )
    parser.add_argument("--snapshots", default=None, help="directory for mid-search PNG snapshots")
    parser.add_argument(
        "--snapshot-count",
        type=int,
        default=_DEFAULT_SNAPSHOTS,
        help="number of mid-search snapshots before the final frame",
    )
    args = parser.parse_args()

    headless = bool(args.save or args.gif or args.snapshots)
    import matplotlib

    if headless:
        matplotlib.use("Agg")

    # Import navigation lazily so `--help` works without the package installed.
    from navigation.maps.loader import load_map
    from navigation.maps.occupancy_grid import OccupancyGrid2D

    events = _read_events(args.trace)
    grid = load_map(_resolve_map(args.trace, events, args.map))
    assert isinstance(grid, OccupancyGrid2D)
    scene = build_scene(events, grid, trace_path=args.trace)

    if args.gif:
        n_frames = save_gif(scene, args.gif, fps=args.fps, events_per_frame=args.events_per_frame)
        print(f"saved {args.gif} ({n_frames} frames)")
    if args.snapshots:
        paths = save_snapshots(scene, args.snapshots, count=args.snapshot_count)
        print(f"saved {len(paths)} snapshots to {args.snapshots}")
    if args.save:
        save_static(scene, args.save)
        print(f"saved {args.save}")
    if not headless:
        _show_interactive(scene)


if __name__ == "__main__":
    main()
