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
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

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
# Snap normalized time to 16 shades per ramp: the gradient still reads smooth, but
# the total distinct mark colors stay well under Pillow's 256-color GIF palette, so
# the GIF re-compresses instead of ballooning. Applied in every mode for color
# consistency between snapshots and GIFs.
_COLOR_LEVELS = 15


def _quantize(t: float) -> float:
    return round(t * _COLOR_LEVELS) / _COLOR_LEVELS


@lru_cache(maxsize=1)
def _path_cmap() -> "LinearSegmentedColormap":
    from matplotlib.colors import LinearSegmentedColormap

    return LinearSegmentedColormap.from_list("nav_path", _PATH_RAMP)


@lru_cache(maxsize=None)
def _ramp_cmap(stops: tuple[str, ...]) -> "LinearSegmentedColormap":
    from matplotlib.colors import LinearSegmentedColormap

    return LinearSegmentedColormap.from_list("nav_ramp", list(stops))


@dataclass
class Scene:
    """Draw-ready geometry extracted from a trace, ordered by event sequence.

    Each drawable element carries the running op index at which it appeared, so a
    frame showing "the first N ops" is a prefix cut across the per-type lists.
    """

    grid: "OccupancyGrid2D"
    extent: tuple[float, float, float, float]
    edges: list[tuple[Point, Point]] = field(default_factory=list)
    edge_orders: list[int] = field(default_factory=list)
    expanded: list[Point] = field(default_factory=list)
    expanded_orders: list[int] = field(default_factory=list)
    samples: list[Point] = field(default_factory=list)
    sample_orders: list[int] = field(default_factory=list)
    path: list[Point] = field(default_factory=list)
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


def _looks_like_cell(state: list[float], grid: "OccupancyGrid2D") -> bool:
    row, col = int(state[0]), int(state[1])
    return 0 <= row < grid.height and 0 <= col < grid.width


def _to_world_fn(grid: "OccupancyGrid2D") -> Callable[[list[float]], Point]:
    def to_world(state: list[float]) -> Point:
        # Discrete states are [row, col] ints; sampling states are [x, y] world.
        if all(float(v).is_integer() for v in state) and _looks_like_cell(state, grid):
            return grid.cell_to_world(int(state[0]), int(state[1]))
        return (float(state[0]), float(state[1]))

    return to_world


def _world_extent(grid: "OccupancyGrid2D") -> tuple[float, float, float, float]:
    # Bottom-left of image = world origin; derive from corner cell centers.
    x0, y1 = grid.cell_to_world(0, 0)
    x1, y0 = grid.cell_to_world(grid.height - 1, grid.width - 1)
    res = grid.resolution
    return (x0 - res / 2, x1 + res / 2, y0 - res / 2, y1 + res / 2)


def build_scene(events: list[dict[str, Any]], grid: "OccupancyGrid2D") -> Scene:
    to_world = _to_world_fn(grid)
    scene = Scene(grid=grid, extent=_world_extent(grid))
    order = 0
    for ev in events:
        name = ev.get("event")
        if name == "planning_started":
            scene.algorithm = str(ev.get("algorithm", ""))
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
        elif name in ("path_found", "planning_finished") and ev.get("path"):
            scene.path = [to_world(s) for s in ev["path"]]
    scene.total_ops = order
    return scene


def _draw(
    ax: "Any", scene: Scene, cutoff: int, show_path: bool, path_segments: int | None = None
) -> None:
    """Render the search state after the first ``cutoff`` ops.

    ``path_segments`` limits how many leading path segments are drawn (None = all);
    the GIF epilogue uses it to reveal the path start->goal. --save / snapshots /
    interactive leave it None, so the full path is drawn as before.
    """
    import numpy as np
    from matplotlib.collections import LineCollection

    ax.clear()
    ax.imshow(
        np.where(scene.grid.free_mask(), 1.0, 0.0),
        cmap="gray",
        origin="upper",
        extent=scene.extent,
        interpolation="nearest",
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
        sc = ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=4, c=colors, alpha=0.4, zorder=3)
        sc.set_antialiased(False)
    if n_expanded:
        pts = scene.expanded[:n_expanded]
        expanded_cmap = _ramp_cmap(_EXPANDED_RAMP)
        colors = [expanded_cmap(_quantize(scene.expanded_orders[i] / denom)) for i in range(n_expanded)]
        # Expanded nodes are the time map's protagonist: larger, near-opaque, on top.
        sc = ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=12, c=colors, alpha=0.9, zorder=4)
        sc.set_antialiased(False)
    drew_path = show_path and len(scene.path) >= 2
    if drew_path:
        _draw_path_gradient(ax, scene.path, path_segments)
    ax.set_xlim(scene.extent[0], scene.extent[1])
    ax.set_ylim(scene.extent[2], scene.extent[3])
    shown = min(cutoff, scene.total_ops)
    frac = 100.0 if scene.total_ops == 0 else 100.0 * shown / scene.total_ops
    ax.set_title(f"{scene.algorithm}  {frac:.0f}%  ({shown}/{scene.total_ops})")
    _draw_legend(
        ax, expanded_shown=n_expanded > 0, samples_shown=n_samples > 0, path_shown=drew_path
    )


def _draw_path_gradient(ax: "Any", path: list[Point], path_segments: int | None = None) -> None:
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


def _mark_proxy(color: tuple[float, float, float, float]) -> "Line2D":
    from matplotlib.lines import Line2D

    return Line2D(
        [0], [0], marker="o", linestyle="none", markerfacecolor=color,
        markeredgecolor="none", markersize=6,
    )


def _draw_legend(
    ax: "Any", *, expanded_shown: bool, samples_shown: bool, path_shown: bool
) -> None:
    from matplotlib.lines import Line2D

    # Per-mark colors vary by time, so legend swatches use each ramp's midpoint.
    handles: list["Line2D"] = []
    labels: list[str] = []
    if expanded_shown:
        handles.append(_mark_proxy(_ramp_cmap(_EXPANDED_RAMP)(0.5)))
        labels.append("expanded (early→late)")
    if samples_shown:
        handles.append(_mark_proxy(_ramp_cmap(_SAMPLE_RAMP)(0.5)))
        labels.append("samples (early→late)")
    if path_shown:
        handles.append(Line2D([0], [0], color=_PATH_RAMP[1], linewidth=2.5))
        labels.append("path (start→goal)")
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
    scene = build_scene(events, grid)

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
