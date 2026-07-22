#!/usr/bin/env python3
"""Benchmark matrix runner: (scenario x algorithm) -> metrics -> markdown report.

Runs each language demo as a subprocess and collects metrics from the trace's
`planning_finished` event. Depends on spec/core/maps only — it knows each
algorithm's required capability from a small static table (not by importing
algorithm modules) and marks incompatible (map, algorithm) pairs rather than
erroring.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navigation.core.capabilities import Capability
from navigation.maps.loader import load_map, load_scenario

_REPO_ROOT = Path(__file__).resolve().parents[2]

# Required capability per algorithm (kept here so bench never imports algorithms).
_REQUIRED: dict[str, Capability] = {
    "bfs": Capability.DISCRETE_SPACE,
    "dijkstra": Capability.DISCRETE_SPACE,
    "astar": Capability.DISCRETE_SPACE,
    "ara_star": Capability.DISCRETE_SPACE,
    # Binding constraint: LINE_OF_SIGHT_SPACE implies DISCRETE_SPACE (it extends
    # it), so a discrete-only map is correctly marked incompatible for Theta*.
    "jps": Capability.DYNAMIC_GRID_SPACE,
    "ad_star": Capability.DYNAMIC_GRID_SPACE,
    "theta_star": Capability.LINE_OF_SIGHT_SPACE,
    "lazy_theta_star": Capability.LINE_OF_SIGHT_SPACE,
    "visibility_astar": Capability.LINE_OF_SIGHT_SPACE,
    "anya": Capability.LINE_OF_SIGHT_SPACE,
    "dstar_lite": Capability.DYNAMIC_GRID_SPACE,
    "hybrid_astar": Capability.SE2_COLLISION_SPACE,
    "rrt": Capability.SAMPLING_SPACE,
    "rrt_connect": Capability.SAMPLING_SPACE,
    "rrt_star": Capability.SAMPLING_SPACE,
    "informed_rrt_star": Capability.SAMPLING_SPACE,
    "prm": Capability.SAMPLING_SPACE,
    "prm_star": Capability.SAMPLING_SPACE,
    "fmt_star": Capability.SAMPLING_SPACE,
    "bit_star": Capability.SAMPLING_SPACE,
    "abit_star": Capability.SAMPLING_SPACE,
    "ait_star": Capability.SAMPLING_SPACE,
    "eit_star": Capability.SAMPLING_SPACE,
    "fcit_star": Capability.SAMPLING_SPACE,
    "fast_rrt": Capability.SAMPLING_SPACE,
    "lqr_rrt_star": Capability.SAMPLING_SPACE,
    "kinodynamic_rrt_star": Capability.SAMPLING_SPACE,
    "sst": Capability.SAMPLING_SPACE,
    # OBSTACLE_QUERY extends SE2_COLLISION_SPACE (core/capabilities.py), so a map that
    # only supports collision queries (no OccupancyGrid2D EDT/enumeration) is already
    # correctly marked incompatible for these three.
    "potential_fields": Capability.OBSTACLE_QUERY,
    "vfh": Capability.OBSTACLE_QUERY,
    "dwa": Capability.OBSTACLE_QUERY,
    "pure_pursuit": Capability.OBSTACLE_QUERY,
    "stanley": Capability.OBSTACLE_QUERY,
    "regulated_pure_pursuit": Capability.OBSTACLE_QUERY,
}
_ORDER = ["bfs", "dijkstra", "astar", "ara_star", "jps", "ad_star", "dstar_lite", "theta_star",
          "lazy_theta_star", "visibility_astar", "anya", "hybrid_astar",
          "rrt", "rrt_connect", "rrt_star", "prm_star", "lqr_rrt_star", "kinodynamic_rrt_star",
          "informed_rrt_star", "prm", "fmt_star", "bit_star", "abit_star", "sst",
          "ait_star", "fast_rrt", "eit_star", "fcit_star",
          "potential_fields", "vfh", "dwa", "pure_pursuit", "stanley", "regulated_pure_pursuit"]
# Path trackers follow a given reference path rather than seeking a bare goal, so a
# scenario without one is a shape mismatch (not a capability mismatch) — reported the
# same "incompatible" way, from a static table so bench still never imports algorithms.
_NEEDS_PATH = {"pure_pursuit", "stanley", "regulated_pure_pursuit"}


@dataclass
class Row:
    scenario: str
    algorithm: str
    # global: "ok"|"incompatible"|"no_path"|"error"; local adds "collision"|"stalled"|"timeout"
    status: str
    metrics: dict[str, float]


def _is_local_algo(algo: str) -> bool:
    """True iff `algo` is a local planner. Reuses _REQUIRED (the one existing
    per-algorithm fact table) instead of adding a second category map — local
    planners are exactly the ones that need the obstacle-proximity capability."""
    return _REQUIRED.get(algo) is Capability.OBSTACLE_QUERY


def _local_status(metrics: dict[str, float]) -> str:
    """Local planning's granular outcome, reconstructed from the simulator's numeric
    collided/stalled flags: `metrics` is a plain number map (spec/trace_schema.json),
    so the simulator cannot emit a string status directly."""
    if metrics.get("success", 0.0) >= 1.0:
        return "ok"
    if metrics.get("collided", 0.0) >= 1.0:
        return "collision"
    if metrics.get("stalled", 0.0) >= 1.0:
        return "stalled"
    return "timeout"


def _is_local_row(row: Row) -> bool:
    """Classify a result row as local vs. global planning from trace facts, not a
    second algorithm->category map. A row with metrics is classified by the presence
    of `time_to_goal` (a local-only metric key); a row with none (incompatible/error,
    where no trace was even produced) falls back to the algorithm's own capability."""
    if "time_to_goal" in row.metrics:
        return True
    if row.metrics:
        return False
    return _is_local_algo(row.algorithm)


def _final_metrics(trace_path: Path) -> dict[str, float] | None:
    result: dict[str, float] | None = None
    success = False
    with open(trace_path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            ev: dict[str, Any] = json.loads(line)
            if ev.get("event") == "planning_finished":
                result = ev.get("metrics", {})
                success = bool(ev.get("success"))
    if result is not None:
        result["success"] = 1.0 if success else 0.0
    return result


def _config_path(configs_dirs: list[Path], algo: str) -> Path | None:
    """First existing `<dir>/<algo>.yaml` across the given config dirs (algorithm
    filenames don't collide across categories, so first match is unambiguous)."""
    for d in configs_dirs:
        candidate = d / f"{algo}.yaml"
        if candidate.exists():
            return candidate
    return None


def _run_one(
    py: str, demos_dir: Path, config: Path, scenario_path: Path, algo: str
) -> Row:
    scenario = load_scenario(scenario_path)
    grid = load_map(scenario.map_path)
    if not grid.supports(_REQUIRED[algo]):
        return Row(scenario_path.name, algo, "incompatible", {})
    if algo in _NEEDS_PATH and not scenario.reference_path:
        return Row(scenario_path.name, algo, "incompatible", {})

    demo = demos_dir / f"demo_{algo}.py"
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tmp:
        trace_path = Path(tmp.name)
    cmd = [
        py, str(demo),
        "--map", scenario.map_path,
        "--scenario", str(scenario_path),
        "--params", str(config),
        "--trace", str(trace_path),
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            return Row(scenario_path.name, algo, "error", {})
        metrics = _final_metrics(trace_path)
    finally:
        # Pre-existing leak fix: a failed subprocess used to skip this cleanup and
        # leave the temp trace file behind (harmless individually, but bench runs a
        # full matrix per invocation).
        trace_path.unlink(missing_ok=True)
    if metrics is None:
        return Row(scenario_path.name, algo, "error", {})
    status = _local_status(metrics) if _is_local_algo(algo) else (
        "ok" if metrics.get("success", 0.0) >= 1.0 else "no_path"
    )
    return Row(scenario_path.name, algo, status, metrics)


def _render_global(rows: list[Row]) -> str:
    header = (
        "| scenario | algorithm | status | path_cost | expanded | samples | tree | runtime_sec |\n"
        "|---|---|---|---|---|---|---|---|\n"
    )
    lines = [header]
    for r in rows:
        m = r.metrics
        if r.status in ("incompatible", "error"):
            lines.append(f"| {r.scenario} | {r.algorithm} | {r.status} | - | - | - | - | - |\n")
            continue
        lines.append(
            f"| {r.scenario} | {r.algorithm} | {r.status} | "
            f"{m.get('path_cost', 0.0):.3f} | {int(m.get('expanded_nodes', 0))} | "
            f"{int(m.get('samples', 0))} | {int(m.get('tree_size', 0))} | "
            f"{m.get('runtime_sec', 0.0):.5f} |\n"
        )
    return "".join(lines)


def _render_local(rows: list[Row]) -> str:
    header = (
        "| scenario | algorithm | status | time_to_goal | distance | min_clearance "
        "| steps | runtime_sec |\n"
        "|---|---|---|---|---|---|---|---|\n"
    )
    lines = [header]
    for r in rows:
        m = r.metrics
        if r.status in ("incompatible", "error"):
            lines.append(f"| {r.scenario} | {r.algorithm} | {r.status} | - | - | - | - | - |\n")
            continue
        lines.append(
            f"| {r.scenario} | {r.algorithm} | {r.status} | "
            f"{m.get('time_to_goal', 0.0):.3f} | {m.get('distance_traveled', 0.0):.3f} | "
            f"{m.get('min_clearance', 0.0):.3f} | {int(m.get('steps', 0))} | "
            f"{m.get('runtime_sec', 0.0):.5f} |\n"
        )
    return "".join(lines)


def _render(rows: list[Row]) -> str:
    # Split by trace facts (see _is_local_row), not a category map. When a run only
    # touches one category (e.g. the historical global_planning-only invocation), the
    # report stays a single table — byte-identical to the pre-local-planning format.
    local_rows = [r for r in rows if _is_local_row(r)]
    global_rows = [r for r in rows if not _is_local_row(r)]
    title = "# navigation benchmark matrix\n\n"
    if global_rows and local_rows:
        return (
            title
            + "## Global planning\n\n" + _render_global(global_rows) + "\n"
            + "## Local planning\n\n" + _render_local(local_rows)
        )
    if local_rows:
        return title + _render_local(local_rows)
    return title + _render_global(global_rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="navigation benchmark matrix runner")
    parser.add_argument("--scenarios", default=str(_REPO_ROOT / "maps" / "scenarios"))
    parser.add_argument(
        "--configs", nargs="+",
        default=[str(_REPO_ROOT / "configs" / "global_planning"),
                 str(_REPO_ROOT / "configs" / "local_planning")],
        help="one or more algorithm-config dirs (algo id -> <dir>/<algo>.yaml); "
             "a single path is accepted for backward compatibility",
    )
    parser.add_argument("--demos", default=str(_REPO_ROOT / "python" / "demos"))
    parser.add_argument("--out", default=str(_REPO_ROOT / "out" / "report.md"))
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--algos", nargs="*", default=None, help="subset of algorithm ids")
    args = parser.parse_args()

    scenarios_dir = Path(args.scenarios)
    configs_dirs = [Path(p) for p in args.configs]
    demos_dir = Path(args.demos)
    algos = args.algos or [a for a in _ORDER if _config_path(configs_dirs, a) is not None]

    scenario_paths = sorted(scenarios_dir.glob("*.yaml"))
    rows: list[Row] = []
    for scenario_path in scenario_paths:
        for algo in algos:
            # Falls back to the first dir even when missing, so an explicit --algos
            # request with no matching config still reports "error" via the demo
            # subprocess (same behavior as the old single-dir CLI), not a silent skip.
            config = _config_path(configs_dirs, algo) or configs_dirs[0] / f"{algo}.yaml"
            rows.append(_run_one(args.python, demos_dir, config, scenario_path, algo))
            print(f"ran {scenario_path.name} x {algo} -> {rows[-1].status}", file=sys.stderr)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(_render(rows), encoding="utf-8")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
