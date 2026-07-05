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
    "rrt": Capability.SAMPLING_SPACE,
    "rrt_star": Capability.SAMPLING_SPACE,
    "fast_rrt": Capability.SAMPLING_SPACE,
}
_ORDER = ["bfs", "dijkstra", "astar", "rrt", "rrt_star", "fast_rrt"]


@dataclass
class Row:
    scenario: str
    algorithm: str
    status: str  # "ok" | "incompatible" | "no_path" | "error"
    metrics: dict[str, float]


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


def _run_one(
    py: str, demos_dir: Path, configs_dir: Path, scenario_path: Path, algo: str
) -> Row:
    scenario = load_scenario(scenario_path)
    grid = load_map(scenario.map_path)
    if not grid.supports(_REQUIRED[algo]):
        return Row(scenario_path.name, algo, "incompatible", {})

    config = configs_dir / f"{algo}.yaml"
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
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        return Row(scenario_path.name, algo, "error", {})
    metrics = _final_metrics(trace_path)
    trace_path.unlink(missing_ok=True)
    if metrics is None:
        return Row(scenario_path.name, algo, "error", {})
    status = "ok" if metrics.get("success", 0.0) >= 1.0 else "no_path"
    return Row(scenario_path.name, algo, status, metrics)


def _render(rows: list[Row]) -> str:
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
    return "# navigation benchmark matrix\n\n" + "".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="navigation benchmark matrix runner")
    parser.add_argument("--scenarios", default=str(_REPO_ROOT / "maps" / "scenarios"))
    parser.add_argument("--configs", default=str(_REPO_ROOT / "configs" / "global_planning"))
    parser.add_argument("--demos", default=str(_REPO_ROOT / "python" / "demos"))
    parser.add_argument("--out", default=str(_REPO_ROOT / "out" / "report.md"))
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--algos", nargs="*", default=None, help="subset of algorithm ids")
    args = parser.parse_args()

    scenarios_dir = Path(args.scenarios)
    configs_dir = Path(args.configs)
    demos_dir = Path(args.demos)
    algos = args.algos or [a for a in _ORDER if (configs_dir / f"{a}.yaml").exists()]

    scenario_paths = sorted(scenarios_dir.glob("*.yaml"))
    rows: list[Row] = []
    for scenario_path in scenario_paths:
        for algo in algos:
            rows.append(_run_one(args.python, demos_dir, configs_dir, scenario_path, algo))
            print(f"ran {scenario_path.name} x {algo} -> {rows[-1].status}", file=sys.stderr)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(_render(rows), encoding="utf-8")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
