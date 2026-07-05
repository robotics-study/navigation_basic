"""Shared plumbing for the four discrete graph-search planners.

Keeps path reconstruction and the finish-emit in one place; each algorithm still
owns its own frontier/behaviour (LIFO / FIFO / priority) in its own module.
"""

from __future__ import annotations

from navigation.core.trace import TraceRecorder
from navigation.core.types import Cell


def reconstruct(parent: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
    path = [goal]
    node = goal
    while node != start:
        node = parent[node]
        path.append(node)
    path.reverse()
    return path


def emit_finish(
    recorder: TraceRecorder | None,
    success: bool,
    path: list[Cell],
    cost: float,
    expanded: int,
    runtime: float,
) -> None:
    if recorder is not None:
        if success and path:
            recorder.path_found(path)
        recorder.planning_finished(
            success,
            {"runtime_sec": runtime, "path_cost": cost, "expanded_nodes": float(expanded)},
        )
