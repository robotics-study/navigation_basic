"""Shared plumbing for the sampling-based planners (RRT family).

Provides the search tree and common parameter handling. Each algorithm still
owns its own extension policy (plain / optimal / fast) in its own module.
"""

from __future__ import annotations

import numpy as np

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Point


class Tree:
    """Search tree over world Points: parallel arrays of point / parent / cost."""

    def __init__(self, root: Point) -> None:
        self.points: list[Point] = [root]
        self.parent: list[int] = [-1]
        self.cost: list[float] = [0.0]
        # children adjacency: rewire must propagate cost to the whole subtree,
        # otherwise descendants keep a stale (over-estimated) cumulative cost and
        # RRT* loses its optimality invariant (Karaman & Frazzoli 2011).
        self.children: list[list[int]] = [[]]

    def __len__(self) -> int:
        return len(self.points)

    def add(self, point: Point, parent_idx: int, cost: float) -> int:
        idx = len(self.points)
        self.points.append(point)
        self.parent.append(parent_idx)
        self.cost.append(cost)
        self.children.append([])
        if parent_idx >= 0:
            self.children[parent_idx].append(idx)
        return idx

    def reparent(
        self, child: int, new_parent: int, new_cost: float, space: SamplingSpace[Point]
    ) -> None:
        """Re-root ``child`` under ``new_parent`` and push the cost delta down its subtree."""
        old = self.parent[child]
        if old >= 0:
            self.children[old].remove(child)
        self.parent[child] = new_parent
        self.cost[child] = new_cost
        self.children[new_parent].append(child)
        stack = [child]
        while stack:
            u = stack.pop()
            for c in self.children[u]:
                self.cost[c] = self.cost[u] + space.distance(self.points[u], self.points[c])
                stack.append(c)

    def nearest(self, p: Point, space: SamplingSpace[Point]) -> int:
        best_idx = 0
        best_d = space.distance(self.points[0], p)
        for idx in range(1, len(self.points)):
            d = space.distance(self.points[idx], p)
            if d < best_d:
                best_d = d
                best_idx = idx
        return best_idx

    def near(self, p: Point, radius: float, space: SamplingSpace[Point]) -> list[int]:
        return [i for i in range(len(self.points)) if space.distance(self.points[i], p) <= radius]

    def path_to(self, idx: int) -> list[Point]:
        path: list[Point] = []
        node = idx
        while node != -1:
            path.append(self.points[node])
            node = self.parent[node]
        path.reverse()
        return path


def path_length(space: SamplingSpace[Point], path: list[Point]) -> float:
    """Ground-truth geometric length of a polyline (avoids stale rewired costs)."""
    return sum(space.distance(path[i], path[i + 1]) for i in range(len(path) - 1))


def insert_best_parent(
    space: SamplingSpace[Point],
    tree: Tree,
    q_new: Point,
    near_idx: int,
    neighborhood: list[int],
    recorder: TraceRecorder | None,
) -> int:
    """Choose-parent step (Karaman & Frazzoli 2011): min-cost feasible parent."""
    best_parent = near_idx
    best_cost = tree.cost[near_idx] + space.distance(tree.points[near_idx], q_new)
    for j in neighborhood:
        if not space.is_motion_valid(tree.points[j], q_new):
            continue
        # candidate_evaluated carries the neighbor and its route-through cost, so it
        # is emitted only once the edge is known feasible (DESIGN §7).
        c = tree.cost[j] + space.distance(tree.points[j], q_new)
        if recorder is not None:
            recorder.candidate_evaluated(tree.points[j], c)
        if c < best_cost:
            best_cost = c
            best_parent = j
    new_idx = tree.add(q_new, best_parent, best_cost)
    if recorder is not None:
        recorder.edge_added(
            q_new, tree.points[best_parent], space.distance(tree.points[best_parent], q_new)
        )
    return new_idx


def rewire(
    space: SamplingSpace[Point],
    tree: Tree,
    new_idx: int,
    q_new: Point,
    neighborhood: list[int],
    recorder: TraceRecorder | None,
) -> None:
    """Rewire step: reroute near nodes through ``q_new`` when it is cheaper."""
    for j in neighborhood:
        if j == tree.parent[new_idx]:
            continue
        if not space.is_motion_valid(q_new, tree.points[j]):
            continue
        rerouted = tree.cost[new_idx] + space.distance(q_new, tree.points[j])
        if rerouted < tree.cost[j]:
            tree.reparent(j, new_idx, rerouted, space)
            if recorder is not None:
                recorder.rewire(tree.points[j], q_new)


class _SamplingPlanner(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    """Base holding common params, RNG, and biased sampling."""

    def required_capabilities(self) -> set[Capability]:
        return {Capability.SAMPLING_SPACE}

    def _common_params(self) -> tuple[int, float, float, float, int]:
        return (
            self.params.get_int("max_iterations"),
            self.params.get_float("step_size"),
            self.params.get_float("goal_bias"),
            self.params.get_float("goal_tolerance"),
            self.params.get_int("seed"),
        )

    def _biased_sample(
        self, space: SamplingSpace[Point], goal: Point, goal_bias: float, rng: np.random.Generator
    ) -> Point:
        # Goal-biasing pulls the tree toward the goal (LaValle 1998).
        if rng.random() < goal_bias:
            return goal
        return space.sample()

    def _finish(
        self,
        recorder: TraceRecorder | None,
        success: bool,
        cost: float,
        expanded: int,
        samples: int,
        tree_size: int,
        iterations: int,
        runtime: float,
    ) -> None:
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": cost,
                    "expanded_nodes": float(expanded),
                    "samples": float(samples),
                    "tree_size": float(tree_size),
                    "iterations": float(iterations),
                },
            )
