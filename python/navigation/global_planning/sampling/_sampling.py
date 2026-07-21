"""Shared plumbing for the sampling-based planners (RRT family).

Provides the search tree and common parameter handling. Each algorithm still
owns its own extension policy (plain / optimal / fast) in its own module.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

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


def near_points(
    space: SamplingSpace[Point],
    points: list[Point],
    candidates: Iterable[int],
    query: Point,
    radius: float,
) -> list[int]:
    """Indices among ``candidates`` whose point lies within ``radius`` of ``query``.

    Batch planners (PRM / FMT* / BIT*) query a fixed sample array rather than an
    incremental tree, so near-neighbour lookup is a free function over an index
    set instead of a ``Tree`` method.
    """
    return [i for i in candidates if space.distance(points[i], query) <= radius]


def radius_neighbors(
    space: SamplingSpace[Point], points: list[Point], radius: float
) -> list[list[int]]:
    """Precompute, once per batch, each point's within-radius neighbour indices.

    Batch planners revisit near-sets many times over a fixed sample array; caching
    the O(n^2) radius graph up front avoids recomputing distances in the hot loop.
    """
    out: list[list[int]] = [[] for _ in points]
    n = len(points)
    for i in range(n):
        pi = points[i]
        for j in range(i + 1, n):
            if space.distance(pi, points[j]) <= radius:
                out[i].append(j)
                out[j].append(i)
    return out


def rgg_radius(gamma: float, n: int) -> float:
    """Random-geometric-graph connection radius r_n = γ·(log n / n)^(1/d), d = 2.

    Shared shrinking radius of the asymptotically optimal batch planners
    (PRM* / FMT* / BIT*): Karaman & Frazzoli (2011), Janson et al. (2015).
    """
    if n <= 1:
        return float("inf")
    return gamma * float(np.sqrt(np.log(n) / n))


def near_radius(mode: str, fixed_radius: float, gamma: float, n: int) -> float:
    """Near-radius selector for the RRT* family (RRT* / Informed RRT* / Fast-RRT).

    ``"shrinking"`` contracts the near-set with tree size via ``rgg_radius`` as in
    canonical RRT* (Karaman & Frazzoli 2011); any other declared mode (``"fixed"``)
    keeps the constant ``fixed_radius`` so default runs are unchanged.
    """
    if mode == "shrinking":
        return rgg_radius(gamma, n)
    return fixed_radius


def informed_sample(
    space: SamplingSpace[Point],
    start: Point,
    goal: Point,
    c_best: float,
    rng: np.random.Generator,
) -> Point:
    """Draw a state biased toward improving the incumbent solution.

    Before a solution exists, sample the whole space; after, sample the informed
    ellipse with foci start/goal and transverse diameter c_best (Gammell, Srinivasa
    & Barfoot 2014), so draws land only where the incumbent can still improve.
    Shared by every asymptotically-optimal batch/anytime planner (BIT*, Informed
    RRT*, and the AIT*/EIT*/FCIT* lineage) so the ellipse math and RNG draw order
    live in exactly one place.
    """
    c_min = space.distance(start, goal)
    if c_best >= float("inf") or c_best <= c_min:
        return space.sample()
    cx, cy = (start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0
    r1 = c_best / 2.0
    r2 = math.sqrt(max(c_best * c_best - c_min * c_min, 0.0)) / 2.0
    theta = math.atan2(goal[1] - start[1], goal[0] - start[0])
    ang = float(rng.uniform(0.0, 2.0 * math.pi))
    rad = math.sqrt(float(rng.random()))
    ux, uy = rad * math.cos(ang) * r1, rad * math.sin(ang) * r2
    x = cx + math.cos(theta) * ux - math.sin(theta) * uy
    y = cy + math.sin(theta) * ux + math.cos(theta) * uy
    # 타원이 맵 경계를 벗어나도 클립하지 않는다 — 범위 밖 표본은 downstream 검사가
    # 버리므로 정확성에는 무관하고 표본 몇 개의 효율만 잃는다.
    return (x, y)


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
