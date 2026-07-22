"""Fast-RRT (Wu et al. 2021, Applied Sciences 11(24):11777).

Improved-RRT + Fast-Optimal on top of an RRT*-style tree:
  * Fast-Sampling: reject a random sample within `reached_radius` of any existing
    tree node, so sampling concentrates on unreached space (lower search-time
    variance).
  * Random Steering: when the straight extension toward the sample is blocked,
    retry up to `steering_attempts` random directions (one step_size each) and
    take the first collision-free step — helps through narrow passages.
  * Fast-Optimal: once a feasible path exists, shortcut-prune it via the triangle
    inequality (drop waypoints whose bypass segment is collision-free) and keep
    the min-cost path; anytime, like RRT*.

Divergence from the paper (deliberate): Wu et al.'s framework re-initialises a
PLAIN RRT (no rewiring) per outer iteration and their Fast-Optimal FUSES the
resulting multiple paths at their crossing points (Alg. 3/4/7). This
implementation instead keeps ONE persistent RRT*-style tree (choose-parent +
rewire) and replaces the multi-path fusion with a single-path triangle-inequality
shortcut — same intent (cheap post-hoc path improvement), different mechanism;
asymptotic optimality here is carried by the RRT* rewiring, not by fusion.
Fast-Sampling and Random Steering follow the paper (Alg. 5/6, with the unbounded
resample loop capped by `steering_attempts`).
"""

from __future__ import annotations

import math
import time

import numpy as np

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import (
    Tree,
    _SamplingPlanner,
    insert_best_parent,
    near_radius,
    path_length,
    rewire,
)


class FastRRT(_SamplingPlanner["SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "fast_rrt"

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        max_iter, step_size, goal_bias, goal_tol, seed = self._common_params()
        neighbor_radius = self.params.get_float("neighbor_radius")
        radius_mode = self.params.get_string("radius_mode")
        rgg_gamma = self.params.get_float("rgg_gamma")
        reached_radius = self.params.get_float("reached_radius")
        steering_attempts = self.params.get_int("steering_attempts")
        rng = np.random.default_rng(seed)
        tree = Tree(start)
        best_path: list[Point] = []
        best_cost = float("inf")
        iterations = 0
        for _ in range(max_iter):
            iterations += 1
            q_rand = self._fast_sample(
                space, tree, goal, goal_bias, reached_radius, steering_attempts, rng
            )
            if recorder is not None:
                recorder.sample_drawn(q_rand)
            near_idx = tree.nearest(q_rand, space)
            q_near = tree.points[near_idx]
            q_new = self._random_steer(space, q_near, q_rand, step_size, steering_attempts, rng)
            if q_new is None:
                continue

            # shrinking 모드는 트리 크기 n 에 따라 근방 반경을 줄인다 (Karaman & Frazzoli
            # 2011); fixed(기본)는 상수 neighbor_radius 라 기존 동작을 보존한다.
            radius = near_radius(radius_mode, neighbor_radius, rgg_gamma, len(tree))
            neighborhood = tree.near(q_new, radius, space)
            new_idx = insert_best_parent(space, tree, q_new, near_idx, neighborhood, recorder)
            rewire(space, tree, new_idx, q_new, neighborhood, recorder)

            if space.distance(q_new, goal) <= goal_tol and space.is_motion_valid(q_new, goal):
                # goal is appended to the extracted path, never inserted into the
                # tree, so it never becomes a nearest/near/rewire candidate.
                raw_path = tree.path_to(new_idx) + [goal]
                pruned = self._shortcut(space, raw_path)
                pruned_cost = path_length(space, pruned)
                if pruned_cost < best_cost:
                    best_cost = pruned_cost
                    best_path = pruned
                    if recorder is not None:
                        recorder.path_found(best_path)

        runtime = time.monotonic() - t0
        success = len(best_path) > 0
        cost = best_cost if success else 0.0
        self._finish(
            recorder, success, cost, len(tree) - 1, iterations, len(tree), iterations, runtime
        )
        stats = PlanStats(
            expanded_nodes=len(tree) - 1,
            samples=iterations,
            iterations=iterations,
            tree_size=len(tree),
        )
        return PlanResult(success, best_path, cost, stats)

    def _fast_sample(
        self,
        space: SamplingSpace[Point],
        tree: Tree,
        goal: Point,
        goal_bias: float,
        reached_radius: float,
        steering_attempts: int,
        rng: np.random.Generator,
    ) -> Point:
        # Fast-Sampling (Wu et al. 2021): a goal-biased draw is always accepted so
        # the tree can still reach the goal; a free draw is rejected while inside any
        # node's reached_radius (bounded by steering_attempts) to concentrate on
        # unreached space and cut search-time variance.
        if rng.random() < goal_bias:
            return goal
        q = space.sample()
        for _ in range(steering_attempts):
            if all(space.distance(q, node) > reached_radius for node in tree.points):
                break
            q = space.sample()
        return q

    def _random_steer(
        self,
        space: SamplingSpace[Point],
        q_near: Point,
        q_rand: Point,
        step_size: float,
        steering_attempts: int,
        rng: np.random.Generator,
    ) -> Point | None:
        q_new = space.steer(q_near, q_rand, step_size)
        if space.is_motion_valid(q_near, q_new):
            return q_new
        # Random Steering: probe random directions to escape a blocked extension.
        for _ in range(steering_attempts):
            angle = float(rng.uniform(0.0, 2.0 * math.pi))
            cand = (
                q_near[0] + step_size * math.cos(angle),
                q_near[1] + step_size * math.sin(angle),
            )
            if space.is_motion_valid(q_near, cand):
                return cand
        return None

    def _shortcut(self, space: SamplingSpace[Point], path: list[Point]) -> list[Point]:
        # Fast-Optimal: single greedy pass that jumps from each kept waypoint to
        # the farthest later waypoint reachable by a valid straight segment. Kept
        # byte-for-byte equivalent to the C++ FastRrtPlanner::shortcut_prune so the
        # cross-language benchmark compares the same shortcut on occluded paths.
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not space.is_motion_valid(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out
