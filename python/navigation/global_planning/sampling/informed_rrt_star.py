"""Informed RRT* — RRT* with informed ellipse sampling (Gammell, Srinivasa & Barfoot 2014).

Identical to RRT* (choose-parent + rewire + anytime incumbent tracking, Karaman &
Frazzoli 2011); only the sampling step changes. Once an incumbent solution exists,
draws come from the informed ellipse (foci start/goal, transverse diameter = current
best cost) instead of the whole space, so post-solution samples land only where the
incumbent can still improve. This is the paper's entire contribution — the tree-growth
mechanics are unchanged — so it converges faster/tighter than RRT* empirically while
keeping the same asymptotic-optimality guarantee.
"""

from __future__ import annotations

import time

import numpy as np

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import (
    Tree,
    _SamplingPlanner,
    informed_sample,
    insert_best_parent,
    near_radius,
    path_length,
    rewire,
)


class InformedRRTStar(_SamplingPlanner["SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "informed_rrt_star"

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
        rng = np.random.default_rng(seed)
        tree = Tree(start)
        # goal is not inserted into the tree (it must not become a growth/rewire
        # candidate); only its best parent index + cost are tracked.
        best_goal_parent = -1
        best_cost = float("inf")
        iterations = 0
        for _ in range(max_iter):
            iterations += 1
            # Before a solution exists, sample uniformly (goal-biased) like RRT*;
            # after, draw from the informed ellipse so samples focus on the region
            # that can still beat the incumbent (Gammell, Srinivasa & Barfoot 2014).
            if best_goal_parent != -1:
                q_rand = informed_sample(space, start, goal, best_cost, rng)
            else:
                q_rand = self._biased_sample(space, goal, goal_bias, rng)
            if recorder is not None:
                recorder.sample_drawn(q_rand)
            near_idx = tree.nearest(q_rand, space)
            q_new = space.steer(tree.points[near_idx], q_rand, step_size)
            if not space.is_motion_valid(tree.points[near_idx], q_new):
                continue

            # shrinking 모드는 트리 크기 n 에 따라 근방 반경을 줄인다 (Karaman & Frazzoli
            # 2011); fixed(기본)는 상수 neighbor_radius 라 기존 동작을 보존한다.
            radius = near_radius(radius_mode, neighbor_radius, rgg_gamma, len(tree))
            neighborhood = tree.near(q_new, radius, space)
            new_idx = insert_best_parent(space, tree, q_new, near_idx, neighborhood, recorder)
            rewire(space, tree, new_idx, q_new, neighborhood, recorder)

            if space.distance(q_new, goal) <= goal_tol and space.is_motion_valid(q_new, goal):
                cand_cost = tree.cost[new_idx] + space.distance(q_new, goal)
                if cand_cost < best_cost:
                    best_cost = cand_cost
                    best_goal_parent = new_idx
                    if recorder is not None:
                        recorder.path_found(tree.path_to(new_idx) + [goal])

        runtime = time.monotonic() - t0
        success = best_goal_parent != -1
        path = tree.path_to(best_goal_parent) + [goal] if success else []
        cost = path_length(space, path) if success else 0.0
        self._finish(
            recorder, success, cost, len(tree) - 1, iterations, len(tree), iterations, runtime
        )
        stats = PlanStats(
            expanded_nodes=len(tree) - 1,
            samples=iterations,
            iterations=iterations,
            tree_size=len(tree),
        )
        return PlanResult(success, path, cost, stats)
