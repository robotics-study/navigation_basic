"""RRT* — asymptotically optimal RRT (Karaman & Frazzoli 2011).

Adds choose-parent (pick the min-cost feasible parent among near nodes) and
rewire (reroute near nodes through the new node when cheaper) to RRT. Anytime:
keep improving the incumbent goal path until the iteration budget is spent.
"""

from __future__ import annotations

import time

import numpy as np

from nav_study.core.capabilities import SamplingSpace
from nav_study.core.trace import TraceRecorder
from nav_study.core.types import PlanResult, PlanStats, Point

from ._sampling import Tree, _SamplingPlanner, insert_best_parent, path_length, rewire


class RRTStar(_SamplingPlanner):
    @property
    def name(self) -> str:
        return "rrt_star"

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
        rng = np.random.default_rng(seed)
        tree = Tree(start)
        # goal is not inserted into the tree (it must not become a growth/rewire
        # candidate); only its best parent index + cost are tracked.
        best_goal_parent = -1
        best_cost = float("inf")
        iterations = 0
        for _ in range(max_iter):
            iterations += 1
            q_rand = self._biased_sample(space, goal, goal_bias, rng)
            if recorder is not None:
                recorder.sample_drawn(q_rand)
            near_idx = tree.nearest(q_rand, space)
            q_new = space.steer(tree.points[near_idx], q_rand, step_size)
            if not space.is_motion_valid(tree.points[near_idx], q_new):
                continue

            neighborhood = tree.near(q_new, neighbor_radius, space)
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
