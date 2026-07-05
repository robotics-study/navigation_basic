"""RRT — Rapidly-exploring Random Tree (LaValle 1998).

Grow a tree by sampling (with goal bias), extending the nearest node one step
toward the sample, and stopping at the first node that reaches the goal region.
Feasible, not optimal.
"""

from __future__ import annotations

import time

import numpy as np

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import Tree, _SamplingPlanner, path_length


class RRT(_SamplingPlanner):
    @property
    def name(self) -> str:
        return "rrt"

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        max_iter, step_size, goal_bias, goal_tol, seed = self._common_params()
        rng = np.random.default_rng(seed)
        tree = Tree(start)
        goal_idx = -1
        iterations = 0
        for _ in range(max_iter):
            iterations += 1
            q_rand = self._biased_sample(space, goal, goal_bias, rng)
            if recorder is not None:
                recorder.sample_drawn(q_rand)
            near_idx = tree.nearest(q_rand, space)
            q_near = tree.points[near_idx]
            q_new = space.steer(q_near, q_rand, step_size)
            if not space.is_motion_valid(q_near, q_new):
                continue
            step_cost = space.distance(q_near, q_new)
            new_idx = tree.add(q_new, near_idx, tree.cost[near_idx] + step_cost)
            if recorder is not None:
                recorder.edge_added(q_new, q_near, step_cost)
            if space.distance(q_new, goal) <= goal_tol and space.is_motion_valid(q_new, goal):
                goal_idx = tree.add(goal, new_idx, tree.cost[new_idx] + space.distance(q_new, goal))
                if recorder is not None:
                    recorder.edge_added(goal, q_new, space.distance(q_new, goal))
                break

        runtime = time.monotonic() - t0
        success = goal_idx != -1
        path = tree.path_to(goal_idx) if success else []
        cost = path_length(space, path) if success else 0.0
        if recorder is not None and success:
            recorder.path_found(path)
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
