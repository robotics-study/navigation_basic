"""RRT-Connect — bidirectional single-query planner (Kuffner & LaValle 2000).

Grow two trees, one rooted at start and one at goal. Each iteration EXTEND one
tree one step toward a uniform random sample, then greedily CONNECT the other
tree toward that new node; when CONNECT reaches it the trees meet and the path
is spliced. Bidirectional growth already targets the opposite tree, so no goal
bias is used. Feasible, not optimal.
"""

from __future__ import annotations

import time

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import Tree, _SamplingPlanner, path_length


class RRTConnect(_SamplingPlanner["SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "rrt_connect"

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        max_iter = self.params.get_int("max_iterations")
        step_size = self.params.get_float("step_size")
        goal_tol = self.params.get_float("goal_tolerance")
        # Every draw is a uniform space.sample() (no goal bias / directional RNG), so
        # sampling reproducibility is owned by the map's seed, not a planner-local RNG.
        tree_start = Tree(start)
        tree_goal = Tree(goal)
        # ta extends toward the sample; tb connects toward ta's new node. They swap
        # every iteration so both trees grow (Kuffner & LaValle 2000). Object identity
        # against tree_start decides splice orientation regardless of the swap parity.
        ta, tb = tree_start, tree_goal
        path: list[Point] = []
        iterations = 0
        for _ in range(max_iter):
            iterations += 1
            q_rand = space.sample()
            if recorder is not None:
                recorder.sample_drawn(q_rand)
            new_idx = self._extend(space, ta, q_rand, step_size, recorder)
            if new_idx is not None:
                tb_idx = self._connect(space, tb, ta.points[new_idx], step_size, goal_tol, recorder)
                # CONNECT 는 tb 를 q_new 의 goal_tol 안까지만 당기므로 접합 두 노드 사이에
                # 최대 goal_tol 구간이 남는다. 이 bridge 를 명시적으로 충돌 검사해야 얇은
                # 벽을 관통하는 경로가 성공으로 새어나가지 않는다 (Kuffner & LaValle 2000).
                if tb_idx is not None and space.is_motion_valid(
                    ta.points[new_idx], tb.points[tb_idx]
                ):
                    bridge = ta.path_to(new_idx) + list(reversed(tb.path_to(tb_idx)))
                    # bridge runs root(ta)->...->root(tb); reverse it when goal is the
                    # extending tree so the returned path always begins at start.
                    path = bridge if ta is tree_start else list(reversed(bridge))
                    if recorder is not None:
                        recorder.path_found(path)
                    break
            ta, tb = tb, ta

        runtime = time.monotonic() - t0
        success = len(path) > 0
        cost = path_length(space, path) if success else 0.0
        tree_size = len(tree_start) + len(tree_goal)
        expanded = tree_size - 2  # exclude both roots
        self._finish(recorder, success, cost, expanded, iterations, tree_size, iterations, runtime)
        stats = PlanStats(
            expanded_nodes=expanded,
            samples=iterations,
            iterations=iterations,
            tree_size=tree_size,
        )
        return PlanResult(success, path, cost, stats)

    def _extend(
        self,
        space: SamplingSpace[Point],
        tree: Tree,
        target: Point,
        step_size: float,
        recorder: TraceRecorder | None,
    ) -> int | None:
        # EXTEND (Kuffner & LaValle 2000): one step_size step from the tree's nearest
        # node toward target. Returns the new node index on a collision-free step
        # (Advanced), or None when the step is blocked (Trapped).
        near_idx = tree.nearest(target, space)
        q_near = tree.points[near_idx]
        q_new = space.steer(q_near, target, step_size)
        if not space.is_motion_valid(q_near, q_new):
            return None
        step_cost = space.distance(q_near, q_new)
        new_idx = tree.add(q_new, near_idx, tree.cost[near_idx] + step_cost)
        if recorder is not None:
            recorder.edge_added(q_new, q_near, step_cost)
        return new_idx

    def _connect(
        self,
        space: SamplingSpace[Point],
        tree: Tree,
        target: Point,
        step_size: float,
        goal_tol: float,
        recorder: TraceRecorder | None,
    ) -> int | None:
        # CONNECT (Kuffner & LaValle 2000): greedily EXTEND the tree toward the fixed
        # target until it Reaches (within goal_tol) or is Trapped. Terminates: steer
        # clamps to target once within step_size, so each Advanced step is monotonic
        # progress and Reached fires in at most ceil(dist/step_size)+1 steps.
        while True:
            new_idx = self._extend(space, tree, target, step_size, recorder)
            if new_idx is None:
                return None
            if space.distance(tree.points[new_idx], target) <= goal_tol:
                return new_idx
