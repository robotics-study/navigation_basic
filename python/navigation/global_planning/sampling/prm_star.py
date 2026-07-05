"""PRM* — asymptotically optimal PRM (Karaman & Frazzoli 2011).

Identical to PRM except the connection radius is not fixed: it shrinks with the
sample count as r_n = γ·(log n / n)^(1/d) (d = 2). This keeps the expected
neighbour count at Θ(log n), which is exactly what buys almost-sure convergence
to the optimal path (a fixed radius either over-connects or, if too small,
breaks optimality).
"""

from __future__ import annotations

import time

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._roadmap import Roadmap, _RoadmapPlanner, connect, dijkstra
from ._sampling import rgg_radius


class PRMStar(_RoadmapPlanner):
    @property
    def name(self) -> str:
        return "prm_star"

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        num_samples = self.params.get_int("num_samples")
        gamma = self.params.get_float("gamma")

        roadmap = Roadmap()
        start_idx = roadmap.add_node(start)
        goal_idx = roadmap.add_node(goal)
        self._sample_free(space, roadmap, num_samples, recorder)
        # PRM* radius: computed once from the final node count (Karaman & Frazzoli 2011).
        radius = rgg_radius(gamma, len(roadmap))
        for idx in range(1, len(roadmap)):
            connect(space, roadmap, idx, radius, recorder)

        path, cost, expanded = dijkstra(roadmap, start_idx, goal_idx, recorder)
        runtime = time.monotonic() - t0
        success = len(path) > 0
        if success and recorder is not None:
            recorder.path_found(path)
        self._finish(recorder, success, cost, expanded, len(roadmap), runtime)
        stats = PlanStats(
            expanded_nodes=expanded, samples=len(roadmap), tree_size=len(roadmap), iterations=0
        )
        return PlanResult(success, path, cost, stats)
