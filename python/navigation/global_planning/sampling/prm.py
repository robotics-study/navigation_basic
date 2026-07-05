"""PRM — Probabilistic Roadmap (Kavraki, Švestka, Latombe & Overmars 1996).

Multi-query roadmap method used here single-query: sample free states, connect
each pair within a fixed ``connection_radius`` by a collision-free straight
motion, then answer start→goal with Dijkstra over the roadmap.
"""

from __future__ import annotations

import time

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._roadmap import Roadmap, _RoadmapPlanner, connect, dijkstra


class PRM(_RoadmapPlanner):
    @property
    def name(self) -> str:
        return "prm"

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        num_samples = self.params.get_int("num_samples")
        radius = self.params.get_float("connection_radius")

        roadmap = Roadmap()
        start_idx = roadmap.add_node(start)
        goal_idx = roadmap.add_node(goal)
        self._sample_free(space, roadmap, num_samples, recorder)
        # Fixed radius for every node (this is what distinguishes PRM from PRM*).
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
