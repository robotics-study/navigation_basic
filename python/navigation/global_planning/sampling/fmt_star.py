"""FMT* — Fast Marching Tree (Janson, Schmerling, Clark & Pavone 2015).

Marches a single tree outward from the start over one fixed batch of samples,
in order of cost-to-come, using lazy dynamic programming: at each step it takes
the lowest-cost frontier node ``z`` and, for every unvisited sample near ``z``,
connects it through its locally cheapest open neighbour — checking collision on
only that one edge. One batch, no rewiring; the marching order alone yields the
optimal cost-to-come on the sampled graph.
"""

from __future__ import annotations

import heapq
import time

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import radius_neighbors, rgg_radius


class FMTStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "fmt_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.SAMPLING_SPACE}

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

        # Sample set: index 0 = start, 1 = goal, then free samples.
        points: list[Point] = [start, goal]
        for _ in range(num_samples * 20):
            if len(points) - 2 >= num_samples:
                break
            q = space.sample()
            if not space.is_state_valid(q):
                continue
            points.append(q)
            if recorder is not None:
                recorder.sample_drawn(q)
        n = len(points)
        radius = rgg_radius(gamma, n)
        neighbors = radius_neighbors(space, points, radius)

        cost = [float("inf")] * n
        parent = [-1] * n
        cost[0] = 0.0
        in_open = [False] * n
        in_open[0] = True
        unvisited: set[int] = set(range(1, n))
        # Frontier heap holds (cost, idx); FMT* never lowers an open node's cost,
        # so entries stay valid and lazy membership via in_open is enough.
        heap: list[tuple[float, int]] = []
        z = 0
        goal_idx = 1
        expanded = 0
        success = False

        while True:
            expanded += 1
            for x in neighbors[z]:
                if x not in unvisited:
                    continue
                best_y = -1
                best_c = float("inf")
                for y in neighbors[x]:
                    if not in_open[y]:
                        continue
                    c = cost[y] + space.distance(points[y], points[x])
                    if c < best_c:
                        best_c = c
                        best_y = y
                # Lazy collision check: only the locally optimal edge is tested; if
                # it collides, x stays unvisited and may connect from a later z.
                if best_y >= 0 and space.is_motion_valid(points[best_y], points[x]):
                    parent[x] = best_y
                    cost[x] = best_c
                    in_open[x] = True
                    unvisited.discard(x)
                    heapq.heappush(heap, (best_c, x))
                    if recorder is not None:
                        recorder.edge_added(points[x], points[best_y],
                                            space.distance(points[best_y], points[x]))
            in_open[z] = False  # z is now closed
            z = -1
            while heap:
                _, i = heapq.heappop(heap)
                if in_open[i]:
                    z = i
                    break
            if z < 0:
                break  # frontier exhausted: goal unreachable on this sample set
            if recorder is not None:
                recorder.node_expanded(points[z], cost[z])
            if z == goal_idx:
                success = True
                break

        path: list[Point] = []
        total = 0.0
        if success:
            node = goal_idx
            while node != -1:
                path.append(points[node])
                node = parent[node]
            path.reverse()
            total = cost[goal_idx]
            if recorder is not None:
                recorder.path_found(path)

        runtime = time.monotonic() - t0
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": total if success else 0.0,
                    "expanded_nodes": float(expanded),
                    "samples": float(n),
                    "tree_size": float(n),
                },
            )
        stats = PlanStats(expanded_nodes=expanded, samples=n, tree_size=n, iterations=expanded)
        return PlanResult(success, path, total if success else 0.0, stats)
