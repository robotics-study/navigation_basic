"""BIT* — Batch Informed Trees (Gammell, Srinivasa & Barfoot 2015).

Processes samples in batches, expanding an implicit random geometric graph in
order of estimated solution cost with an edge queue (best-first, like LPA*/A*
on the RGG). Collision checks are lazy — deferred to the moment an edge is
dequeued — and once a solution exists new batches are drawn from the informed
ellipse (Gammell et al. 2014), so search focuses on the region that can still
improve the incumbent. Anytime: keeps tightening the path across batches.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass

import numpy as np

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import path_length, radius_neighbors, rgg_radius

_INF = float("inf")


@dataclass
class _Tree:
    """Mutable BIT* state carried across batches (sample array + tree pointers).

    Index 0 = start (root), 1 = goal (a permanent sample). ``samples`` holds the
    indices not yet in the tree; ``g_t[i]`` is cost-to-come (inf while unconnected).
    """

    points: list[Point]
    g_t: list[float]
    parent: list[int]
    children: list[set[int]]
    in_tree: list[bool]
    samples: set[int]
    c_best: float = _INF
    expanded: int = 0

    @classmethod
    def rooted(cls, start: Point, goal: Point) -> _Tree:
        return cls(
            points=[start, goal],
            g_t=[0.0, _INF],
            parent=[-1, -1],
            children=[set(), set()],
            in_tree=[True, False],
            samples={1},
        )

    def add_sample(self, p: Point) -> int:
        idx = len(self.points)
        self.points.append(p)
        self.g_t.append(_INF)
        self.parent.append(-1)
        self.children.append(set())
        self.in_tree.append(False)
        self.samples.add(idx)
        return idx


class BITStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "bit_star"

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
        batch_size = self.params.get_int("batch_size")
        max_batches = self.params.get_int("max_batches")
        gamma = self.params.get_float("gamma")
        rng = np.random.default_rng(self.params.get_int("seed"))

        tree = _Tree.rooted(start, goal)
        for _ in range(max_batches):
            self._run_batch(space, start, goal, batch_size, gamma, rng, tree, recorder)

        # --- extract incumbent ------------------------------------------------
        success = tree.in_tree[1] and tree.g_t[1] < _INF
        path: list[Point] = []
        if success:
            node = 1
            while node != -1:
                path.append(tree.points[node])
                node = tree.parent[node]
            path.reverse()
        cost = path_length(space, path) if success else 0.0
        if success and recorder is not None:
            recorder.path_found(path)
        runtime = time.monotonic() - t0
        n = len(tree.points)
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": cost,
                    "expanded_nodes": float(tree.expanded),
                    "samples": float(n),
                    "tree_size": float(n),
                },
            )
        stats = PlanStats(
            expanded_nodes=tree.expanded, samples=n, tree_size=n, iterations=tree.expanded
        )
        return PlanResult(success, path, cost, stats)

    def _run_batch(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        batch_size: int,
        gamma: float,
        rng: np.random.Generator,
        tree: _Tree,
        recorder: TraceRecorder | None,
    ) -> None:
        points, g_t, parent, children, in_tree, samples = (
            tree.points, tree.g_t, tree.parent, tree.children, tree.in_tree, tree.samples
        )

        def h_hat(i: int) -> float:
            return space.distance(points[i], goal)

        def g_hat(i: int) -> float:
            return space.distance(start, points[i])

        def propagate(root: int) -> None:
            # Rewiring changed g_t[root]; push the delta to its subtree so queue
            # keys and the reported cost stay consistent.
            stack = [root]
            while stack:
                u = stack.pop()
                for c in children[u]:
                    g_t[c] = g_t[u] + space.distance(points[u], points[c])
                    stack.append(c)

        # --- prune samples that can no longer improve the incumbent -------------
        if tree.c_best < _INF:
            for x in [s for s in samples if g_hat(s) + h_hat(s) >= tree.c_best]:
                samples.discard(x)

        # --- draw a new batch (informed once a solution exists) -----------------
        drawn = 0
        for _ in range(batch_size * 40):
            if drawn >= batch_size:
                break
            q = self._sample(space, start, goal, tree.c_best, rng)
            if not space.is_state_valid(q):
                continue
            tree.add_sample(q)
            drawn += 1
            if recorder is not None:
                recorder.sample_drawn(q)

        n = len(points)
        radius = rgg_radius(gamma, n)
        nbr = radius_neighbors(space, points, radius)

        # --- queues: vertices to expand + candidate edges -----------------------
        q_v: list[tuple[float, int]] = []
        q_e: list[tuple[float, int, int]] = []
        for v in range(n):
            if in_tree[v]:
                heapq.heappush(q_v, (g_t[v] + h_hat(v), v))
        expanded_v: set[int] = set()

        def expand_vertex(v: int) -> None:
            for x in nbr[v]:
                if x in samples:
                    # Candidate edge to an unconnected sample.
                    if g_hat(v) + space.distance(points[v], points[x]) + h_hat(x) < tree.c_best:
                        key = g_t[v] + space.distance(points[v], points[x]) + h_hat(x)
                        heapq.heappush(q_e, (key, v, x))
                elif in_tree[x] and x != parent[v]:
                    # Candidate rewiring of an existing vertex through v.
                    cvw = space.distance(points[v], points[x])
                    if g_hat(v) + cvw + h_hat(x) < tree.c_best and g_t[v] + cvw < g_t[x]:
                        heapq.heappush(q_e, (g_t[v] + cvw + h_hat(x), v, x))

        def best_v() -> float:
            while q_v:
                key, v = q_v[0]
                if v in expanded_v or key > g_t[v] + h_hat(v) + 1e-9:
                    heapq.heappop(q_v)
                    continue
                return key
            return _INF

        def best_e() -> float:
            return q_e[0][0] if q_e else _INF

        while True:
            # Drain vertices whose expansion could beat the best queued edge.
            while q_v and best_v() <= best_e():
                _, v = heapq.heappop(q_v)
                if v in expanded_v:
                    continue
                expanded_v.add(v)
                expand_vertex(v)
            if not q_e:
                break
            edge_key, vm, xm = heapq.heappop(q_e)
            # Best remaining edge cannot improve the incumbent -> end batch.
            if edge_key >= tree.c_best:
                break
            # Can this edge improve the tree cost-to-come of x_m at all?
            if g_t[vm] + space.distance(points[vm], points[xm]) >= g_t[xm]:
                continue
            if not space.is_motion_valid(points[vm], points[xm]):
                continue
            edge_cost = space.distance(points[vm], points[xm])
            new_g = g_t[vm] + edge_cost
            if new_g + h_hat(xm) >= tree.c_best or new_g >= g_t[xm]:
                continue
            # Accept the edge: connect a sample or rewire a vertex under v_m.
            if in_tree[xm]:
                children[parent[xm]].discard(xm)
            else:
                samples.discard(xm)
                in_tree[xm] = True
            parent[xm] = vm
            g_t[xm] = new_g
            children[vm].add(xm)
            propagate(xm)
            tree.expanded += 1
            expanded_v.discard(xm)  # improved: allow re-expansion this batch
            heapq.heappush(q_v, (g_t[xm] + h_hat(xm), xm))
            if recorder is not None:
                recorder.edge_added(points[xm], points[vm], edge_cost)
            if in_tree[1] and g_t[1] < tree.c_best:
                tree.c_best = g_t[1]
                if recorder is not None:
                    recorder.candidate_evaluated(goal, tree.c_best)

    def _sample(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        c_best: float,
        rng: np.random.Generator,
    ) -> Point:
        # Before a solution, sample the whole space; after, sample the informed
        # ellipse with foci start/goal and transverse diameter c_best (Gammell
        # et al. 2014), so draws land only where the incumbent can still improve.
        c_min = space.distance(start, goal)
        if c_best >= _INF or c_best <= c_min:
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
        return (x, y)
