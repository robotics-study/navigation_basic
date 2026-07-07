"""AIT* — Adaptively Informed Trees (Strub & Gammell 2020; extended 2022).

What differentiates AIT* from BIT* is the cost-to-go heuristic. BIT* uses the raw
straight-line distance ``h_hat(x) = ||x - goal||``, which ignores obstacles. AIT*
instead computes ``h_hat`` by a **reverse search over the current random geometric
graph (RGG)**: a heuristic that follows the graph's actual connectivity (detours
around a wall) rather than a straight line through it. Crucially it is *adaptive* —
edges found invalid during forward validation are permanently excluded from the
graph the reverse search runs over, so the heuristic self-corrects as obstacles
are discovered (the "lazy reverse search feeds the forward search" loop of Strub &
Gammell 2020, ICRA; extended in IJRR 2022).

Per batch: (1) draw informed samples and append to a persistent point array;
(2) build the RGG neighbour graph, filtering out every edge accumulated in the
persistent ``invalid_edges`` set; (3) reverse Dijkstra from the goal over that
filtered graph produces the adaptive heuristic ``h_hat`` (optimistic — it does not
collision-check, that is the forward search's job); (4) forward A* keyed on
``g[v] + h_hat[v]`` lazily validates each edge it relaxes, adding any invalid edge
to ``invalid_edges`` so it feeds back into every future batch's reverse search.

Implementation simplification (documented, deliberate): the full paper interleaves
a single *incremental* bidirectional LPA*-based search that repairs the reverse
tree and reuses forward ``g`` values across events. Here each batch instead
recomputes the reverse search and the forward ``g``/``parent`` from scratch over
all accumulated samples. This drops the LPA* incrementality (an optimisation of
*how* the searches are updated) while preserving AIT*'s defining behaviour — a
forward search guided by an obstacle-aware reverse heuristic that adapts to
discovered invalid edges — which is what makes it informed and adaptive. Same
guarantee class as BIT*: probabilistically complete, almost-surely asymptotically
optimal, anytime. ``c_best`` (the informed ellipse) and ``invalid_edges`` both
persist across batches and only improve/grow.
"""

from __future__ import annotations

import heapq
import time

import numpy as np

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import informed_sample, path_length, radius_neighbors, rgg_radius

_INF = float("inf")

# Edges are keyed order-independently: an edge is invalid regardless of the
# direction it was traversed when the collision was detected.
Edge = tuple[int, int]


def _edge(i: int, j: int) -> Edge:
    return (i, j) if i < j else (j, i)


def _reverse_search(
    space: SamplingSpace[Point],
    points: list[Point],
    nbr: list[list[int]],
    invalid_edges: set[Edge],
    goal_idx: int,
) -> list[float]:
    """Adaptive cost-to-go heuristic: Dijkstra from the goal over the RGG minus the
    known-invalid edges. Optimistic — it assumes remaining edges are collision-free
    (Strub & Gammell 2020); only the forward search validates, and its findings
    shrink this graph on the next batch."""
    n = len(points)
    h = [_INF] * n
    h[goal_idx] = 0.0
    heap: list[tuple[float, int]] = [(0.0, goal_idx)]
    settled = [False] * n
    while heap:
        d_u, u = heapq.heappop(heap)
        if settled[u]:
            continue
        settled[u] = True
        for w in nbr[u]:
            if _edge(u, w) in invalid_edges:
                continue
            nd = d_u + space.distance(points[u], points[w])
            if nd < h[w]:
                h[w] = nd
                heapq.heappush(heap, (nd, w))
    return h


def _reconstruct(points: list[Point], parent: list[int], goal_idx: int) -> list[Point]:
    path: list[Point] = []
    node = goal_idx
    while node != -1:
        path.append(points[node])
        node = parent[node]
    path.reverse()
    return path


class AITStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "ait_star"

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

        goal_idx = 1
        points: list[Point] = [start, goal]  # 0 = start (root), 1 = goal
        invalid_edges: set[Edge] = set()  # grows across batches: the adaptive feedback
        c_best = _INF
        expanded = 0
        # Last batch's forward-search result; the final incumbent is read from it.
        g: list[float] = [0.0, _INF]
        parent: list[int] = [-1, -1]

        for _ in range(max_batches):
            # --- 1. grow the RGG (informed once a solution exists) ----------------
            drawn = 0
            for _ in range(batch_size * 40):
                if drawn >= batch_size:
                    break
                q = informed_sample(space, start, goal, c_best, rng)
                if not space.is_state_valid(q):
                    continue
                points.append(q)
                drawn += 1
                if recorder is not None:
                    recorder.sample_drawn(q)

            n = len(points)
            radius = rgg_radius(gamma, n)
            nbr = radius_neighbors(space, points, radius)

            # --- 2+3. reverse search over the filtered graph = adaptive heuristic --
            h_hat = _reverse_search(space, points, nbr, invalid_edges, goal_idx)

            # --- 4. forward A* keyed on g + h_hat, lazily validating each edge -----
            g = [_INF] * n
            g[0] = 0.0
            parent = [-1] * n
            closed = [False] * n
            open_heap: list[tuple[float, int]] = [(h_hat[0], 0)]
            while open_heap:
                key, v = heapq.heappop(open_heap)
                # Lazy deletion: skip entries superseded by a cheaper g[v] (same
                # trick BIT* uses for its vertex queue).
                if closed[v] or key > g[v] + h_hat[v] + 1e-9:
                    continue
                closed[v] = True
                expanded += 1
                for x in nbr[v]:
                    if _edge(v, x) in invalid_edges:
                        continue
                    d = space.distance(points[v], points[x])
                    if not space.is_motion_valid(points[v], points[x]):
                        # Discovered invalid: exclude it from every future batch's
                        # reverse search — this is AIT*'s adaptive feedback loop.
                        invalid_edges.add(_edge(v, x))
                        continue
                    new_g = g[v] + d
                    if new_g < g[x]:
                        first = g[x] == _INF
                        g[x] = new_g
                        parent[x] = v
                        if recorder is not None:
                            # Emit candidate_evaluated only for feasible, improving
                            # edges (not every relaxed neighbour) so the trace stays
                            # renderable by replay.py — matches BIT*'s emit-on-accept
                            # scale rather than exploding on the batch-recomputed graph.
                            recorder.candidate_evaluated(points[x], new_g)
                            if first:
                                recorder.edge_added(points[x], points[v], d)
                            else:
                                recorder.rewire(points[x], points[v])
                        heapq.heappush(open_heap, (new_g + h_hat[x], x))
                        if x == goal_idx and new_g < c_best:
                            c_best = new_g
                            if recorder is not None:
                                recorder.path_found(_reconstruct(points, parent, goal_idx))

        # --- extract final incumbent ---------------------------------------------
        n = len(points)
        success = g[goal_idx] < _INF
        path = _reconstruct(points, parent, goal_idx) if success else []
        cost = path_length(space, path) if success else 0.0
        runtime = time.monotonic() - t0
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": cost,
                    "expanded_nodes": float(expanded),
                    "samples": float(n),
                    "tree_size": float(n),
                },
            )
        stats = PlanStats(expanded_nodes=expanded, samples=n, tree_size=n, iterations=expanded)
        return PlanResult(success, path, cost, stats)
