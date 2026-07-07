"""EIT* — Effort Informed Trees (Strub & Gammell 2022, IJRR).

EIT* extends the AIT* idea — a reverse search over the random geometric graph
(RGG) produces an adaptive cost-to-go heuristic that a forward best-first search
consumes, with edges found in collision fed back so the heuristic adapts — by
*also* estimating the **validation effort** of the remaining path. Among
candidates of near-equal cost the forward search then prefers the ones that are
cheaper to collision-check (fewer / shorter sub-segments), surfacing feasible
solutions sooner. Strub & Gammell (2022) motivate this by the observation that
in high-dimensional or expensive-to-validate spaces the collision checker, not
the graph search, dominates runtime.

What this module implements (the faithful core):
- Per batch, grow the RGG exactly as AIT*/BIT*: draw ``batch_size`` informed
  samples (Gammell et al. 2014 ellipse), build the within-radius neighbour graph,
  and filter it against a set of edges already found in collision that persists
  and only grows across batches (the adaptive edge-invalidation feedback of the
  AIT* lineage).
- Reverse search: two independent single-criterion Dijkstra passes from the goal
  over the filtered graph — ``h_hat`` (cost-to-go, edge weight ``distance``) and
  ``e_hat`` (effort-to-go, edge weight ``effort``).
- Forward search: lazy-deletion best-first keyed by the lexicographic pair
  ``(g + h_hat, effort_g + e_hat)`` — cost primary, effort the tie-break, exactly
  the "solutions of equal cost differentiated by effort" ordering of the paper.
  Motions are collision-checked lazily on dequeue; an invalid motion is recorded
  in the persistent invalid-edge set and skipped.

Per-edge effort needs no new capability: ``effort(u, v)`` = the number of
``step_size``-sized sub-segments a discretized motion validator would check =
``max(1, round(distance(u, v) / step_size))``. This is a proxy for collision-check
cost that reads only the ``SamplingSpace`` distance, never map internals.

Explicit simplifications vs. the full paper (Strub & Gammell 2022, IJRR):
- The reverse search is recomputed from scratch each batch over the accumulated
  sample array rather than repaired incrementally (LPA*-style) as in the paper;
  only ``c_best`` and the invalid-edge set carry across batches.
- Cost and effort heuristics come from two clean independent single-criterion
  Dijkstra passes rather than the paper's more integrated joint treatment.
- Effort is the simple ``distance / step_size`` discretization proxy, not a
  learned or measured validator-cost model.

Properties: probabilistically complete, almost-surely asymptotically optimal,
anytime; additionally biases toward cheaper-to-validate solutions among
near-equal-cost candidates.
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

# Undirected edge key: order-independent so (u,v) and (v,u) hit the same entry in
# the persistent invalid-edge set.
_Edge = tuple[int, int]


def _edge_key(a: int, b: int) -> _Edge:
    return (a, b) if a < b else (b, a)


def _dijkstra_from(
    source: int,
    adjacency: list[list[int]],
    weight: list[list[float]],
) -> list[float]:
    """Single-source shortest cost over a fixed undirected graph.

    Runs one clean single-criterion relaxation; EIT* invokes it twice from the
    goal (distance weights, then effort weights) to build the two reverse
    heuristics independently (Strub & Gammell 2022).
    """
    n = len(adjacency)
    dist = [_INF] * n
    dist[source] = 0.0
    heap: list[tuple[float, int]] = [(0.0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for k, v in enumerate(adjacency[u]):
            nd = d + weight[u][k]
            if nd < dist[v]:
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return dist


class EITStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "eit_star"

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
        step_size = self.params.get_float("step_size")
        rng = np.random.default_rng(self.params.get_int("seed"))

        # Sample array grows across batches: index 0 = start, 1 = goal (permanent).
        points: list[Point] = [start, goal]
        start_idx, goal_idx = 0, 1
        invalid_edges: set[_Edge] = set()
        c_best = _INF
        expanded = 0
        # Forward-search cost/effort/parent from the final batch — the incumbent is
        # read off these once batches are exhausted (batch-recomputed, per the
        # documented simplification of the AIT*/EIT* reverse search).
        g: list[float] = [_INF, _INF]
        parent: list[int] = [-1, -1]

        for _ in range(max_batches):
            g, parent, c_best, expanded = self._run_batch(
                space, start, goal, start_idx, goal_idx, batch_size, gamma,
                step_size, rng, points, invalid_edges, c_best, expanded, recorder,
            )

        # --- extract incumbent from the final batch's forward tree ----------------
        success = g[goal_idx] < _INF
        path: list[Point] = []
        if success:
            node = goal_idx
            while node != -1:
                path.append(points[node])
                node = parent[node]
            path.reverse()
        cost = path_length(space, path) if success else 0.0
        if success and recorder is not None:
            recorder.path_found(path)

        runtime = time.monotonic() - t0
        n = len(points)
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

    def _run_batch(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        start_idx: int,
        goal_idx: int,
        batch_size: int,
        gamma: float,
        step_size: float,
        rng: np.random.Generator,
        points: list[Point],
        invalid_edges: set[_Edge],
        c_best: float,
        expanded: int,
        recorder: TraceRecorder | None,
    ) -> tuple[list[float], list[int], float, int]:
        # --- draw a new batch (informed once a solution exists) -------------------
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

        def effort(a: int, b: int) -> int:
            # Number of step_size-sized sub-segments a discretized validator would
            # check on edge (a,b): a capability-free proxy for collision-check cost
            # (Strub & Gammell 2022).
            return max(1, round(space.distance(points[a], points[b]) / step_size))

        # --- filtered adjacency + parallel distance/effort weights ----------------
        adjacency: list[list[int]] = [[] for _ in range(n)]
        dist_w: list[list[float]] = [[] for _ in range(n)]
        eff_w: list[list[float]] = [[] for _ in range(n)]
        for u in range(n):
            for v in nbr[u]:
                if _edge_key(u, v) in invalid_edges:
                    continue
                adjacency[u].append(v)
                dist_w[u].append(space.distance(points[u], points[v]))
                eff_w[u].append(float(effort(u, v)))

        # --- reverse search: two independent Dijkstra passes from the goal --------
        h_hat = _dijkstra_from(goal_idx, adjacency, dist_w)
        e_hat = _dijkstra_from(goal_idx, adjacency, eff_w)

        # --- forward search: lazy-deletion best-first over (cost, effort) ---------
        g = [_INF] * n
        effort_g = [_INF] * n
        parent = [-1] * n
        closed = [False] * n
        g[start_idx] = 0.0
        effort_g[start_idx] = 0.0
        # Heap key: (g + h_hat, effort_g + e_hat, idx). Python tuples compare
        # lexicographically, so cost is primary and effort the tie-break exactly as
        # the paper intends; idx makes the order deterministic.
        heap: list[tuple[float, float, int]] = [(h_hat[start_idx], e_hat[start_idx], start_idx)]

        while heap:
            _, _, v = heapq.heappop(heap)
            if closed[v]:
                continue
            closed[v] = True
            expanded += 1
            for x in adjacency[v]:
                d = space.distance(points[v], points[x])
                if recorder is not None:
                    # Cost is the primary reported metric for a candidate.
                    recorder.candidate_evaluated(points[x], g[v] + d)
                if not space.is_motion_valid(points[v], points[x]):
                    invalid_edges.add(_edge_key(v, x))
                    continue
                new_g = g[v] + d
                new_effort = effort_g[v] + float(effort(v, x))
                # Lexicographic acceptance: cost primary, cumulative effort the
                # tie-break, consistent with the priority-queue ordering.
                if (new_g, new_effort) < (g[x], effort_g[x]):
                    first = parent[x] == -1
                    g[x] = new_g
                    effort_g[x] = new_effort
                    parent[x] = v
                    closed[x] = False  # improved: allow re-expansion this batch
                    heapq.heappush(heap, (new_g + h_hat[x], new_effort + e_hat[x], x))
                    if recorder is not None:
                        if first:
                            recorder.edge_added(points[x], points[v], d)
                        else:
                            recorder.rewire(points[x], points[v])
                    if x == goal_idx and new_g < c_best:
                        c_best = new_g
                        if recorder is not None:
                            recorder.path_found(_reconstruct(points, parent, goal_idx))

        return g, parent, c_best, expanded


def _reconstruct(points: list[Point], parent: list[int], goal_idx: int) -> list[Point]:
    path: list[Point] = []
    node = goal_idx
    while node != -1:
        path.append(points[node])
        node = parent[node]
    path.reverse()
    return path
