"""FCIT* — Fully Connected Informed Trees (Wilson, Thomason, Kingston, Kavraki & Gammell 2025, ICRA).

FCIT* observes that modern collision-checking is cheap enough that a batch
planner need not restrict its candidate graph to a shrinking radius-neighbour RGG
(as BIT*/AIT*/EIT* do to bound edge count). Instead it searches the **fully
connected** graph over the current informed batch — every accumulated sample
pairs with every other — and runs an informed best-first search over it directly,
validating edges lazily exactly like the radius-limited variants. Dropping the
radius trades more (cheap, un-collision-checked) candidate edges for a search that
can take shortcuts a radius graph would miss.

Mechanism implemented here (a faithful core, descending from the AIT*
reverse-search-heuristic idea, Strub & Gammell 2020):
  * Grow a persistent point array by drawing informed-ellipse samples per batch
    (Gammell, Srinivasa & Barfoot 2014); index 0 = start, 1 = goal.
  * Build the fully connected adjacency over ALL accumulated points, filtered by a
    persistent ``invalid_edges`` set that grows as motions are found in collision —
    the same adaptive lazy-validation feedback as the AIT*/EIT* lineage, applied to
    a complete graph instead of a radius graph.
  * Reverse search: Dijkstra from the goal over that filtered complete graph gives
    an admissible, adaptive cost-to-go heuristic ``h_hat`` for every vertex.
  * Forward search: a lazy-deletion best-first (A*) over ``g + h_hat`` that
    validates each edge with ``is_motion_valid`` only when it would improve a
    vertex, recording newly-invalid motions back into ``invalid_edges``.

Deliberate simplifications versus the full paper (documented so the scope is
explicit): the reverse search is recomputed from scratch each batch rather than
repaired incrementally, and ``g``/``parent``/open-heap are likewise rebuilt fresh
per batch (only ``c_best`` and ``invalid_edges`` persist and monotonically
improve/grow). The sample budget is kept modest because the complete graph has
O(n^2) edges; the paper develops a more sophisticated scheme for keeping eager
all-pairs evaluation cheap at scale, which this implementation does not reproduce.

Properties: probabilistically complete, anytime, asymptotically optimal.
"""

from __future__ import annotations

import heapq
import time

import numpy as np

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import informed_sample, path_length

_INF = float("inf")
_START = 0  # persistent index of the start (forward-search root)
_GOAL = 1  # persistent index of the goal sample (reverse-search source)


def _edge_key(a: int, b: int) -> tuple[int, int]:
    """Normalised undirected edge id so (a,b) and (b,a) share one ``invalid_edges`` entry."""
    return (a, b) if a < b else (b, a)


class FCITStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "fcit_star"

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
        rng = np.random.default_rng(self.params.get_int("seed"))

        # Persistent across batches: sample array (0=start, 1=goal), the adaptive
        # invalid-motion set, and the incumbent cost / path. Only these carry over —
        # every batch rebuilds its reverse heuristic and forward tree from scratch.
        points: list[Point] = [start, goal]
        invalid_edges: set[tuple[int, int]] = set()
        c_best = _INF
        best_path: list[Point] = []
        expanded = 0

        for _ in range(max_batches):
            c_best, best_path, batch_expanded = self._run_batch(
                space, start, goal, batch_size, rng, points, invalid_edges,
                c_best, best_path, recorder,
            )
            expanded += batch_expanded

        success = c_best < _INF
        path = best_path if success else []
        cost = path_length(space, path) if success else 0.0
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
        stats = PlanStats(
            expanded_nodes=expanded, samples=n, tree_size=n, iterations=expanded
        )
        return PlanResult(success, path, cost, stats)

    def _run_batch(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        batch_size: int,
        rng: np.random.Generator,
        points: list[Point],
        invalid_edges: set[tuple[int, int]],
        c_best: float,
        best_path: list[Point],
        recorder: TraceRecorder | None,
    ) -> tuple[float, list[Point], int]:
        # --- 1. grow the batch (informed once a solution exists) ----------------
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

        # --- 2. fully connected adjacency, minus known-invalid motions ----------
        # No radius: FCIT* pairs every sample with every other, trading a denser
        # candidate graph for a search that can find shortcuts a radius-limited RGG
        # (BIT*/AIT*) would miss (Wilson, Thomason, Kingston, Kavraki & Gammell 2025).
        nbr: list[list[int]] = [
            [j for j in range(n) if j != i and _edge_key(i, j) not in invalid_edges]
            for i in range(n)
        ]

        # --- 3. reverse search: Dijkstra from the goal gives h_hat --------------
        h_hat = self._reverse_search(space, points, nbr)

        # --- 4. forward search: lazy-validated best-first over g + h_hat --------
        g = [_INF] * n
        parent = [-1] * n
        closed = [False] * n
        g[_START] = 0.0
        open_heap: list[tuple[float, int]] = [(h_hat[_START], _START)]
        expanded = 0

        while open_heap:
            f, v = heapq.heappop(open_heap)
            if closed[v]:
                continue
            if f > g[v] + h_hat[v] + 1e-9:
                continue  # stale lazy-deletion entry: a cheaper g[v] superseded it
            closed[v] = True
            expanded += 1
            if v == _GOAL:
                break  # goal settled with its final cost for this batch's graph
            pv = points[v]
            for x in nbr[v]:
                if closed[x]:
                    continue
                edge_cost = space.distance(pv, points[x])
                tentative = g[v] + edge_cost
                if tentative >= g[x]:
                    continue  # routing through v cannot improve x
                if recorder is not None:
                    recorder.candidate_evaluated(points[x], tentative)
                ek = _edge_key(v, x)
                if ek in invalid_edges:
                    continue
                if not space.is_motion_valid(pv, points[x]):
                    invalid_edges.add(ek)  # adaptive feedback: never reconsider it
                    continue
                was_connected = g[x] < _INF
                g[x] = tentative
                parent[x] = v
                heapq.heappush(open_heap, (tentative + h_hat[x], x))
                if recorder is not None:
                    if was_connected:
                        recorder.rewire(points[x], pv)
                    else:
                        recorder.edge_added(points[x], pv, edge_cost)
                if x == _GOAL and g[x] < c_best:
                    c_best = g[x]
                    best_path = self._extract(points, parent)
                    if recorder is not None:
                        recorder.path_found(best_path)

        return c_best, best_path, expanded

    @staticmethod
    def _reverse_search(
        space: SamplingSpace[Point], points: list[Point], nbr: list[list[int]]
    ) -> list[float]:
        """Dijkstra from the goal over the filtered complete graph.

        Distances are a lower bound on the collision-free cost-to-go (the graph is a
        superset of the validated edges), so h_hat is admissible; and because it is a
        shortest-path metric on that graph it is also consistent for the forward
        search — the adaptive-heuristic idea of AIT* (Strub & Gammell 2020).
        """
        dist = [_INF] * len(points)
        settled = [False] * len(points)
        dist[_GOAL] = 0.0
        pq: list[tuple[float, int]] = [(0.0, _GOAL)]
        while pq:
            d, u = heapq.heappop(pq)
            if settled[u]:
                continue
            settled[u] = True
            pu = points[u]
            for v in nbr[u]:
                if settled[v]:
                    continue
                nd = d + space.distance(pu, points[v])
                if nd < dist[v]:
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
        return dist

    @staticmethod
    def _extract(points: list[Point], parent: list[int]) -> list[Point]:
        path: list[Point] = []
        node = _GOAL
        while node != -1:
            path.append(points[node])
            node = parent[node]
        path.reverse()
        return path
