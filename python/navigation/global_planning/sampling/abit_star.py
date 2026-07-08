"""ABIT* — Advanced Batch Informed Trees (Strub & Gammell 2020).

Builds on BIT* (Gammell, Srinivasa & Barfoot 2015) — same batch RGG, vertex/edge
queues, lazy edge collision checks, informed-ellipse sampling, prune, and subtree
cost propagation — and adds two mechanisms that trade a bounded, closing gap of
suboptimality for a much faster first solution and cheaper batches:

1. Inflation factor ε_infl ≥ 1: the queue keys inflate the cost-to-go heuristic
   (weighted-A*/ARA* over the RGG; Likhachev, Gordon & Thrun 2003), so early
   batches order edges greedily toward the goal and reach a first solution in far
   fewer edge processings. ε_infl decays inflation_factor → inflation_final across
   batches, so the last batch recovers BIT*'s admissible ordering.
2. Truncation factor ε_trunc ≥ 1: a batch stops processing edges once the best
   remaining edge cannot improve the incumbent by more than a factor ε_trunc,
   skipping the (expensive) lazy collision checks on edges that can only shave the
   last sliver of cost. ε_trunc decays truncation_factor → 1.0 across batches.

The admissibility gates that guarantee an accepted edge genuinely lowers the tree
cost-to-come (and the incumbent) are left un-inflated, so with ε_infl = ε_trunc = 1
the last batch reduces exactly to BIT* and the planner stays asymptotically
optimal as the schedule reaches 1.
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass

import numpy as np

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import informed_sample, path_length, radius_neighbors, rgg_radius

_INF = float("inf")


@dataclass
class _Tree:
    """Mutable ABIT* state carried across batches (sample array + tree pointers).

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


def _schedule(batch: int, max_batches: int, initial: float, final: float) -> float:
    """Linear per-batch decay initial → final (batch 0 → last), monotone in ``batch``.

    ARA*-style ε schedule (Likhachev, Gordon & Thrun 2003): start inflated for a
    quick first solution, relax to ``final`` on the last batch to recover optimality.
    A single batch keeps the initial value (nothing later can tighten it).
    """
    if max_batches <= 1:
        return initial
    frac = batch / (max_batches - 1)
    return initial + (final - initial) * frac


class ABITStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    @property
    def name(self) -> str:
        return "abit_star"

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
        inflation0 = self.params.get_float("inflation_factor")
        inflation_final = self.params.get_float("inflation_final")
        truncation0 = self.params.get_float("truncation_factor")
        rng = np.random.default_rng(self.params.get_int("seed"))

        tree = _Tree.rooted(start, goal)
        for batch in range(max_batches):
            # ε_infl decays to inflation_final; ε_trunc decays to 1.0 (no truncation)
            # so the final batch runs admissible, untruncated — i.e. exactly BIT*.
            eps_infl = _schedule(batch, max_batches, inflation0, inflation_final)
            eps_trunc = _schedule(batch, max_batches, truncation0, 1.0)
            self._run_batch(
                space, start, goal, batch_size, gamma, eps_infl, eps_trunc, rng, tree, recorder
            )

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
        eps_infl: float,
        eps_trunc: float,
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
        # Admissible bound (un-inflated): inflation must never drop a sample that a
        # later, less-inflated batch could still route through.
        if tree.c_best < _INF:
            for x in [s for s in samples if g_hat(s) + h_hat(s) >= tree.c_best]:
                samples.discard(x)

        # --- draw a new batch (informed once a solution exists) -----------------
        drawn = 0
        for _ in range(batch_size * 40):
            if drawn >= batch_size:
                break
            q = informed_sample(space, start, goal, tree.c_best, rng)
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
        # Keys inflate the cost-to-go term by ε_infl (weighted-A*/ARA* over the RGG,
        # Strub & Gammell 2020): early batches order greedily toward the goal.
        q_v: list[tuple[float, int]] = []
        q_e: list[tuple[float, int, int]] = []
        for v in range(n):
            if in_tree[v]:
                heapq.heappush(q_v, (g_t[v] + eps_infl * h_hat(v), v))
        expanded_v: set[int] = set()

        def expand_vertex(v: int) -> None:
            for x in nbr[v]:
                d = space.distance(points[v], points[x])
                if x in samples:
                    # Candidate edge to an unconnected sample. Enqueue gate stays
                    # admissible; ordering key is inflated.
                    if g_hat(v) + d + h_hat(x) < tree.c_best:
                        heapq.heappush(q_e, (g_t[v] + d + eps_infl * h_hat(x), v, x))
                elif in_tree[x] and x != parent[v]:
                    # Candidate rewiring of an existing vertex through v.
                    if g_hat(v) + d + h_hat(x) < tree.c_best and g_t[v] + d < g_t[x]:
                        heapq.heappush(q_e, (g_t[v] + d + eps_infl * h_hat(x), v, x))

        def best_v() -> float:
            while q_v:
                key, v = q_v[0]
                if v in expanded_v or key > g_t[v] + eps_infl * h_hat(v) + 1e-9:
                    heapq.heappop(q_v)
                    continue
                return key
            return _INF

        def best_e() -> float:
            return q_e[0][0] if q_e else _INF

        # Truncation threshold: stop the batch once no edge can pull the incumbent
        # below c_best / ε_trunc (Strub & Gammell 2020). ε_trunc = 1 → BIT*'s c_best.
        trunc_bound = tree.c_best / eps_trunc

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
            d_vm_xm = space.distance(points[vm], points[xm])
            # Admissible (un-inflated) estimate of the solution through this edge.
            a_key = g_t[vm] + d_vm_xm + h_hat(xm)
            # Truncation: the best-ordered edge can no longer improve past the bound.
            if a_key >= trunc_bound:
                break
            # Can this edge improve the tree cost-to-come of x_m at all?
            if g_t[vm] + d_vm_xm >= g_t[xm]:
                continue
            if not space.is_motion_valid(points[vm], points[xm]):
                continue
            edge_cost = d_vm_xm
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
            heapq.heappush(q_v, (g_t[xm] + eps_infl * h_hat(xm), xm))
            if recorder is not None:
                recorder.edge_added(points[xm], points[vm], edge_cost)
            if in_tree[1] and g_t[1] < tree.c_best:
                tree.c_best = g_t[1]
                trunc_bound = tree.c_best / eps_trunc
                if recorder is not None:
                    recorder.candidate_evaluated(goal, tree.c_best)
