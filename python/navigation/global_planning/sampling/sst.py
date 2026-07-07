"""SST / SST* — Stable Sparse RRT (Li, Littlefield & Bekris 2016).

A kinodynamic sampling planner that needs NO steering / boundary-value solver: it
grows a tree purely by forward-propagating a RANDOM control for a random duration
(unicycle dynamics) and collision-checking the resulting arc. The planner OWNS its
dynamics — the map only answers state / motion validity (``SamplingSpace``), so the
required capability stays {SAMPLING_SPACE} exactly as for the RRT family.

The "stable" + "sparse" behaviour comes from a WITNESS set with sparsification
radius delta_s: every witness keeps at most one ACTIVE representative — the
lowest-cost node in its delta_s ball — and dominated leaves are pruned. This bounds
the number of active nodes and makes the incumbent cost improve monotonically
(anytime), where a naive RRT tree would grow without bound. SST* additionally
shrinks delta_BN and delta_s over iterations to recover asymptotic optimality
(Li, Littlefield & Bekris 2016).
"""

from __future__ import annotations

import math
import time

import numpy as np

from navigation.core.capabilities import SamplingSpace
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

from ._sampling import _SamplingPlanner, path_length

# Waypoint spacing (m) for arc collision sampling. Small enough that the straight
# chord between consecutive unicycle waypoints stays well under a map cell, so
# is_motion_valid's supercover traversal catches every obstacle the curved arc
# crosses (the arc never deviates from its chord by more than this at this spacing).
_ARC_WAYPOINT_SPACING = 0.2

# SST* radius decay applied once per doubling of the iteration count (Li,
# Littlefield & Bekris 2016 §V): keep a wide BestNear / witness radius early for
# exploration, then tighten so the tree converges toward the optimum. A gentle
# 0.9 per doubling shrinks to ~0.2x over 2^14 iterations — fast enough to sharpen
# the solution, slow enough to keep connectivity.
_SST_STAR_SHRINK = 0.9


class SST(_SamplingPlanner):
    @property
    def name(self) -> str:
        return "sst"

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        max_iter = self.params.get_int("max_iterations")
        goal_bias = self.params.get_float("goal_bias")
        goal_tol = self.params.get_float("goal_tolerance")
        delta_bn0 = self.params.get_float("delta_bn")
        delta_s0 = self.params.get_float("delta_s")
        max_v = self.params.get_float("max_velocity")
        max_omega = self.params.get_float("max_omega")
        prop_min = self.params.get_float("prop_duration_min")
        prop_max = self.params.get_float("prop_duration_max")
        sst_star = self.params.get_bool("sst_star")
        rng = np.random.default_rng(self.params.get_int("seed"))

        # --- tree (parallel arrays; SST needs active/witness/pruning bookkeeping the
        # shared Tree does not model, so it keeps its own node arrays) --------------
        pt: list[Point] = [start]
        # Root heading faces the goal so early propagation is productive.
        th: list[float] = [math.atan2(goal[1] - start[1], goal[0] - start[0])]
        parent: list[int] = [-1]
        cost: list[float] = [0.0]
        children: list[list[int]] = [[]]
        # Incoming dense arc per node (parent-exclusive .. node-inclusive); [] for root.
        arc: list[list[Point]] = [[]]
        active_ids: set[int] = {0}

        # --- witness set: witness point + its active representative index ----------
        wpt: list[Point] = [start]
        wrep: list[int] = [0]

        def radii(it: int) -> tuple[float, float]:
            if not sst_star:
                return delta_bn0, delta_s0
            k = math.floor(math.log2(it + 2.0))
            scale = _SST_STAR_SHRINK**k
            return delta_bn0 * scale, delta_s0 * scale

        def best_near(s_sample: Point, delta_bn: float) -> int:
            # BestNear: min-cost active node within delta_bn of the sample; fall back
            # to the nearest active node when the ball is empty (Li et al. 2016).
            best = -1
            best_cost = math.inf
            for i in active_ids:
                if space.distance(pt[i], s_sample) <= delta_bn and cost[i] < best_cost:
                    best_cost = cost[i]
                    best = i
            if best != -1:
                return best
            near = -1
            near_d = math.inf
            for i in active_ids:
                d = space.distance(pt[i], s_sample)
                if d < near_d:
                    near_d = d
                    near = i
            return near

        def nearest_witness(p: Point) -> tuple[int, float]:
            best = 0
            best_d = space.distance(wpt[0], p)
            for i in range(1, len(wpt)):
                d = space.distance(wpt[i], p)
                if d < best_d:
                    best_d = d
                    best = i
            return best, best_d

        def propagate(from_idx: int) -> tuple[float, list[Point]] | None:
            # Monte-Carlo forward propagation of a unicycle (x, y, theta) under a
            # random constant control (v, omega) held for a random duration. Euler
            # integration; every waypoint is collision-checked on its (x, y)
            # projection and the chord to the previous one is supercover-tested.
            v = float(rng.uniform(0.0, max_v))
            omega = float(rng.uniform(-max_omega, max_omega))
            duration = float(rng.uniform(prop_min, prop_max))
            n_sub = max(2, math.ceil(v * duration / _ARC_WAYPOINT_SPACING))
            dt = duration / n_sub
            x, y = pt[from_idx]
            theta = th[from_idx]
            prev = pt[from_idx]
            waypoints: list[Point] = []
            for _ in range(n_sub):
                theta += omega * dt
                x += v * math.cos(theta) * dt
                y += v * math.sin(theta) * dt
                p: Point = (x, y)
                if not space.is_state_valid(p) or not space.is_motion_valid(prev, p):
                    return None
                waypoints.append(p)
                prev = p
            return theta, waypoints

        def add_node(p: Point, theta: float, par: int, c: float, wps: list[Point]) -> int:
            idx = len(pt)
            pt.append(p)
            th.append(theta)
            parent.append(par)
            cost.append(c)
            children.append([])
            arc.append(wps)
            children[par].append(idx)
            active_ids.add(idx)
            return idx

        def prune_leaf_chain(node: int) -> None:
            # Deactivate a dominated representative and drop it + any inactive leaf
            # ancestors from the tree so the active set stays bounded ("sparse").
            active_ids.discard(node)
            cur = node
            while cur != -1 and cur not in active_ids and not children[cur]:
                par = parent[cur]
                if par != -1:
                    children[par].remove(cur)
                cur = par

        def reconstruct(idx: int) -> list[Point]:
            segs: list[list[Point]] = []
            node = idx
            while parent[node] != -1:
                segs.append(arc[node])
                node = parent[node]
            path: list[Point] = [start]
            for seg in reversed(segs):
                path.extend(seg)
            return path

        best_goal = -1
        best_cost = math.inf
        total_added = 0
        iterations = 0
        for it in range(max_iter):
            iterations += 1
            delta_bn, delta_s = radii(it)
            s_sample = self._biased_sample(space, goal, goal_bias, rng)
            if recorder is not None:
                recorder.sample_drawn(s_sample)

            selected = best_near(s_sample, delta_bn)
            prop = propagate(selected)
            if prop is None:
                continue
            new_theta, waypoints = prop
            new_pt = waypoints[-1]
            new_cost = cost[selected] + path_length(space, [pt[selected], *waypoints])

            # IsNodeLocallyBest: locate (or create) the governing witness, then keep
            # the node only if it beats that witness's current representative.
            wi, wd = nearest_witness(new_pt)
            if wd > delta_s:
                wi = len(wpt)
                wpt.append(new_pt)
                wrep.append(-1)
            peer = wrep[wi]
            if peer != -1 and new_cost >= cost[peer]:
                continue

            ci = add_node(new_pt, new_theta, selected, new_cost, waypoints)
            total_added += 1
            if recorder is not None:
                prev = pt[selected]
                for w in waypoints:
                    recorder.edge_added(w, prev, space.distance(prev, w))
                    prev = w
            wrep[wi] = ci
            if peer != -1:
                # rewire marks the witness representative moving to the cheaper node,
                # so the viz shows sparsification (the old branch is pruned away).
                if recorder is not None:
                    recorder.rewire(new_pt, pt[peer])
                prune_leaf_chain(peer)

            if space.distance(new_pt, goal) <= goal_tol and new_cost < best_cost:
                best_cost = new_cost
                best_goal = ci
                if recorder is not None:
                    recorder.path_found(reconstruct(ci))

        runtime = time.monotonic() - t0
        success = best_goal != -1
        path = reconstruct(best_goal) if success else []
        result_cost = path_length(space, path) if success else 0.0
        active_count = len(active_ids)
        self._finish(
            recorder, success, result_cost, total_added, iterations, active_count, iterations,
            runtime,
        )
        stats = PlanStats(
            expanded_nodes=total_added,
            samples=iterations,
            iterations=iterations,
            tree_size=active_count,
        )
        return PlanResult(success, path, result_cost, stats)
