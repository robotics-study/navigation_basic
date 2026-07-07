"""Kinodynamic RRT* — RRT* for systems with differential constraints.

Webb & van den Berg (2013), "Kinodynamic RRT*: Asymptotically Optimal Motion
Planning for Robots with Linear Dynamics". Unlike geometric RRT*, edges are not
straight segments but the trajectory of a fixed-final-state, free-final-time
optimal controller for a controllable linear system x' = Ax + Bu with running
cost J = ∫ (1 + uᵀR u) dt. That optimal-arrival cost is used as BOTH the
nearest-neighbour metric and the choose-parent / rewire cost, exactly as
geometric RRT* uses Euclidean distance — which is what makes the tree grow and
straighten in the *cost* geometry induced by the dynamics rather than in
Euclidean space.

The planner OWNS its dynamics: a 2D double integrator with state (x, y, vx, vy)
and control = acceleration. It depends only on the ``SamplingSpace`` capability
(``is_state_valid`` / ``is_motion_valid`` on the (x, y) projection) — the map
knows nothing about velocity or the controller. Start and goal are lifted to
rest states (velocity 0); the returned path is the (x, y) projection of the
concatenated optimal trajectories, densified for collision checking and viz.

Steering closed form (double integrator). With
    A = [[0, I₂],[0, 0]],  B = [[0],[I₂]],  R = r·I₂,
A is nilpotent (A² = 0) so exp(A t) = I + A t, and the weighted controllability
Gramian decouples per axis into the 2×2 block (1/r)[[t³/3, t²/2],[t²/2, t]].
The min-effort cost-to-go for arrival time t is therefore, summed over axes with
a = p₁−p₀, Δ = v₁−v₀ (v₀,v₁ the per-axis endpoint velocities),
    c(t) = t + r·( C₃/t³ + C₂/t² + C₁/t ),
    C₃ = Σ 12 a²,  C₂ = Σ −12 a (v₀+v₁),  C₁ = Σ 4 (v₀²+v₀v₁+v₁²).
Setting c'(t)=0 and clearing t⁴ gives the (already depressed) quartic
    t⁴ − r·C₁·t² − 2r·C₂·t − 3r·C₃ = 0,
whose optimal arrival time τ* is the positive real root minimising c(t). The
optimal position trajectory realising that minimum is, per axis, the unique
cubic through (p₀,v₀) at 0 and (p₁,v₁) at τ* — the minimum-∫‖u‖² Hermite
interpolant (Webb & van den Berg 2013).
"""

from __future__ import annotations

import math
import time
from collections.abc import Iterable

import numpy as np

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import PlanResult, PlanStats, Point

# Double-integrator planning state: world position + velocity (x, y, vx, vy). The
# planner works in this 4D space; only the (x, y) projection is ever handed to the
# SamplingSpace map, which knows nothing about velocity.
State4 = tuple[float, float, float, float]

# A connection is directional (parent → child): its optimal cost, arrival time τ*,
# and the (x, y) waypoints of the propagated trajectory (parent-exclusive).
_EPS = 1e-9
# Nearest-neighbour is selected by the *exact* optimal cost, but computing it against
# every node each iteration (a quartic solve per node) is intractable in Python. Since
# the double-integrator cost is dominated by the position gap, a Euclidean prefilter to
# the K closest nodes preserves the true optimum among candidates with high probability;
# the winner is then chosen by exact optimal cost. K-nearest RRT* is itself an analysed
# variant (Karaman & Frazzoli 2011).
_NEAREST_CANDIDATES = 16
# Cap the choose-parent / rewire neighbourhood to the K closest within neighbor_radius
# so per-iteration work stays bounded on dense trees (k-nearest RRT* variant).
_MAX_NEIGHBORS = 16
# Trajectory collision sampling: one waypoint per this many metres of straight
# separation (is_motion_valid supercover fills each sub-segment), floored/capped.
_COLLISION_SPACING = 0.3
_MIN_WAYPOINTS = 4
_MAX_WAYPOINTS = 64


def _steer_coeffs(x0: State4, x1: State4) -> tuple[float, float, float]:
    """Per-axis-summed (C₁, C₂, C₃) of the double-integrator cost-to-go c(t)."""
    c1 = c2 = c3 = 0.0
    # x-axis then y-axis: position gap a, endpoint velocities v0, v1.
    for a, v0, v1 in ((x1[0] - x0[0], x0[2], x1[2]), (x1[1] - x0[1], x0[3], x1[3])):
        c3 += 12.0 * a * a
        c2 += -12.0 * a * (v0 + v1)
        c1 += 4.0 * (v0 * v0 + v0 * v1 + v1 * v1)
    return c1, c2, c3


def _cost_at(t: float, r: float, c1: float, c2: float, c3: float) -> float:
    return t + r * (c3 / (t * t * t) + c2 / (t * t) + c1 / t)


def optimal_cost(x0: State4, x1: State4, r: float) -> tuple[float, float]:
    """Fixed-final-state free-final-time optimal cost and arrival time τ* (Webb &
    van den Berg 2013). Returns (cost, tau); (0, 0) when the states coincide."""
    c1, c2, c3 = _steer_coeffs(x0, x1)
    if abs(c1) < _EPS and abs(c2) < _EPS and abs(c3) < _EPS:
        return 0.0, 0.0
    # c'(t)=0 cleared by t⁴ — a depressed quartic (no cubic term). numpy.roots via the
    # companion matrix; take the positive real root that minimises c(t).
    roots = np.roots([1.0, 0.0, -r * c1, -2.0 * r * c2, -3.0 * r * c3])
    best_cost = math.inf
    best_tau = 0.0
    for root in roots:
        if abs(root.imag) > _EPS * (1.0 + abs(root.real)):
            continue
        t = float(root.real)
        if t <= _EPS:
            continue
        c = _cost_at(t, r, c1, c2, c3)
        if c < best_cost:
            best_cost = c
            best_tau = t
    return best_cost, best_tau


def _hermite_xy(x0: State4, x1: State4, tau: float, s: float) -> Point:
    """Optimal (min-∫‖u‖²) cubic position at normalised time s∈[0,1] along τ*."""
    # Cubic Hermite basis on [0, tau]; the tangents are the endpoint velocities
    # scaled by tau (the unique cubic meeting all four boundary conditions).
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    x = h00 * x0[0] + h10 * tau * x0[2] + h01 * x1[0] + h11 * tau * x1[2]
    y = h00 * x0[1] + h10 * tau * x0[3] + h01 * x1[1] + h11 * tau * x1[3]
    return (x, y)


def _trajectory_xy(x0: State4, x1: State4, tau: float) -> list[Point]:
    """(x, y) waypoints of the optimal trajectory, parent-exclusive (s>0..1)."""
    gap = math.hypot(x1[0] - x0[0], x1[1] - x0[1])
    n = max(_MIN_WAYPOINTS, math.ceil(gap / _COLLISION_SPACING))
    n = min(_MAX_WAYPOINTS, n)
    return [_hermite_xy(x0, x1, tau, k / n) for k in range(1, n + 1)]


class _KinoTree:
    """Search tree over double-integrator states (parallel arrays), mirroring the
    geometric RRT* tree but keyed on optimal-control cost instead of Euclidean
    distance. Each node stores its incoming edge cost + dense trajectory so a
    rewire can propagate cumulative cost through a subtree without re-solving the
    steering, and path reconstruction can emit the true curved trajectory."""

    def __init__(self, root: State4) -> None:
        self.states: list[State4] = [root]
        self.parent: list[int] = [-1]
        self.cost: list[float] = [0.0]
        self.edge_cost: list[float] = [0.0]
        self.incoming: list[list[Point]] = [[]]
        self.children: list[list[int]] = [[]]

    def __len__(self) -> int:
        return len(self.states)

    def add(self, state: State4, parent_idx: int, edge_cost: float, traj: list[Point]) -> int:
        idx = len(self.states)
        self.states.append(state)
        self.parent.append(parent_idx)
        self.cost.append(self.cost[parent_idx] + edge_cost)
        self.edge_cost.append(edge_cost)
        self.incoming.append(traj)
        self.children.append([])
        self.children[parent_idx].append(idx)
        return idx

    def positions(self) -> np.ndarray:
        return np.asarray([(s[0], s[1]) for s in self.states], dtype=float)

    def reparent(self, child: int, new_parent: int, edge_cost: float, traj: list[Point]) -> None:
        old = self.parent[child]
        if old >= 0:
            self.children[old].remove(child)
        self.parent[child] = new_parent
        self.edge_cost[child] = edge_cost
        self.incoming[child] = traj
        self.cost[child] = self.cost[new_parent] + edge_cost
        self.children[new_parent].append(child)
        # Push the cost delta down the subtree (edge costs are unchanged by a rewire
        # of an ancestor; only the cumulative sums shift).
        stack = [child]
        while stack:
            u = stack.pop()
            for c in self.children[u]:
                self.cost[c] = self.cost[u] + self.edge_cost[c]
                stack.append(c)

    def path_xy_to(self, idx: int) -> list[Point]:
        """Dense (x, y) polyline root→idx: root position + each incoming trajectory."""
        chain: list[int] = []
        node = idx
        while node != -1:
            chain.append(node)
            node = self.parent[node]
        chain.reverse()
        root = self.states[chain[0]]
        path: list[Point] = [(root[0], root[1])]
        for node in chain[1:]:
            path.extend(self.incoming[node])
        return path


class KinodynamicRRTStar(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._max_iter = params.get_int("max_iterations")
        self._goal_bias = params.get_float("goal_bias")
        self._goal_tol = params.get_float("goal_tolerance")
        self._neighbor_radius = params.get_float("neighbor_radius")
        self._r = params.get_float("control_weight")
        self._vmax = params.get_float("max_velocity")
        self._seed = params.get_int("seed")

    @property
    def name(self) -> str:
        return "kinodynamic_rrt_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.SAMPLING_SPACE}

    def _sample(
        self, space: SamplingSpace[Point], goal: State4, rng: np.random.Generator
    ) -> State4:
        # Goal-biasing draws the goal rest-state directly (LaValle 1998); otherwise a
        # free position with a random velocity in [-v_max, v_max]² (a full 4D sample).
        if rng.random() < self._goal_bias:
            return goal
        px, py = space.sample()
        vx = float(rng.uniform(-self._vmax, self._vmax))
        vy = float(rng.uniform(-self._vmax, self._vmax))
        return (px, py, vx, vy)

    def _connect(
        self, space: SamplingSpace[Point], x0: State4, x1: State4
    ) -> tuple[float, list[Point]] | None:
        """Optimal edge x0→x1: (edge_cost, dense trajectory) if collision-free, else None."""
        cost, tau = optimal_cost(x0, x1, self._r)
        if not math.isfinite(cost) or tau <= _EPS:
            return None
        traj = _trajectory_xy(x0, x1, tau)
        prev = (x0[0], x0[1])
        for pt in traj:
            if not space.is_motion_valid(prev, pt):
                return None
            prev = pt
        return cost, traj

    def _nearest(self, tree: _KinoTree, positions: np.ndarray, target: State4) -> int:
        # Euclidean prefilter to the K closest, then pick the exact optimal-cost minimiser.
        dists = np.hypot(positions[:, 0] - target[0], positions[:, 1] - target[1])
        k = min(_NEAREST_CANDIDATES, len(tree))
        candidates = np.argpartition(dists, k - 1)[:k] if k < len(tree) else range(len(tree))
        best_idx = -1
        best_cost = math.inf
        for i in candidates:
            i = int(i)
            cost, _ = optimal_cost(tree.states[i], target, self._r)
            if cost < best_cost:
                best_cost = cost
                best_idx = i
        return best_idx

    def _neighborhood(self, tree: _KinoTree, positions: np.ndarray, target: State4) -> list[int]:
        dists = np.hypot(positions[:, 0] - target[0], positions[:, 1] - target[1])
        within = np.nonzero(dists <= self._neighbor_radius)[0]
        if len(within) > _MAX_NEIGHBORS:
            order = np.argsort(dists[within])[:_MAX_NEIGHBORS]
            within = within[order]
        return [int(i) for i in within]

    def _choose_parent(
        self,
        space: SamplingSpace[Point],
        tree: _KinoTree,
        x_new: State4,
        near_idx: int,
        neighborhood: Iterable[int],
        recorder: TraceRecorder | None,
    ) -> int:
        """Attach x_new to the min-cost feasible parent (choose-parent, Webb & van
        den Berg 2013 kinodynamic form of Karaman & Frazzoli 2011)."""
        best_parent = -1
        best_edge: list[Point] = []
        best_total = math.inf
        for j in {near_idx, *neighborhood}:
            if j < 0:
                continue
            conn = self._connect(space, tree.states[j], x_new)
            if conn is None:
                continue
            edge_cost, traj = conn
            total = tree.cost[j] + edge_cost
            if recorder is not None:
                recorder.candidate_evaluated((x_new[0], x_new[1]), total)
            if total < best_total:
                best_total = total
                best_parent = j
                best_edge = traj
        if best_parent < 0:
            return -1
        edge_cost = best_total - tree.cost[best_parent]
        new_idx = tree.add(x_new, best_parent, edge_cost, best_edge)
        if recorder is not None:
            # Emit the curved edge as a chain of chords so the trajectory renders (the
            # SamplingSpace viz draws straight edge_added segments).
            prev = (tree.states[best_parent][0], tree.states[best_parent][1])
            for pt in best_edge:
                recorder.edge_added(pt, prev)
                prev = pt
        return new_idx

    def _rewire(
        self,
        space: SamplingSpace[Point],
        tree: _KinoTree,
        new_idx: int,
        neighborhood: Iterable[int],
        recorder: TraceRecorder | None,
    ) -> None:
        x_new = tree.states[new_idx]
        for j in neighborhood:
            if j == tree.parent[new_idx] or j == new_idx:
                continue
            conn = self._connect(space, x_new, tree.states[j])
            if conn is None:
                continue
            edge_cost, traj = conn
            if tree.cost[new_idx] + edge_cost < tree.cost[j]:
                tree.reparent(j, new_idx, edge_cost, traj)
                if recorder is not None:
                    recorder.rewire((tree.states[j][0], tree.states[j][1]),
                                    (x_new[0], x_new[1]))

    def _goal_arrival(
        self,
        space: SamplingSpace[Point],
        tree: _KinoTree,
        new_idx: int,
        x_new: State4,
        x_goal: State4,
    ) -> tuple[float, list[Point]] | None:
        """Cost + trajectory of reaching the goal rest-state from ``x_new``.

        A goal-biased sample lands x_new *on* the goal, whose self-connection is
        degenerate (τ*=0): the node itself is the arrival, so the trajectory is empty
        and the cost is its cumulative tree cost. Otherwise fly the optimal edge to
        the goal rest-state and require it collision-free."""
        if (
            abs(x_new[0] - x_goal[0]) <= _EPS
            and abs(x_new[1] - x_goal[1]) <= _EPS
            and abs(x_new[2]) <= _EPS
            and abs(x_new[3]) <= _EPS
        ):
            return tree.cost[new_idx], []
        conn = self._connect(space, x_new, x_goal)
        if conn is None:
            return None
        edge_cost, traj = conn
        return tree.cost[new_idx] + edge_cost, traj

    def plan(
        self,
        space: SamplingSpace[Point],
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        rng = np.random.default_rng(self._seed)
        # Fixed final state: start and goal are lifted to rest (zero velocity).
        x_start: State4 = (start[0], start[1], 0.0, 0.0)
        x_goal: State4 = (goal[0], goal[1], 0.0, 0.0)
        tree = _KinoTree(x_start)
        # Goal is not a growth/rewire node (Karaman & Frazzoli 2011); track its best
        # parent + incoming trajectory only.
        best_goal_parent = -1
        best_goal_cost = math.inf
        best_goal_traj: list[Point] = []
        iterations = 0

        for _ in range(self._max_iter):
            iterations += 1
            q_rand = self._sample(space, x_goal, rng)
            if recorder is not None:
                recorder.sample_drawn((q_rand[0], q_rand[1]))
            positions = tree.positions()
            near_idx = self._nearest(tree, positions, q_rand)
            # Optimal steering reaches the sampled state exactly (no truncation needed
            # for a controllable linear system); x_new is the sample itself.
            x_new = q_rand
            neighborhood = self._neighborhood(tree, positions, x_new)
            new_idx = self._choose_parent(space, tree, x_new, near_idx, neighborhood, recorder)
            if new_idx < 0:
                continue
            self._rewire(space, tree, new_idx, neighborhood, recorder)

            if math.hypot(x_new[0] - goal[0], x_new[1] - goal[1]) <= self._goal_tol:
                arrival = self._goal_arrival(space, tree, new_idx, x_new, x_goal)
                if arrival is not None:
                    cand, traj = arrival
                    if cand < best_goal_cost:
                        best_goal_cost = cand
                        best_goal_parent = new_idx
                        best_goal_traj = traj
                        if recorder is not None:
                            recorder.path_found(tree.path_xy_to(new_idx) + traj)

        runtime = time.monotonic() - t0
        success = best_goal_parent >= 0
        if success:
            path = tree.path_xy_to(best_goal_parent) + best_goal_traj
            cost = best_goal_cost
        else:
            path = []
            cost = 0.0
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": cost,
                    "expanded_nodes": float(len(tree) - 1),
                    "samples": float(iterations),
                    "tree_size": float(len(tree)),
                    "iterations": float(iterations),
                },
            )
        stats = PlanStats(
            expanded_nodes=len(tree) - 1,
            samples=iterations,
            iterations=iterations,
            tree_size=len(tree),
        )
        return PlanResult(success, path, cost, stats)
