"""LQR-RRT* — RRT* whose extension heuristics (distance metric + steering) are
derived automatically from a Linear-Quadratic Regulator.

Perez, Platt, Konidaris, Kaelbling & Lozano-Pérez (2012), "LQR-RRT*: Optimal
Sampling-Based Motion Planning with Automatically Derived Extension Heuristics".
Geometric RRT* needs a metric and a steering primitive supplied by hand; for a
system with dynamics those are hard to design. LQR-RRT* instead linearises the
dynamics and picks a quadratic cost J=∫(xᵀQx+uᵀRu)dt, then lets the LQR solution
*derive* both:

  * the nearest-neighbour / near-set metric is the LQR cost-to-go quadratic form
    dist(a,b) = (a−b)ᵀ S (a−b), where S solves the Riccati equation, and
  * steering is the LQR feedback policy u = −K(x − ref), K = (R+BᵀPB)⁻¹BᵀPA,
    integrated forward — no bespoke steer() function.

This sits between geometric RRT* (2011, straight edges / Euclidean metric) and
kinodynamic RRT* (2013, exact fixed-final-state OBVP): the LQR gives a *feedback*
extension heuristic that is cheap and general, at the price of being asymptotic
rather than an exact two-point solve.

Dynamics. Same 2D double integrator as this repo's kinodynamic RRT*, so the two
compare on one benchmark: state (x, y, vx, vy), control = acceleration, decoupled
per axis with ẋ=Ax+Bu, A=[[0,1],[0,0]], B=[[0],[1]]. The planner OWNS its dynamics
and depends only on the ``SamplingSpace`` capability — collisions are checked on
the (x, y) projection via ``is_motion_valid`` (the map knows nothing of velocity).

Riccati. Because the system is linear/time-invariant and Q is block-diagonal, S
and K are state-independent and shared by both axes; we solve the 2×2 discrete
Riccati recursion once (fixed-point of the finite-horizon DARE) at construction:
    P ← Q + AᵀPA − AᵀPB (R+BᵀPB)⁻¹ BᵀPA,   K = (R+BᵀPB)⁻¹ BᵀPA.
Distinct from kinodynamic RRT*, which has no Riccati solve — it roots a quartic
for an exact optimal-arrival time. Here the metric/steer come from S/K.

Nodes are lifted to REST states (velocity 0): the LQR feedback regulates exactly
to a rest equilibrium (the closed loop A−BK is Hurwitz, no steady-state offset),
so every stored edge is a real, collision-free, dynamically-feasible trajectory
that reaches its child — which keeps the RRT* rewiring machinery exact. Samples
still carry a random velocity so the nearest-neighbour LQR metric ranks nodes by
full-state cost-to-go (direction-aware, unlike Euclidean); the extension steers to
a rest waypoint at most ``step_size`` away toward the sample.
"""

from __future__ import annotations

import math
import time

import numpy as np

from navigation.core.capabilities import Capability, SamplingSE2Space
from navigation.core.params import ParamSet
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Footprint, PlanResult, PlanStats, Point

# Double-integrator planning state: world position + velocity (x, y, vx, vy). Only
# the (x, y) projection is ever handed to the SamplingSpace map (no velocity).
State4 = tuple[float, float, float, float]

_EPS = 1e-9
# Euclidean position prefilter to the K closest before the exact LQR-metric compare
# in nearest-neighbour (k-nearest RRT* variant, Karaman & Frazzoli 2011).
_NEAREST_CANDIDATES = 16
# Cap the choose-parent / rewire neighbourhood so per-iteration work stays bounded.
_MAX_NEIGHBORS = 16
# LQR steering horizon: a rest→rest regulator converges asymptotically, so cap the
# number of integration steps; blocked/non-converged rolls are rejected.
_STEER_MAX_STEPS = 200
# A rest waypoint is "reached" when position and velocity errors fall under these.
_REACH_POS_TOL = 0.05
_REACH_VEL_TOL = 0.05
# DARE fixed-point iteration budget + convergence tolerance (2×2 problem).
_DARE_MAX_ITERS = 1000
_DARE_TOL = 1e-12


def solve_dlqr(
    q_pos: float, q_vel: float, r_ctrl: float, dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """Per-axis discrete LQR of the double integrator via the Riccati (DARE)
    recursion. Returns (S, K): S the 2×2 steady-state cost-to-go matrix and K the
    1×2 optimal feedback gain (Perez et al. 2012). Both axes share these because
    the LTI dynamics and diagonal Q decouple identically per axis.

    Discretisation over dt: A=[[1,dt],[0,1]], B=[[dt²/2],[dt]], Q=diag(q_pos,q_vel),
    R=[r_ctrl]. Iterating P ← Q + AᵀPA − AᵀPB(R+BᵀPB)⁻¹BᵀPA from P₀=Q converges to
    the unique stabilising solution (double integrator is controllable, Q≻0)."""
    a = np.array([[1.0, dt], [0.0, 1.0]])
    b = np.array([[0.5 * dt * dt], [dt]])
    q = np.diag([q_pos, q_vel])
    r = np.array([[r_ctrl]])
    p = q.copy()
    for _ in range(_DARE_MAX_ITERS):
        bt_p = b.T @ p
        gain = np.linalg.solve(r + bt_p @ b, bt_p @ a)  # 1×2
        p_next = q + a.T @ p @ a - (a.T @ p @ b) @ gain
        if float(np.max(np.abs(p_next - p))) < _DARE_TOL:
            p = p_next
            break
        p = p_next
    bt_p = b.T @ p
    gain = np.linalg.solve(r + bt_p @ b, bt_p @ a)
    return p, gain[0]


def lqr_cost_to_go(a: State4, b: State4, s: np.ndarray) -> float:
    """LQR distance metric dist(a,b) = (a−b)ᵀ S (a−b) summed over the two decoupled
    axes, S the per-axis cost-to-go (Perez et al. 2012). Zero iff a==b, else > 0."""
    dpx, dpy = a[0] - b[0], a[1] - b[1]
    dvx, dvy = a[2] - b[2], a[3] - b[3]
    s00, s01, s11 = float(s[0, 0]), float(s[0, 1]), float(s[1, 1])
    return (
        s00 * (dpx * dpx + dpy * dpy)
        + 2.0 * s01 * (dpx * dvx + dpy * dvy)
        + s11 * (dvx * dvx + dvy * dvy)
    )


class _LqrTree:
    """Search tree over double-integrator rest states (parallel arrays), mirroring
    the RRT* tree but keyed on LQR cost. Each node stores its incoming edge cost +
    the dense (x, y) trajectory of the LQR roll so a rewire propagates cumulative
    cost through a subtree without re-steering, and path reconstruction emits the
    true curved trajectory. (The State4-with-trajectory tree is not in the shared
    _sampling helpers, which are 2D-Point only; importing kinodynamic's copy would
    couple two algorithm modules, which the architecture forbids.)"""

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
        # Push the cost delta down the subtree (edge costs unchanged by a rewire of
        # an ancestor; only cumulative sums shift) so descendants keep exact costs.
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


class LQRRRTStar(GlobalPlanner[Point, "SamplingSE2Space[Point]"]):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._max_iter = params.get_int("max_iterations")
        self._step_size = params.get_float("step_size")
        self._goal_bias = params.get_float("goal_bias")
        self._goal_tol = params.get_float("goal_tolerance")
        self._neighbor_radius = params.get_float("neighbor_radius")
        self._q_pos = params.get_float("q_pos")
        self._q_vel = params.get_float("q_vel")
        self._r_ctrl = params.get_float("r_ctrl")
        self._dt = params.get_float("lqr_dt")
        self._u_max = params.get_float("control_limit")
        self._vmax = params.get_float("max_velocity")
        self._seed = params.get_int("seed")
        # 차체는 inscribed disc — 점이 아니라 몸체가 벽을 비켜 가야 한다.
        self._footprint = Footprint(params.get_float("footprint_radius"))
        # Riccati solve once: S/K are state-independent for this LTI system.
        self._s, self._k = solve_dlqr(self._q_pos, self._q_vel, self._r_ctrl, self._dt)

    @property
    def name(self) -> str:
        return "lqr_rrt_star"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.SAMPLING_SPACE, Capability.SE2_COLLISION_SPACE}

    def _sample(
        self, space: "SamplingSE2Space[Point]", goal: State4, rng: np.random.Generator
    ) -> State4:
        # Goal-biasing draws the goal rest-state directly (LaValle 1998); otherwise a
        # free position with a random velocity so the LQR nearest metric is full-state.
        if rng.random() < self._goal_bias:
            return goal
        px, py = space.sample()
        vx = float(rng.uniform(-self._vmax, self._vmax))
        vy = float(rng.uniform(-self._vmax, self._vmax))
        return (px, py, vx, vy)

    def _rest_target(self, x_from: State4, sample: State4) -> State4:
        """Rest waypoint at most ``step_size`` from ``x_from`` toward the sample's
        position (RRT* eta cap). Velocity 0 so the LQR feedback regulates to it
        exactly (a rest equilibrium of the double integrator)."""
        dx, dy = sample[0] - x_from[0], sample[1] - x_from[1]
        dist = math.hypot(dx, dy)
        if dist <= self._step_size or dist < _EPS:
            return (sample[0], sample[1], 0.0, 0.0)
        scale = self._step_size / dist
        return (x_from[0] + dx * scale, x_from[1] + dy * scale, 0.0, 0.0)

    def _roll(
        self, space: "SamplingSE2Space[Point]", x_from: State4, target: State4
    ) -> tuple[float, list[Point]] | None:
        """Integrate the LQR feedback u=−K(x−target), clamped, from ``x_from`` until
        it reaches ``target`` (rest) or the horizon. Returns (edge_cost, dense (x, y)
        trajectory) if it reaches target collision-free, else None.

        edge_cost is the realised LQR cost Σ(xᵀQx+uᵀRu)·dt of the roll — the true
        cost of this edge, used for choose-parent / rewire (Perez et al. 2012)."""
        k0, k1 = float(self._k[0]), float(self._k[1])
        dt = self._dt
        half_dt2 = 0.5 * dt * dt
        px, py, vx, vy = x_from
        tx, ty, tvx, tvy = target
        cost = 0.0
        traj: list[Point] = []
        prev: Point = (px, py)
        for _ in range(_STEER_MAX_STEPS):
            ex_p, ex_v = px - tx, vx - tvx
            ey_p, ey_v = py - ty, vy - tvy
            ux = -(k0 * ex_p + k1 * ex_v)
            uy = -(k0 * ey_p + k1 * ey_v)
            ux = max(-self._u_max, min(self._u_max, ux))
            uy = max(-self._u_max, min(self._u_max, uy))
            # Stage cost xᵀQx + uᵀRu integrated over the step (both axes).
            cost += (
                self._q_pos * (ex_p * ex_p + ey_p * ey_p)
                + self._q_vel * (ex_v * ex_v + ey_v * ey_v)
                + self._r_ctrl * (ux * ux + uy * uy)
            ) * dt
            px = px + dt * vx + half_dt2 * ux
            py = py + dt * vy + half_dt2 * uy
            vx = vx + dt * ux
            vy = vy + dt * uy
            cur: Point = (px, py)
            # disc 는 방향 불변이라 theta 는 형식상 0. 적분 스텝(≤ v_max·dt ≈ 0.3 m)과
            # 반경이 같은 자릿수라 disc 사슬이 몸체 여유를 근사하고, 점 수준
            # corner-cut 은 supercover chord 검사가 마저 막는다.
            if space.is_collision(self._footprint, (px, py, 0.0)):
                return None
            if not space.is_motion_valid(prev, cur):
                return None
            traj.append(cur)
            prev = cur
            if (
                abs(px - tx) <= _REACH_POS_TOL
                and abs(py - ty) <= _REACH_POS_TOL
                and abs(vx - tvx) <= _REACH_VEL_TOL
                and abs(vy - tvy) <= _REACH_VEL_TOL
            ):
                # Snap the final waypoint onto the target so node joins are exact.
                traj[-1] = (tx, ty)
                return cost, traj
        return None

    def _nearest(self, tree: _LqrTree, positions: np.ndarray, target: State4) -> int:
        # Euclidean prefilter to the K closest, then the exact LQR cost-to-go minimiser.
        dists = np.hypot(positions[:, 0] - target[0], positions[:, 1] - target[1])
        k = min(_NEAREST_CANDIDATES, len(tree))
        candidates = np.argpartition(dists, k - 1)[:k] if k < len(tree) else range(len(tree))
        best_idx = -1
        best_cost = math.inf
        for i in candidates:
            i = int(i)
            c = lqr_cost_to_go(tree.states[i], target, self._s)
            if c < best_cost:
                best_cost = c
                best_idx = i
        return best_idx

    def _neighborhood(self, tree: _LqrTree, positions: np.ndarray, target: State4) -> list[int]:
        dists = np.hypot(positions[:, 0] - target[0], positions[:, 1] - target[1])
        within = np.nonzero(dists <= self._neighbor_radius)[0]
        if len(within) > _MAX_NEIGHBORS:
            order = np.argsort(dists[within])[:_MAX_NEIGHBORS]
            within = within[order]
        return [int(i) for i in within]

    def _choose_parent(
        self,
        space: "SamplingSE2Space[Point]",
        tree: _LqrTree,
        x_new: State4,
        near_idx: int,
        near_edge: tuple[float, list[Point]],
        neighborhood: list[int],
        recorder: TraceRecorder | None,
    ) -> int:
        """Attach x_new to the min-cost feasible parent (choose-parent, Karaman &
        Frazzoli 2011). ``near_idx`` with its already-rolled ``near_edge`` is the
        default; neighbours can only improve if their LQR roll also reaches x_new."""
        best_parent = near_idx
        best_edge = near_edge[1]
        best_total = tree.cost[near_idx] + near_edge[0]
        for j in neighborhood:
            if j == near_idx:
                continue
            roll = self._roll(space, tree.states[j], x_new)
            if roll is None:
                continue
            edge_cost, traj = roll
            total = tree.cost[j] + edge_cost
            if recorder is not None:
                recorder.candidate_evaluated((x_new[0], x_new[1]), total)
            if total < best_total:
                best_total = total
                best_parent = j
                best_edge = traj
        edge_cost = best_total - tree.cost[best_parent]
        new_idx = tree.add(x_new, best_parent, edge_cost, best_edge)
        if recorder is not None:
            # Emit the curved edge as a chain of chords so the trajectory renders.
            prev = (tree.states[best_parent][0], tree.states[best_parent][1])
            for pt in best_edge:
                recorder.edge_added(pt, prev)
                prev = pt
        return new_idx

    def _rewire(
        self,
        space: "SamplingSE2Space[Point]",
        tree: _LqrTree,
        new_idx: int,
        neighborhood: list[int],
        recorder: TraceRecorder | None,
    ) -> None:
        x_new = tree.states[new_idx]
        for j in neighborhood:
            if j == tree.parent[new_idx] or j == new_idx:
                continue
            roll = self._roll(space, x_new, tree.states[j])
            if roll is None:
                continue
            edge_cost, traj = roll
            if tree.cost[new_idx] + edge_cost < tree.cost[j]:
                tree.reparent(j, new_idx, edge_cost, traj)
                if recorder is not None:
                    recorder.rewire((tree.states[j][0], tree.states[j][1]),
                                    (x_new[0], x_new[1]))

    def plan(
        self,
        space: "SamplingSE2Space[Point]",
        start: Point,
        goal: Point,
        recorder: TraceRecorder | None = None,
    ) -> PlanResult[Point]:
        t0 = time.monotonic()
        rng = np.random.default_rng(self._seed)
        # Start and goal are lifted to rest (zero velocity) — LQR equilibria.
        x_start: State4 = (start[0], start[1], 0.0, 0.0)
        x_goal: State4 = (goal[0], goal[1], 0.0, 0.0)
        tree = _LqrTree(x_start)
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
            # Extend: steer x_near to a rest waypoint at most step_size toward q_rand.
            x_new = self._rest_target(tree.states[near_idx], q_rand)
            near_roll = self._roll(space, tree.states[near_idx], x_new)
            if near_roll is None:
                continue
            neighborhood = self._neighborhood(tree, positions, x_new)
            new_idx = self._choose_parent(
                space, tree, x_new, near_idx, near_roll, neighborhood, recorder
            )
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

    def _goal_arrival(
        self,
        space: "SamplingSE2Space[Point]",
        tree: _LqrTree,
        new_idx: int,
        x_new: State4,
        x_goal: State4,
    ) -> tuple[float, list[Point]] | None:
        """Cost + trajectory of the LQR roll from ``x_new`` (rest) to the goal rest
        state. A goal-biased sample lands x_new on the goal (already at rest), whose
        self-roll is degenerate — the node itself is the arrival (empty trajectory)."""
        if (
            abs(x_new[0] - x_goal[0]) <= _EPS
            and abs(x_new[1] - x_goal[1]) <= _EPS
            and abs(x_new[2]) <= _EPS
            and abs(x_new[3]) <= _EPS
        ):
            return tree.cost[new_idx], []
        roll = self._roll(space, x_new, x_goal)
        if roll is None:
            return None
        edge_cost, traj = roll
        return tree.cost[new_idx] + edge_cost, traj
