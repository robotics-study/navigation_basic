"""Model Predictive Control for a kinematic mobile robot (Klančar & Škrjanc,
Robotics and Autonomous Systems 55(6):460-469, 2007, DOI
10.1016/j.robot.2007.01.002): each tick predicts the next H steps with the
unicycle model, optimizes the control sequence U against the shared receding-
horizon cost J(U), executes only the first control u_0, and re-optimizes next
tick (receding horizon).

The optimizer is fixed-iteration projected gradient descent with a central
finite-difference gradient. Finite differences (rather than a hand-derived
analytic Jacobian of the arc integrator) are chosen because the gradient then
follows mechanically from the one shared scalar cost -- so as long as J(U) is
identical across Python/C++/TS, the descent is too, and the contrast with MPPI
stays a clean "same J, sampling instead of gradient".
"""

from __future__ import annotations

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Pose, RobotState, VelocityCommand

from ._rollout import clamp, rollout, sequence_cost

# A control sequence U = [u_0, ..., u_{H-1}] with u_k = (v_k, omega_k).
_Controls = list[tuple[float, float]]


class MpcPlanner(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._horizon = params.get_int("horizon")
        self._iterations = params.get_int("iterations")
        self._step_alpha = params.get_float("step_alpha")
        self._grad_eps = params.get_float("grad_eps")
        self._max_step_v = params.get_float("max_step_v")
        self._max_step_omega = params.get_float("max_step_omega")
        self._w_goal = params.get_float("w_goal")
        self._w_obstacle = params.get_float("w_obstacle")
        self._w_control = params.get_float("w_control")
        self._min_obstacle_dist = params.get_float("min_obstacle_dist")
        self._v_max = params.get_float("v_max")
        self._omega_max = params.get_float("omega_max")
        self._a_max = params.get_float("a_max")
        self._footprint_radius = params.get_float("footprint_radius")

        # Control sequence U carried across ticks for warm-starting; empty = cold
        # start (first tick / after reset()), which seeds U with zeros.
        self._controls: _Controls = []

    @property
    def name(self) -> str:
        return "mpc"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def reset(self) -> None:
        self._controls = []

    def _warm_start(self) -> _Controls:
        """Left-shift the previous U and duplicate its last control, so this
        tick starts from where the last horizon left off (the executed u_0 is
        dropped). Cold start seeds H zero controls."""
        if not self._controls:
            return [(0.0, 0.0) for _ in range(self._horizon)]
        return self._controls[1:] + [self._controls[-1]]

    def _cost(
        self, space: ObstacleQuery, s0: Pose, controls: _Controls, goal: Pose, h: float
    ) -> float:
        traj = rollout(s0, controls, h)
        return sequence_cost(
            space,
            traj,
            controls,
            goal,
            self._footprint_radius,
            self._w_goal,
            self._w_obstacle,
            self._min_obstacle_dist,
            self._w_control,
        )

    def _descent_step(
        self, space: ObstacleQuery, s0: Pose, controls: _Controls, goal: Pose, h: float
    ) -> None:
        """One projected gradient-descent iteration: a full 2H-component central
        finite-difference gradient of J at the current U (component order k
        ascending, v before omega -- part of the cross-language determinism
        contract), then one U <- U - step_alpha*grad update with per-component
        step clamp and box projection (v in [0, v_max], |omega| <= omega_max)."""
        n = len(controls)
        eps = self._grad_eps
        grad_v = [0.0] * n
        grad_omega = [0.0] * n
        for k in range(n):
            v, omega = controls[k]
            controls[k] = (v + eps, omega)
            j_plus = self._cost(space, s0, controls, goal, h)
            controls[k] = (v - eps, omega)
            j_minus = self._cost(space, s0, controls, goal, h)
            grad_v[k] = (j_plus - j_minus) / (2.0 * eps)
            controls[k] = (v, omega + eps)
            j_plus = self._cost(space, s0, controls, goal, h)
            controls[k] = (v, omega - eps)
            j_minus = self._cost(space, s0, controls, goal, h)
            grad_omega[k] = (j_plus - j_minus) / (2.0 * eps)
            controls[k] = (v, omega)

        for k in range(n):
            v, omega = controls[k]
            v -= clamp(self._step_alpha * grad_v[k], -self._max_step_v, self._max_step_v)
            omega -= clamp(
                self._step_alpha * grad_omega[k], -self._max_step_omega, self._max_step_omega
            )
            controls[k] = (
                clamp(v, 0.0, self._v_max),
                clamp(omega, -self._omega_max, self._omega_max),
            )

    def _emit_band(
        self,
        recorder: TraceRecorder,
        s0: Pose,
        traj: list[Pose],
        h: float,
        total_cost: float,
    ) -> None:
        band = [[s0[0], s0[1], s0[2], 0.0]]
        for p in traj:
            band.append([p[0], p[1], p[2], h])
        recorder.band_updated(
            band,
            data={
                "iterations": float(self._iterations),
                "horizon": float(self._horizon),
                "total_cost": total_cost,
            },
        )

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        s0 = state.pose
        goal = task.goal
        # Prediction step equals the control period (dt): predicting and
        # executing on the same discretization keeps the executed u_0 consistent
        # with the horizon it was optimized in.
        h = dt

        controls = self._warm_start()
        for _ in range(self._iterations):
            self._descent_step(space, s0, controls, goal, h)
        self._controls = controls

        if recorder is not None:
            traj = rollout(s0, controls, h)
            total_cost = sequence_cost(
                space,
                traj,
                controls,
                goal,
                self._footprint_radius,
                self._w_goal,
                self._w_obstacle,
                self._min_obstacle_dist,
                self._w_control,
            )
            self._emit_band(recorder, s0, traj, h, total_cost)

        # Executed command: accel-limit the linear speed against the velocity the
        # simulator reports (RobotState.v, like DWA -- no separate _v_prev state),
        # then box-clamp so the executed command is always within limits.
        v0, omega0 = controls[0]
        v0 = clamp(v0, state.v - self._a_max * h, state.v + self._a_max * h)
        v0 = clamp(v0, 0.0, self._v_max)
        omega0 = clamp(omega0, -self._omega_max, self._omega_max)
        return VelocityCommand(v0, omega0)
