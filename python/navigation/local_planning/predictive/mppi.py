"""Model Predictive Path Integral control (Williams, Aldrich & Theodorou, ICRA
2016, DOI 10.1109/ICRA.2016.7487277; Williams et al., IEEE T-RO 34(6):1603-1622,
2018, DOI 10.1109/TRO.2018.2865891): each tick draws K Gaussian-perturbed control
sequences, rolls each out, scores it with the *same* shared receding-horizon cost
J(U) that MPC uses, and updates the nominal control sequence by a softmax
importance-weighted average of the sampled perturbations -- a derivative-free
optimizer where MPC descends a finite-difference gradient. Executes only the
first control u_0 and re-optimizes next tick (receding horizon).

MPPI is a stochastic planner: Python and C++ share the trace event *types* and the
algorithm, not the exact numeric stream. Python draws its Gaussian noise from
numpy's default_rng (PCG64) via Box-Muller so the browser/TS mirror can reproduce
it bit-for-bit off the same uniform stream; C++ draws from std::mt19937 and gives
behavioral parity only, matching the sampling-planner precedent where C++ RRT uses
std::mt19937 while Python uses default_rng. Box-Muller (not numpy's ziggurat
rng.normal) is used because ziggurat is not bit-reproducible across that mirror.
"""

from __future__ import annotations

import math

import numpy as np

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Pose, RobotState, VelocityCommand

from ._rollout import clamp, rollout, sequence_cost

# A control sequence U = [u_0, ..., u_{H-1}] with u_k = (v_k, omega_k).
_Controls = list[tuple[float, float]]


class MppiPlanner(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._horizon = params.get_int("horizon")
        self._num_samples = params.get_int("num_samples")
        self._temperature = params.get_float("temperature")
        self._sigma_v = params.get_float("sigma_v")
        self._sigma_omega = params.get_float("sigma_omega")
        self._w_goal = params.get_float("w_goal")
        self._w_obstacle = params.get_float("w_obstacle")
        self._w_control = params.get_float("w_control")
        self._min_obstacle_dist = params.get_float("min_obstacle_dist")
        self._v_max = params.get_float("v_max")
        self._omega_max = params.get_float("omega_max")
        self._a_max = params.get_float("a_max")
        self._footprint_radius = params.get_float("footprint_radius")
        self._seed = params.get_int("seed")

        # Nominal control sequence Û carried across ticks for warm-starting; empty
        # = cold start (first tick / after reset()), seeded with zeros.
        self._controls: _Controls = []
        self._rng = np.random.default_rng(self._seed)
        # Box-Muller yields two standard normals per uniform pair; the second is
        # cached and returned by the next _gaussian() call. Each control step draws
        # v then omega, so one (v, omega) pair consumes exactly one Box-Muller pair
        # -- and K*H*2 draws per tick is even, so no half-pair leaks across ticks.
        self._spare: float | None = None

    @property
    def name(self) -> str:
        return "mppi"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def reset(self) -> None:
        # Reseed the RNG and drop the nominal sequence so an episode replays
        # identically from a fixed seed (the accel clamp reads RobotState.v, so
        # there is no extra velocity state to reset).
        self._controls = []
        self._rng = np.random.default_rng(self._seed)
        self._spare = None

    def _gaussian(self) -> float:
        """One standard normal via Box-Muller over the PCG64 uniform stream. The
        order of uniform draws is the cross-language reproducibility contract: two
        uniforms per pair, first pair member returned now, second cached. ``1.0 -
        rng.random()`` maps the uniform to (0, 1] so log(0) never occurs."""
        if self._spare is not None:
            z = self._spare
            self._spare = None
            return z
        u1 = 1.0 - float(self._rng.random())
        u2 = float(self._rng.random())
        magnitude = math.sqrt(-2.0 * math.log(u1))
        self._spare = magnitude * math.sin(2.0 * math.pi * u2)
        return magnitude * math.cos(2.0 * math.pi * u2)

    def _warm_start(self) -> _Controls:
        """Left-shift the previous Û and duplicate its last control (the executed
        u_0 is dropped), so this tick starts where the last horizon left off. Cold
        start seeds H zero controls."""
        if not self._controls:
            return [(0.0, 0.0) for _ in range(self._horizon)]
        return self._controls[1:] + [self._controls[-1]]

    def _emit_band(
        self,
        recorder: TraceRecorder,
        s0: Pose,
        traj: list[Pose],
        h: float,
        min_cost: float,
    ) -> None:
        band = [[s0[0], s0[1], s0[2], 0.0]]
        for p in traj:
            band.append([p[0], p[1], p[2], h])
        recorder.band_updated(
            band,
            data={
                "min_cost": min_cost,
                "num_samples": float(self._num_samples),
                "temperature": self._temperature,
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
        # Prediction step equals the control period (dt): predicting and executing
        # on the same discretization keeps the executed u_0 consistent with the
        # horizon it was optimized in.
        h = dt

        controls = self._warm_start()
        horizon = len(controls)

        # Sample K perturbed control sequences and score each. Every RNG draw is
        # unconditional (outside the recorder guard): the samples determine the
        # trajectory, so tracing on/off must not change what the robot does.
        eps_samples: list[_Controls] = []
        costs: list[float] = []
        terminals: list[tuple[float, float]] = []
        rollouts_xy: list[list[tuple[float, float]]] = []
        for _ in range(self._num_samples):
            eps_seq: _Controls = []
            perturbed: _Controls = []
            for j in range(horizon):
                eps_v = self._gaussian() * self._sigma_v
                eps_omega = self._gaussian() * self._sigma_omega
                eps_seq.append((eps_v, eps_omega))
                base_v, base_omega = controls[j]
                perturbed.append(
                    (
                        clamp(base_v + eps_v, 0.0, self._v_max),
                        clamp(base_omega + eps_omega, -self._omega_max, self._omega_max),
                    )
                )
            traj = rollout(s0, perturbed, h)
            cost = sequence_cost(
                space,
                traj,
                perturbed,
                goal,
                self._footprint_radius,
                self._w_goal,
                self._w_obstacle,
                self._min_obstacle_dist,
                self._w_control,
            )
            eps_samples.append(eps_seq)
            costs.append(cost)
            if recorder is not None:
                tx, ty, _ = traj[-1]
                terminals.append((tx, ty))
                rollouts_xy.append([(p[0], p[1]) for p in traj])

        # Softmax importance weights. Subtracting beta = min_k S_k before exp keeps
        # the exponentials in range (the min-cost sample contributes exp(0) = 1, so
        # the normalizer is >= 1 and never underflows); the shift cancels in the
        # normalization (Williams et al. 2018).
        beta = min(costs)
        weights: list[float] = []
        total = 0.0
        for cost in costs:
            w = math.exp(-(cost - beta) / self._temperature)
            weights.append(w)
            total += w
        inv_total = 1.0 / total
        weights = [w * inv_total for w in weights]

        # Update the nominal sequence by the weighted average of the raw noise, then
        # box-project. Accumulation order (j outer, k inner) is a cross-language
        # determinism contract.
        for j in range(horizon):
            base_v, base_omega = controls[j]
            acc_v = 0.0
            acc_omega = 0.0
            for k in range(self._num_samples):
                w = weights[k]
                eps_v, eps_omega = eps_samples[k][j]
                acc_v += w * eps_v
                acc_omega += w * eps_omega
            controls[j] = (
                clamp(base_v + acc_v, 0.0, self._v_max),
                clamp(base_omega + acc_omega, -self._omega_max, self._omega_max),
            )
        self._controls = controls

        if recorder is not None:
            best_index = costs.index(beta)
            for k in range(self._num_samples):
                recorder.candidate_evaluated(
                    terminals[k],
                    costs[k],
                    data={"weight": weights[k], "selected": 1.0 if k == best_index else 0.0},
                    rollout=rollouts_xy[k],
                )
            nominal_traj = rollout(s0, controls, h)
            self._emit_band(recorder, s0, nominal_traj, h, beta)

        # Executed command: accel-limit the linear speed against the velocity the
        # simulator reports (RobotState.v, like DWA/MPC -- no separate _v_prev
        # state), then box-clamp so the executed command is always within limits.
        v0, omega0 = controls[0]
        v0 = clamp(v0, state.v - self._a_max * h, state.v + self._a_max * h)
        v0 = clamp(v0, 0.0, self._v_max)
        omega0 = clamp(omega0, -self._omega_max, self._omega_max)
        return VelocityCommand(v0, omega0)
