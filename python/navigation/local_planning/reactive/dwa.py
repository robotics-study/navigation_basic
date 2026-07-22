"""Dynamic Window Approach (Fox, Burgard & Thrun 1997, DOI 10.1109/100.580977):
searches the reachable (v, omega) command space directly instead of a
Cartesian path. Every tick samples a deterministic grid inside the box formed
by the robot's kinematic limits intersected with what one accel-limited tick
can reach from the current velocity, rolls each candidate forward as a
constant-command arc, discards ones that collide or cannot stop before the
nearest obstacle, and picks the highest-scoring survivor.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Footprint, LocalTask, Point, Pose, RobotState, VelocityCommand

from .._geometry import wrap_to_pi

# Below this |omega| the closed-form arc (division by omega) is numerically
# unstable, so the rollout falls back to the straight-line limit instead --
# the same threshold and reasoning as simulation.py's _OMEGA_EPS. Duplicated
# rather than imported: algorithm modules depend on core only, never on the
# simulator (see local_planning/simulation.py's own docstring).
_OMEGA_EPS = 1e-9

class Dwa(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._max_speed = params.get_float("max_speed")
        self._min_speed = params.get_float("min_speed")
        self._max_omega = params.get_float("max_omega")
        self._accel = params.get_float("accel")
        self._accel_omega = params.get_float("accel_omega")
        self._v_samples = params.get_int("v_samples")
        self._omega_samples = params.get_int("omega_samples")
        self._sim_time = params.get_float("sim_time")
        self._sim_steps = params.get_int("sim_steps")
        self._heading_weight = params.get_float("heading_weight")
        self._clearance_weight = params.get_float("clearance_weight")
        self._velocity_weight = params.get_float("velocity_weight")
        self._clearance_limit = params.get_float("clearance_limit")
        self._slow_radius = params.get_float("slow_radius")
        self._footprint_radius = params.get_float("footprint_radius")

    @property
    def name(self) -> str:
        return "dwa"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def _rollout(self, pose: Pose, v: float, omega: float) -> list[Pose]:
        # Each candidate is scored by holding (v, omega) constant for sim_time,
        # sampled at sim_steps equally spaced instants (Fox 1997's circular-arc
        # trajectory prediction) -- every point is computed directly from the
        # start pose (not chained step-to-step), which is exact for a constant
        # command and matches simulation.py's per-tick integrate_unicycle.
        x, y, theta = pose
        poses: list[Pose] = []
        for k in range(1, self._sim_steps + 1):
            t = self._sim_time * k / self._sim_steps
            if abs(omega) < _OMEGA_EPS:
                poses.append((x + v * t * math.cos(theta), y + v * t * math.sin(theta), theta))
                continue
            new_theta = theta + omega * t
            px = x + (v / omega) * (math.sin(new_theta) - math.sin(theta))
            py = y - (v / omega) * (math.cos(new_theta) - math.cos(theta))
            poses.append((px, py, wrap_to_pi(new_theta)))
        return poses

    def _score(
        self,
        space: ObstacleQuery,
        footprint: Footprint,
        rollout: list[Pose],
        v: float,
        omega: float,
        goal_x: float,
        goal_y: float,
    ) -> tuple[float, float, float, float, bool]:
        """Returns (cost, heading, clearance, velocity, admissible). A colliding
        rollout is rejected outright (cost=0, the three terms zeroed) rather
        than scored, per Fox 1997's hard obstacle constraint."""
        if any(space.is_collision(footprint, pose) for pose in rollout):
            return 0.0, 0.0, 0.0, 0.0, False

        # Fox 1997 eq. 14 uses the curvature's true distance to the nearest
        # obstacle; this implementation approximates it conservatively with the
        # minimum clearance sampled along the finite rollout.
        nearest = min(space.distance_to_nearest((px, py)) for px, py, _ in rollout)
        clearance = max(0.0, nearest - self._footprint_radius)
        x_end, y_end, theta_end = rollout[-1]
        # Fox 1997 eq. 15's target-heading term, evaluated at this
        # implementation's rollout endpoint rather than the paper's
        # maximum-deceleration stopping position (a deliberate simplification).
        goal_bearing = math.atan2(goal_y - y_end, goal_x - x_end)
        heading = 1.0 - abs(wrap_to_pi(goal_bearing - theta_end)) / math.pi
        velocity = v / self._max_speed
        clearance_term = min(clearance, self._clearance_limit) / self._clearance_limit
        # Fixed (non-batch) normalization: scoring one candidate never depends
        # on the rest of the candidate set, which keeps py/cpp/TS scoring
        # bit-identical and reproducible test-to-test, unlike Fox 1997's
        # per-tick min-max smoothing over the whole candidate batch.
        cost = (
            self._heading_weight * heading
            + self._clearance_weight * clearance_term
            + self._velocity_weight * velocity
        )
        # Fox 1997 eq. 14: admissible iff the robot could stop (at accel/
        # accel_omega) before covering `clearance` -- a stopping-distance bound,
        # approximated here with the finite-rollout clearance above.
        admissible = v <= math.sqrt(2.0 * clearance * self._accel) and abs(omega) <= math.sqrt(
            2.0 * clearance * self._accel_omega
        )
        return cost, heading, clearance, velocity, admissible

    def _decelerate(self, v_a: float, omega_a: float, dt: float) -> tuple[float, float]:
        # No admissible candidate survived this tick (e.g. boxed in on every
        # side): brake at the kinematic limits rather than execute an
        # unscored guess. Persistent triggering here is a local minimum, which
        # the simulator's stall detector reports honestly as STALLED.
        v_cmd = max(0.0, v_a - self._accel * dt)
        sign = 1.0 if omega_a > 0.0 else (-1.0 if omega_a < 0.0 else 0.0)
        omega_cmd = omega_a - sign * min(abs(omega_a), self._accel_omega * dt)
        return v_cmd, omega_cmd

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        x, y, theta = state.pose
        v_a, omega_a = state.v, state.omega
        goal_x, goal_y = task.goal[0], task.goal[1]

        remaining = math.hypot(goal_x - x, goal_y - y)
        # Goal-proximity speed cap: not part of Fox 1997, a practical extension
        # so an episode ends in REACHED instead of orbiting/overshooting the
        # goal at max_speed.
        v_max_eff = self._max_speed * min(1.0, remaining / self._slow_radius)

        v_lo = max(self._min_speed, v_a - self._accel * dt)
        v_hi = min(v_max_eff, v_a + self._accel * dt)
        omega_lo = max(-self._max_omega, omega_a - self._accel_omega * dt)
        omega_hi = min(self._max_omega, omega_a + self._accel_omega * dt)

        best_index: int | None = None
        best_cost = 0.0
        best_v, best_omega = 0.0, 0.0
        # Trace buffering: only populated when a recorder is attached (zero
        # cost otherwise) because `selected` cannot be known until every
        # candidate in the tick has been scored.
        buffered: list[tuple[Point, float, dict[str, float], list[Point]]] = []

        if v_lo <= v_hi and omega_lo <= omega_hi:
            footprint = Footprint(self._footprint_radius)
            v_step = (v_hi - v_lo) / (self._v_samples - 1) if self._v_samples > 1 else 0.0
            n_omega = self._omega_samples
            omega_step = (omega_hi - omega_lo) / (n_omega - 1) if n_omega > 1 else 0.0
            candidate_index = 0
            # Deterministic uniform grid (never random sampling): fixed
            # traversal order (v outer, omega inner) keeps py/cpp/TS scoring
            # and tie-breaking bit-identical.
            for i in range(self._v_samples):
                v = v_lo + v_step * i
                for j in range(self._omega_samples):
                    omega = omega_lo + omega_step * j
                    rollout = self._rollout((x, y, theta), v, omega)
                    cost, heading, clearance, velocity, admissible = self._score(
                        space, footprint, rollout, v, omega, goal_x, goal_y
                    )
                    if admissible and (best_index is None or cost > best_cost):
                        best_index, best_cost, best_v, best_omega = candidate_index, cost, v, omega
                    if recorder is not None:
                        x_end, y_end, _ = rollout[-1]
                        data = {
                            "v": v,
                            "omega": omega,
                            "heading": heading,
                            "clearance": clearance,
                            "velocity": velocity,
                            "admissible": 1.0 if admissible else 0.0,
                            "selected": 0.0,  # finalized below once selection is known
                        }
                        rollout_xy = [(p[0], p[1]) for p in rollout]
                        buffered.append(((x_end, y_end), cost, data, rollout_xy))
                    candidate_index += 1

        if best_index is not None:
            v_cmd, omega_cmd = best_v, best_omega
        else:
            v_cmd, omega_cmd = self._decelerate(v_a, omega_a, dt)

        if recorder is not None:
            for idx, (state_xy, cost, data, rollout_xy) in enumerate(buffered):
                data["selected"] = 1.0 if idx == best_index else 0.0
                recorder.candidate_evaluated(state_xy, cost, data=data, rollout=rollout_xy)

        return VelocityCommand(v_cmd, omega_cmd)
