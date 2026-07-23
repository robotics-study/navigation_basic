"""Shared receding-horizon rollout and cost for the predictive family.

MPC and MPPI optimize the *same* finite-horizon optimal-control problem: the
decision variable is the control sequence ``U = [u_0, ..., u_{H-1}]`` with
``u_k = (v_k, omega_k)``, and both score it with the *same* cost ``J(U)``. Only
the optimizer differs -- MPC descends a finite-difference gradient, MPPI
averages sampled rollouts. Keeping the closed-form arc integrator and the scalar
cost here as free functions both planners call makes that "same J, different
optimizer" contrast the only thing separating them.

Kept as a subpackage-private module (`_rollout`), mirroring the band family's
`_band.py` precedent for machinery shared across a family's planners rather than
owned by one.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import ObstacleQuery
from navigation.core.types import Pose

from .._geometry import nearest_occupied, wrap_to_pi

# Below this |omega| the closed-form arc (division by omega) is numerically
# unstable, so integration falls back to the straight-line limit instead.
# Re-implemented here rather than imported from simulation.py because algorithm
# modules depend on core only, never on the simulator -- the same reason DWA
# keeps its own _OMEGA_EPS. Sharing the exact closed-form arc (identical to the
# simulator's integrate_unicycle) keeps the planner's internal prediction on the
# same discretization as the executed motion, so observed behavior is the
# algorithm's, not an integrator mismatch.
OMEGA_EPS = 1e-9

# Extra query band beyond the obstacle-penalty activation distance. The hinge
# term is nonzero only where clearance c_k < min_obstacle_dist, i.e. where the
# nearest occupied cell is within min_obstacle_dist + footprint_radius; querying
# one half-cell further lets the finite-difference gradient see an obstacle just
# as it enters the active band instead of only once c_k has already crossed it.
_QUERY_MARGIN = 0.5


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def unicycle_step(s: Pose, u: tuple[float, float], h: float) -> Pose:
    """Exact constant-(v, omega) unicycle arc over one step ``h`` (closed form)
    -- identical to ``simulation.integrate_unicycle``, re-implemented so the
    predictive family never imports the simulator."""
    x, y, theta = s
    v, omega = u
    if abs(omega) < OMEGA_EPS:
        return (x + v * h * math.cos(theta), y + v * h * math.sin(theta), theta)
    new_theta = theta + omega * h
    x2 = x + (v / omega) * (math.sin(new_theta) - math.sin(theta))
    y2 = y - (v / omega) * (math.cos(new_theta) - math.cos(theta))
    return (x2, y2, wrap_to_pi(new_theta))


def rollout(s0: Pose, controls: list[tuple[float, float]], h: float) -> list[Pose]:
    """States ``s_1 .. s_H`` produced by applying each control in ``controls``
    from ``s0`` (length ``H = len(controls)``). ``s0`` is NOT included -- callers
    prepend it themselves when emitting the band."""
    states: list[Pose] = []
    s = s0
    for u in controls:
        s = unicycle_step(s, u, h)
        states.append(s)
    return states


def sequence_cost(
    space: ObstacleQuery,
    traj: list[Pose],
    controls: list[tuple[float, float]],
    goal: Pose,
    footprint_radius: float,
    w_goal: float,
    w_obstacle: float,
    min_obstacle_dist: float,
    w_control: float,
) -> float:
    """Receding-horizon cost ``J(U)`` shared by MPC and MPPI:

    ``J = sum_{k=1..H} [ w_goal*||p_k - g||^2
                       + w_obstacle*max(0, d_min - c_k)^2
                       + w_control*(v_{k-1}^2 + omega_{k-1}^2) ]``

    ``traj[i]`` is the rolled-out state ``s_{i+1}`` and pairs with ``controls[i]
    = u_i``. ``c_k`` is the continuous nearest-occupied distance at ``p_k`` minus
    ``footprint_radius``; the obstacle term uses that continuous distance (not
    the cell-quantized ``distance_to_nearest``, which is constant inside a cell
    and would give a zero finite-difference gradient -- the reason TEB's gradient
    solver uses the same continuous clearance). Explicit scalar accumulation (no
    numpy) keeps the fold order identical across Python/C++/TS.
    """
    gx, gy = goal[0], goal[1]
    r_query = min_obstacle_dist + footprint_radius + _QUERY_MARGIN
    total = 0.0
    for k in range(len(traj)):
        x, y, _ = traj[k]
        dx = x - gx
        dy = y - gy
        total += w_goal * (dx * dx + dy * dy)
        _, d_tilde = nearest_occupied(space, (x, y), r_query)
        if d_tilde != float("inf"):
            c_k = d_tilde - footprint_radius
            hinge = min_obstacle_dist - c_k
            if hinge > 0.0:
                total += w_obstacle * hinge * hinge
        v, omega = controls[k]
        total += w_control * (v * v + omega * omega)
    return total
