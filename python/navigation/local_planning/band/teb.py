"""Timed Elastic Band (Rösmann, Feiten, Wösch, Hoffmann & Bertram, ROBOTIK 2012;
extended in Rösmann et al., Robotics and Autonomous Systems 88:142-153, 2017,
DOI 10.1016/j.robot.2016.11.007): augments an Elastic-Bands-style pose chain
with a per-segment time interval ΔT_i and optimizes both jointly against a
multi-objective soft-constraint cost (reference tracking, obstacle clearance,
velocity/acceleration limits, time-optimality, and a nonholonomic two-pose-arc
constraint) via a fixed number of damped gradient-descent steps -- no external
solver, so every iteration, clamp, and floating-point accumulation order is
spelled out explicitly and mirrored bit-for-bit in the C++ port.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Point, Pose, RobotState, VelocityCommand

from .._geometry import (
    advance_progress_index,
    closest_point_on_segment,
    nearest_occupied,
    sq_dist,
    wrap_to_pi,
)
from ._band import resample_polyline

# Below this squared norm a unit-vector division (or segment-direction atan2) is
# unstable, so the term/direction is skipped instead -- same 1e-12 threshold as
# tracking/_path.py's segment-degeneracy guard and elastic_bands.py's _EPS_SQ,
# applied uniformly to every division-by-length in this file.
_EPS_SQ = 1e-12


def _clamp(value: float, bound: float) -> float:
    return max(-bound, min(bound, value))


def _segment_t(probe: Point, a: Point, b: Point) -> float:
    """Raw (unclamped) projection parameter of ``probe`` onto segment a->b.

    Unlike ``closest_point_on_segment`` (which clamps to [0, 1] and returns a
    point), warm start needs to know whether the robot has advanced *past* the
    band's own first edge -- t >= 1 -- so the parameter itself, not the clamped
    point, is what the caller needs.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < _EPS_SQ:
        return 0.0
    return ((probe[0] - a[0]) * dx + (probe[1] - a[1]) * dy) / seg_len_sq


def _closest_point_on_polyline(points: list[Point], p: Point) -> Point:
    """Nearest point to ``p`` over every segment of ``points`` (no monotonic
    constraint, unlike ``advance_progress_index`` -- this is a per-tick anchor
    lookup, not a forward-only progress cursor). Strict ``<`` keeps the first
    tie, mirroring ``_geometry.nearest_occupied``'s determinism convention."""
    best = points[0]
    best_sq = float("inf")
    for i in range(len(points) - 1):
        c = closest_point_on_segment(p, points[i], points[i + 1])
        d = sq_dist(p, c)
        if d < best_sq:
            best_sq = d
            best = c
    return best


class TebPlanner(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._w_path = params.get_float("w_path")
        self._w_obstacle = params.get_float("w_obstacle")
        self._w_velocity = params.get_float("w_velocity")
        self._w_acceleration = params.get_float("w_acceleration")
        self._w_time = params.get_float("w_time")
        self._w_kinematics = params.get_float("w_kinematics")
        self._v_max = params.get_float("v_max")
        self._omega_max = params.get_float("omega_max")
        self._a_max = params.get_float("a_max")
        self._min_obstacle_dist = params.get_float("min_obstacle_dist")
        self._dt_ref = params.get_float("dt_ref")
        self._dt_min = params.get_float("dt_min")
        self._horizon = params.get_float("horizon")
        self._iterations = params.get_int("iterations")
        self._step_alpha = params.get_float("step_alpha")
        self._max_step_xy = params.get_float("max_step_xy")
        self._max_step_theta = params.get_float("max_step_theta")
        self._max_step_dt = params.get_float("max_step_dt")
        self._max_poses = params.get_int("max_poses")
        self._reinit_distance = params.get_float("reinit_distance")

        # Band state: poses[0] is always overwritten to the executed robot pose
        # and poses[-1] to the current local goal every tick; empty = no band
        # (first tick / after reset()), forcing re-initialization.
        self._poses: list[Pose] = []
        self._dts: list[float] = []
        # Reference-path segment cursor for the monotonic forward projection
        # (advance_progress_index) -- distinct from the band's own internal
        # warm-start cursor, which tracks the band's first edge, not the path's.
        self._progress_index = 0

    @property
    def name(self) -> str:
        return "teb"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def requires_reference_path(self) -> bool:
        return True

    def reset(self) -> None:
        self._poses = []
        self._dts = []
        self._progress_index = 0

    def _clip(
        self, path: tuple[Point, ...], start_index: int, origin: Point, goal_theta: float
    ) -> tuple[Pose, list[Point]]:
        """Walks the reference path forward from ``origin`` (already projected
        onto path[start_index]->path[start_index+1]) up to arc-length
        ``horizon``, returning the local-goal pose and the clipped polyline
        (used both to anchor the path-attraction cost and, on re-init, to
        resample the initial band). If the remaining path is shorter than
        horizon, the local goal is the final goal itself (inheriting its
        theta) rather than an interpolated point."""
        points: list[Point] = [origin]
        remaining = self._horizon
        idx = start_index
        prev = origin
        while idx < len(path) - 1:
            nxt = path[idx + 1]
            seg_len = math.sqrt(sq_dist(prev, nxt))
            if remaining <= seg_len:
                if seg_len < 1e-12:
                    gx, gy = nxt
                else:
                    t = remaining / seg_len
                    gx = prev[0] + t * (nxt[0] - prev[0])
                    gy = prev[1] + t * (nxt[1] - prev[1])
                theta = math.atan2(nxt[1] - prev[1], nxt[0] - prev[0])
                points.append((gx, gy))
                return (gx, gy, theta), points
            remaining -= seg_len
            points.append(nxt)
            prev = nxt
            idx += 1
        return (path[-1][0], path[-1][1], goal_theta), points

    def _init_band(
        self, clip_points: list[Point], local_goal_theta: float
    ) -> tuple[list[Pose], list[float]]:
        """Re-initializes the band from the clipped reference path, resampled
        at v_max*dt_ref spacing. ΔT_i = ell_i/(0.5*v_max) is a deliberately
        conservative (slow) initial guess -- the time-optimality term then
        pulls it down during optimization -- rather than an attempt at the
        true achievable time."""
        spacing = self._v_max * self._dt_ref
        pts = resample_polyline(clip_points, spacing)
        poses: list[Pose] = []
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            theta = math.atan2(dy, dx) if dx * dx + dy * dy >= _EPS_SQ else 0.0
            poses.append((pts[i][0], pts[i][1], theta))
        poses.append((pts[-1][0], pts[-1][1], local_goal_theta))
        dts: list[float] = []
        for i in range(len(poses) - 1):
            ell = math.sqrt(sq_dist((poses[i][0], poses[i][1]), (poses[i + 1][0], poses[i + 1][1])))
            dts.append(ell / (0.5 * self._v_max))
        return poses, dts

    def _resize(self, poses: list[Pose], dts: list[float]) -> None:
        """Forward while-loop (mirrors Elastic Bands' overlap maintenance):
        splits an over-long interval with a wrap-aware midpoint pose, merges
        an under-short interval into its neighbor. Never touches dts[i] when
        i+1 is out of range, which keeps the merge from ever deleting the
        fixed local-goal pose at index n-1."""
        i = 0
        while i < len(dts):
            if dts[i] > 1.5 * self._dt_ref and len(poses) < self._max_poses:
                x0, y0, th0 = poses[i]
                x1, y1, th1 = poses[i + 1]
                mx = 0.5 * (x0 + x1)
                my = 0.5 * (y0 + y1)
                mth = wrap_to_pi(th0 + 0.5 * wrap_to_pi(th1 - th0))
                poses.insert(i + 1, (mx, my, mth))
                half = 0.5 * dts[i]
                dts[i] = half
                dts.insert(i + 1, half)
                continue
            if dts[i] < 0.5 * self._dt_ref and len(poses) > 3 and i + 1 < len(dts):
                del poses[i + 1]
                dts[i] += dts[i + 1]
                del dts[i + 1]
                continue
            i += 1

    def _gradient_step(
        self, poses: list[Pose], dts: list[float], anchors: list[Point], space: ObstacleQuery
    ) -> None:
        """One damped gradient-descent iteration: accumulates every cost
        term's gradient in the fixed order (a) path -> (b) obstacle ->
        (c) velocity -> (d) acceleration -> (f) kinematics -> (e) time, each
        with i ascending, then applies clamped updates (positions/theta first,
        then every ΔT) -- this order, not just the final result, is part of
        the cross-language determinism contract."""
        n = len(poses)
        gx = [0.0] * n
        gy = [0.0] * n
        gth = [0.0] * n
        gdt = [0.0] * (n - 1)

        # Per-segment quantities shared by the velocity/acceleration/
        # kinematics terms, cached once per iteration in index order.
        dxs = [0.0] * (n - 1)
        dys = [0.0] * (n - 1)
        ell = [0.0] * (n - 1)
        has_pos = [False] * (n - 1)
        v = [0.0] * (n - 1)
        omega = [0.0] * (n - 1)
        for i in range(n - 1):
            dx = poses[i + 1][0] - poses[i][0]
            dy = poses[i + 1][1] - poses[i][1]
            dxs[i], dys[i] = dx, dy
            d_sq = dx * dx + dy * dy
            if d_sq < _EPS_SQ:
                ell[i] = 0.0
                v[i] = 0.0
                has_pos[i] = False
            else:
                ell[i] = math.sqrt(d_sq)
                v[i] = ell[i] / dts[i]
                has_pos[i] = True
            omega[i] = wrap_to_pi(poses[i + 1][2] - poses[i][2]) / dts[i]

        # (a) reference-path attraction.
        for i in range(1, n - 1):
            ax, ay = anchors[i]
            c = 2.0 * self._w_path
            gx[i] += c * (poses[i][0] - ax)
            gy[i] += c * (poses[i][1] - ay)

        # (b) obstacle clearance -- continuous distance to the nearest occupied
        # cell center (not the quantized distance_to_nearest EDT), recomputed
        # every iteration since p_i moves.
        for i in range(1, n - 1):
            p = (poses[i][0], poses[i][1])
            o, d_tilde = nearest_occupied(space, p, self._min_obstacle_dist)
            if o is None:
                continue
            g_i = self._min_obstacle_dist - d_tilde
            if g_i <= 0.0:
                continue
            if d_tilde * d_tilde < _EPS_SQ:
                continue
            c = -2.0 * self._w_obstacle * g_i / d_tilde
            gx[i] += c * (p[0] - o[0])
            gy[i] += c * (p[1] - o[1])

        # (c) velocity limits.
        for i in range(n - 1):
            e_v = max(0.0, v[i] - self._v_max)
            if e_v > 0.0:
                c = 2.0 * self._w_velocity * e_v
                if has_pos[i]:
                    coeff = c / (ell[i] * dts[i])
                    gx[i] -= coeff * dxs[i]
                    gy[i] -= coeff * dys[i]
                    gx[i + 1] += coeff * dxs[i]
                    gy[i + 1] += coeff * dys[i]
                gdt[i] += c * (-v[i] / dts[i])
            e_w = max(0.0, abs(omega[i]) - self._omega_max)
            if e_w > 0.0:
                sign = 1.0 if omega[i] > 0.0 else (-1.0 if omega[i] < 0.0 else 0.0)
                c = 2.0 * self._w_velocity * e_w * sign
                gth[i] -= c / dts[i]
                gth[i + 1] += c / dts[i]
                gdt[i] += c * (-omega[i] / dts[i])

        # (d) translational acceleration limits.
        for i in range(n - 2):
            denom = 0.5 * (dts[i] + dts[i + 1])
            a_i = (v[i + 1] - v[i]) / denom
            e_a = max(0.0, abs(a_i) - self._a_max)
            if e_a <= 0.0:
                continue
            sign_a = 1.0 if a_i > 0.0 else (-1.0 if a_i < 0.0 else 0.0)
            c = 2.0 * self._w_acceleration * e_a * sign_a
            dv1 = c / denom
            dv0 = -c / denom
            if has_pos[i]:
                coeff0 = dv0 / (ell[i] * dts[i])
                gx[i] -= coeff0 * dxs[i]
                gy[i] -= coeff0 * dys[i]
                gx[i + 1] += coeff0 * dxs[i]
                gy[i + 1] += coeff0 * dys[i]
            gdt[i] += dv0 * (-v[i] / dts[i])
            if has_pos[i + 1]:
                coeff1 = dv1 / (ell[i + 1] * dts[i + 1])
                gx[i + 1] -= coeff1 * dxs[i + 1]
                gy[i + 1] -= coeff1 * dys[i + 1]
                gx[i + 2] += coeff1 * dxs[i + 1]
                gy[i + 2] += coeff1 * dys[i + 1]
            gdt[i + 1] += dv1 * (-v[i + 1] / dts[i + 1])
            gdt[i] += c * (-a_i * 0.5 / denom)
            gdt[i + 1] += c * (-a_i * 0.5 / denom)

        # (f) nonholonomic two-pose-arc kinematics (Rösmann 2012).
        for i in range(n - 1):
            th_i, th_i1 = poses[i][2], poses[i + 1][2]
            cos_sum = math.cos(th_i) + math.cos(th_i1)
            sin_sum = math.sin(th_i) + math.sin(th_i1)
            h_i = cos_sum * dys[i] - sin_sum * dxs[i]
            c = 2.0 * self._w_kinematics * h_i
            gx[i] += c * sin_sum
            gx[i + 1] -= c * sin_sum
            gy[i] -= c * cos_sum
            gy[i + 1] += c * cos_sum
            gth[i] += c * (-math.sin(th_i) * dys[i] - math.cos(th_i) * dxs[i])
            gth[i + 1] += c * (-math.sin(th_i1) * dys[i] - math.cos(th_i1) * dxs[i])

        # (e) time optimality.
        for i in range(n - 1):
            gdt[i] += self._w_time

        for i in range(1, n - 1):
            x_i, y_i, th_i = poses[i]
            x_i -= _clamp(self._step_alpha * gx[i], self._max_step_xy)
            y_i -= _clamp(self._step_alpha * gy[i], self._max_step_xy)
            th_i = wrap_to_pi(th_i - _clamp(self._step_alpha * gth[i], self._max_step_theta))
            poses[i] = (x_i, y_i, th_i)
        for i in range(n - 1):
            step = _clamp(self._step_alpha * gdt[i], self._max_step_dt)
            dts[i] = max(self._dt_min, dts[i] - step)

    def _total_cost(
        self, poses: list[Pose], dts: list[float], anchors: list[Point], space: ObstacleQuery
    ) -> float:
        """Re-evaluates every cost term (no gradient) from the final
        optimized state -- kept as a separate pass, gated by the recorder
        check at the call site, so the hot solver loop never pays for a
        scalar it only needs when tracing is on."""
        n = len(poses)
        total = 0.0
        for i in range(1, n - 1):
            ax, ay = anchors[i]
            total += self._w_path * ((poses[i][0] - ax) ** 2 + (poses[i][1] - ay) ** 2)
        for i in range(1, n - 1):
            p = (poses[i][0], poses[i][1])
            o, d_tilde = nearest_occupied(space, p, self._min_obstacle_dist)
            if o is not None:
                g_i = max(0.0, self._min_obstacle_dist - d_tilde)
                total += self._w_obstacle * g_i * g_i
        v = [0.0] * (n - 1)
        for i in range(n - 1):
            dx = poses[i + 1][0] - poses[i][0]
            dy = poses[i + 1][1] - poses[i][1]
            d_sq = dx * dx + dy * dy
            v[i] = math.sqrt(d_sq) / dts[i] if d_sq >= _EPS_SQ else 0.0
            omega_i = wrap_to_pi(poses[i + 1][2] - poses[i][2]) / dts[i]
            e_v = max(0.0, v[i] - self._v_max)
            e_w = max(0.0, abs(omega_i) - self._omega_max)
            total += self._w_velocity * (e_v * e_v + e_w * e_w)
        for i in range(n - 2):
            denom = 0.5 * (dts[i] + dts[i + 1])
            a_i = (v[i + 1] - v[i]) / denom
            e_a = max(0.0, abs(a_i) - self._a_max)
            total += self._w_acceleration * e_a * e_a
        for i in range(n - 1):
            th_i, th_i1 = poses[i][2], poses[i + 1][2]
            dx = poses[i + 1][0] - poses[i][0]
            dy = poses[i + 1][1] - poses[i][1]
            h_i = (math.cos(th_i) + math.cos(th_i1)) * dy - (math.sin(th_i) + math.sin(th_i1)) * dx
            total += self._w_kinematics * h_i * h_i
        total += self._w_time * sum(dts)
        return total

    def _emit_band(
        self,
        recorder: TraceRecorder,
        poses: list[Pose],
        dts: list[float],
        iterations: int,
        total_cost: float,
    ) -> None:
        band = [[poses[0][0], poses[0][1], poses[0][2], 0.0]]
        for i in range(1, len(poses)):
            band.append([poses[i][0], poses[i][1], poses[i][2], dts[i - 1]])
        recorder.band_updated(
            band,
            data={
                "iterations": float(iterations),
                "poses": float(len(poses)),
                "total_cost": total_cost,
                "horizon_time": sum(dts),
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
        x, y, theta = state.pose
        robot_xy: Point = (x, y)
        path = task.reference_path

        self._progress_index = advance_progress_index(path, robot_xy, self._progress_index)
        origin = closest_point_on_segment(
            robot_xy, path[self._progress_index], path[self._progress_index + 1]
        )
        local_goal, clip_points = self._clip(path, self._progress_index, origin, task.goal[2])

        need_reinit = not self._poses or math.sqrt(
            sq_dist((self._poses[-1][0], self._poses[-1][1]), (local_goal[0], local_goal[1]))
        ) > self._reinit_distance

        if need_reinit:
            poses, dts = self._init_band(clip_points, local_goal[2])
        else:
            poses, dts = self._poses, self._dts
            # Warm start: the band's own first edge, not the reference path's
            # progress index above -- the band trails the robot at whatever
            # pace optimization left it.
            while len(poses) > 2 and _segment_t(
                robot_xy, (poses[0][0], poses[0][1]), (poses[1][0], poses[1][1])
            ) >= 1.0:
                poses.pop(0)
                dts.pop(0)

        poses[0] = (x, y, theta)
        poses[-1] = local_goal

        if len(poses) < 3:
            # Degenerate: the robot is essentially at the local goal already,
            # with no internal pose to optimize -- skip the solver and steer
            # straight at the goal with proportional heading control (same
            # structure as Elastic Bands' command extraction). The gain is
            # omega_max/pi rather than a dedicated heading_gain parameter
            # (TEB declares none): alpha never exceeds pi in magnitude, so
            # this keeps the clamp a structural no-op instead of inventing an
            # unvalidated constant.
            self._poses, self._dts = poses, dts
            if recorder is not None:
                self._emit_band(recorder, poses, dts, iterations=0, total_cost=0.0)
            gx, gy, _ = local_goal
            alpha = wrap_to_pi(math.atan2(gy - y, gx - x) - theta)
            v_cmd = self._v_max * max(math.cos(alpha), 0.0)
            omega_cmd = _clamp((self._omega_max / math.pi) * alpha, self._omega_max)
            return VelocityCommand(v_cmd, omega_cmd)

        self._resize(poses, dts)

        # Anchors fixed for the whole tick: the nearest point on the clipped
        # reference path to each internal pose's *initial* position (before
        # this tick's optimization moves it) -- a moving target would make
        # the path-attraction term chase the pose it's supposed to pull.
        anchors: list[Point] = [(0.0, 0.0)] * len(poses)
        for i in range(1, len(poses) - 1):
            anchors[i] = _closest_point_on_polyline(clip_points, (poses[i][0], poses[i][1]))

        for _ in range(self._iterations):
            self._gradient_step(poses, dts, anchors, space)

        self._poses, self._dts = poses, dts

        if recorder is not None:
            total_cost = self._total_cost(poses, dts, anchors, space)
            self._emit_band(
                recorder, poses, dts, iterations=self._iterations, total_cost=total_cost
            )

        dx0 = poses[1][0] - poses[0][0]
        dy0 = poses[1][1] - poses[0][1]
        ell0 = math.sqrt(dx0 * dx0 + dy0 * dy0)
        sigma = 1.0 if dx0 * math.cos(theta) + dy0 * math.sin(theta) >= 0.0 else -1.0
        v_cmd = _clamp(sigma * ell0 / dts[0], self._v_max)
        omega_cmd = _clamp(wrap_to_pi(poses[1][2] - poses[0][2]) / dts[0], self._omega_max)
        return VelocityCommand(v_cmd, omega_cmd)
