"""Elastic Bands (Quinlan & Khatib, "Elastic bands: connecting path planning
and control," ICRA 1993, DOI 10.1109/ROBOT.1993.291936): represents a corridor
of free space around a reference path as a chain of bubbles -- world-space
discs sized to local clearance -- and deforms the chain every tick under an
internal contraction force (keeps it taut) and an external repulsion force
(keeps it off obstacles), then drives toward a lookahead point on the deformed
polyline instead of tracking a fixed discrete path.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, Point, RobotState, VelocityCommand

from .._geometry import sq_dist, wrap_to_pi
from ._band import point_at_arclength, resample_polyline

# Below this squared norm a unit-vector division is unstable, so the term is
# skipped instead -- same 1e-12 threshold as tracking/_path.py's
# segment-degeneracy guard, applied uniformly to every division-by-distance in
# this file (contraction neighbor terms, repulsion terms, tangent removal).
_EPS_SQ = 1e-12


class ElasticBandsPlanner(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._k_contraction = params.get_float("k_contraction")
        self._k_repulsion = params.get_float("k_repulsion")
        self._rho_max = params.get_float("rho_max")
        self._rho_influence = params.get_float("rho_influence")
        self._rho_min = params.get_float("rho_min")
        self._step_size = params.get_float("step_size")
        self._deform_iterations = params.get_int("deform_iterations")
        self._repair_iterations = params.get_int("repair_iterations")
        self._repair_step = params.get_float("repair_step")
        self._overlap_factor = params.get_float("overlap_factor")
        self._max_bubbles = params.get_int("max_bubbles")
        self._bubble_spacing = params.get_float("bubble_spacing")
        self._lookahead_distance = params.get_float("lookahead_distance")
        self._heading_gain = params.get_float("heading_gain")
        self._v_max = params.get_float("v_max")
        self._omega_max = params.get_float("omega_max")

        # Band state: parallel center/radius lists. Empty = no band (first tick,
        # after reset(), or right after the previous tick broke) -- the next tick
        # re-initializes from task.reference_path.
        self._centers: list[Point] = []
        self._radii: list[float] = []

    @property
    def name(self) -> str:
        return "elastic_bands"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def requires_reference_path(self) -> bool:
        return True

    def reset(self) -> None:
        self._centers = []
        self._radii = []

    def _clearance(self, space: ObstacleQuery, p: Point) -> float:
        return min(space.distance_to_nearest(p), self._rho_max)

    def _initialize(self, space: ObstacleQuery, task: LocalTask, robot_xy: Point) -> None:
        """Builds the initial band: the reference path resampled at even
        arc-length spacing, anchored at the robot's position and the goal, then
        repaired with repair_iterations of deformation so a bubble that starts
        inside an obstacle (rho=0, when the raw reference path cuts through one)
        gets a chance to push itself out -- via the repulsion summation's
        floor-clamped step (see `_deform_once`) -- before regular per-tick
        deformation begins."""
        resampled = resample_polyline(list(task.reference_path), self._bubble_spacing)
        goal_xy: Point = (task.goal[0], task.goal[1])
        self._centers = [robot_xy, *resampled, goal_xy]
        self._radii = [self._clearance(space, c) for c in self._centers]
        for _ in range(self._repair_iterations):
            self._deform_once(space)

    def _deform_once(self, space: ObstacleQuery) -> None:
        """One Jacobi pass over interior bubbles: every force is computed from a
        snapshot of the pass-start band, then every displacement is applied as a
        batch afterward -- so the order bubbles happen to be visited in never
        changes what force any of them sees (Quinlan & Khatib 1993 leave the
        update order unspecified; Jacobi is the order-independent choice, which
        also keeps py/C++ numerically identical)."""
        n = len(self._centers)
        centers = self._centers
        radii = self._radii
        deltas: list[Point] = [(0.0, 0.0)] * n
        for i in range(1, n - 1):
            cx, cy = centers[i]

            # Internal contraction: pulls bubble i toward both neighbors (keeps
            # the band taut). Tangential component kept -- it is what equalizes
            # bubble spacing along the band.
            fcx = fcy = 0.0
            for j in (i - 1, i + 1):
                nx, ny = centers[j]
                dx, dy = nx - cx, ny - cy
                d_sq = dx * dx + dy * dy
                if d_sq < _EPS_SQ:
                    continue
                d = math.sqrt(d_sq)
                fcx += dx / d
                fcy += dy / d
            fcx *= self._k_contraction
            fcy *= self._k_contraction

            # External repulsion: summed over every occupied cell within
            # rho_influence, in occupied_within's row/col-ascending order --
            # summation (not just the nearest cell) so a bubble embedded inside a
            # multi-cell obstacle still feels a net push toward its nearest edge
            # rather than one that might point inward from a single sampled cell.
            frx = fry = 0.0
            for ox, oy in space.occupied_within((cx, cy), self._rho_influence):
                dx, dy = cx - ox, cy - oy
                d_sq = dx * dx + dy * dy
                if d_sq < _EPS_SQ:
                    continue
                d = math.sqrt(d_sq)
                frx += (self._rho_influence - d) * dx / d
                fry += (self._rho_influence - d) * dy / d
            frx *= self._k_repulsion
            fry *= self._k_repulsion

            # Tangent-component removal applies to the repulsive force only
            # (Quinlan & Khatib 1993): its tangential part would slide bubbles
            # along the band and bunch them together, whereas the contraction
            # force's tangential part is exactly what equalizes spacing above.
            tx = centers[i + 1][0] - centers[i - 1][0]
            ty = centers[i + 1][1] - centers[i - 1][1]
            t_sq = tx * tx + ty * ty
            if t_sq >= _EPS_SQ:
                t_norm = math.sqrt(t_sq)
                tx, ty = tx / t_norm, ty / t_norm
                proj = frx * tx + fry * ty
                frx -= proj * tx
                fry -= proj * ty

            dx_step = self._step_size * (fcx + frx)
            dy_step = self._step_size * (fcy + fry)
            # Displacement cap: floored at repair_step (not just 0.5*rho) so a
            # bubble that starts with rho=0 (inside an obstacle) still moves
            # instead of being permanently clamped to zero step.
            limit = max(0.5 * radii[i], self._repair_step)
            mag_sq = dx_step * dx_step + dy_step * dy_step
            if mag_sq > limit * limit:
                scale = limit / math.sqrt(mag_sq)
                dx_step *= scale
                dy_step *= scale
            deltas[i] = (dx_step, dy_step)

        for i in range(1, n - 1):
            dx_step, dy_step = deltas[i]
            cx, cy = centers[i]
            new_center = (cx + dx_step, cy + dy_step)
            centers[i] = new_center
            radii[i] = self._clearance(space, new_center)

    def _maintain(self, space: ObstacleQuery) -> bool:
        """Overlap maintenance: deletes bubbles made redundant by their
        neighbors' growth, inserts a midpoint bubble across any gap too wide to
        guarantee a collision-free interpolation, and reports whether the band
        survived (False = broken, either because a gap couldn't be bridged
        within max_bubbles or because the bridging midpoint itself sits below
        rho_min). When maintenance completes without breaking, this also
        performs the post-maintenance validity check: any surviving interior
        bubble below rho_min means the earlier deform/repair passes could not
        push the band clear of an obstacle."""
        centers = self._centers
        radii = self._radii
        i = 0
        while i < len(centers) - 1:
            if i + 2 < len(centers):
                gap = math.sqrt(sq_dist(centers[i], centers[i + 2]))
                if gap <= self._overlap_factor * (radii[i] + radii[i + 2]):
                    del centers[i + 1]
                    del radii[i + 1]
                    continue
            gap = math.sqrt(sq_dist(centers[i], centers[i + 1]))
            if gap > self._overlap_factor * (radii[i] + radii[i + 1]):
                if len(centers) >= self._max_bubbles:
                    return False
                mid: Point = (
                    (centers[i][0] + centers[i + 1][0]) / 2.0,
                    (centers[i][1] + centers[i + 1][1]) / 2.0,
                )
                rho_mid = self._clearance(space, mid)
                if rho_mid < self._rho_min:
                    return False
                centers.insert(i + 1, mid)
                radii.insert(i + 1, rho_mid)
                continue
            i += 1
        return all(radii[k] >= self._rho_min for k in range(1, len(radii) - 1))

    def _emit_band(self, recorder: TraceRecorder, broken: int) -> None:
        band = [[cx, cy, r] for (cx, cy), r in zip(self._centers, self._radii, strict=True)]
        recorder.band_updated(
            band,
            data={
                "iterations": float(self._deform_iterations),
                "bubbles": float(len(self._centers)),
                "broken": float(broken),
            },
        )

    def _on_broken(self, recorder: TraceRecorder | None) -> VelocityCommand:
        # The band is serialized exactly as it stood at the moment maintenance
        # broke it (not rolled back to the last valid band) -- a single fixed
        # rule shared with the byte-identical parity fixture. The internal band
        # is then discarded so the next tick re-initializes and repairs from
        # scratch (recovers if, e.g., a sandbox wall painted over the band gets
        # erased again).
        if recorder is not None:
            self._emit_band(recorder, broken=1)
        self._centers = []
        self._radii = []
        return VelocityCommand(0.0, 0.0)

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

        if not self._centers:
            self._initialize(space, task, robot_xy)
            if not self._maintain(space):
                return self._on_broken(recorder)

        # Front pruning: drop bubbles the robot has already advanced past (its
        # position sits within the next bubble's clearance disc), then anchor
        # c_0 to the executed pose -- the two endpoints are never moved by
        # deformation, only re-pinned here and at initialization.
        while len(self._centers) > 2 and (
            math.sqrt(sq_dist(robot_xy, self._centers[1])) <= self._radii[1]
        ):
            del self._centers[0]
            del self._radii[0]
        self._centers[0] = robot_xy
        self._radii[0] = self._clearance(space, robot_xy)

        for _ in range(self._deform_iterations):
            self._deform_once(space)
        if not self._maintain(space):
            return self._on_broken(recorder)

        if recorder is not None:
            self._emit_band(recorder, broken=0)

        # Command extraction: a lookahead point on the deformed band's own
        # polyline, tracked with plain proportional heading control. The band is
        # re-deformed whole every tick (not a progress-indexed path), so the
        # tracking family's lookahead-circle intersection doesn't apply here.
        target = point_at_arclength(self._centers, self._lookahead_distance)
        alpha = wrap_to_pi(math.atan2(target[1] - y, target[0] - x) - theta)
        v = self._v_max * max(math.cos(alpha), 0.0)
        omega = max(-self._omega_max, min(self._omega_max, self._heading_gain * alpha))
        return VelocityCommand(v, omega)
