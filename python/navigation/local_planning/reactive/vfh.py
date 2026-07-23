"""Vector Field Histogram (Borenstein & Koren 1991): polar-histogram reactive
obstacle avoidance. Each tick bins nearby obstacles by bearing into a circular
histogram, smooths it, finds the free "valleys" below a density threshold, and
steers toward whichever valley best serves the goal direction.
"""

from __future__ import annotations

import math

from navigation.core.capabilities import Capability, ObstacleQuery
from navigation.core.params import ParamError, ParamSet
from navigation.core.planner import ObstacleLocalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import LocalTask, RobotState, VelocityCommand

from .._geometry import heading_command, wrap_to_pi

# (start_sector, end_sector, width): a maximal circular run of sectors whose
# smoothed histogram value is below threshold. `end_sector` is reached from
# `start_sector` by walking +1 (mod n) `width` times -- the run may wrap past
# sector n-1 back to 0, so the two borders are not necessarily start <= end.
_Valley = tuple[int, int, int]


def _wrap_2pi(angle: float) -> float:
    # VFH bins bearings into sector 0..n-1 over a full turn, so it needs [0, 2pi)
    # rather than the (-pi, pi] range `wrap_to_pi` (shared with PF/PP) provides --
    # kept local since no other reactive/tracking planner needs a 2pi range.
    wrapped = math.fmod(angle, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped


class Vfh(ObstacleLocalPlanner):
    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._num_sectors = params.get_int("num_sectors")
        self._window_radius = params.get_float("window_radius")
        self._threshold = params.get_float("threshold")
        self._smoothing_window = params.get_int("smoothing_window")
        self._wide_valley_sectors = params.get_int("wide_valley_sectors")
        self._h_m = params.get_float("h_m")
        self._k_omega = params.get_float("k_omega")
        self._max_speed = params.get_float("max_speed")
        self._max_omega = params.get_float("max_omega")
        # param_schema.json has no "must be odd" constraint, so this is enforced
        # here at construction (load time, not the compute_command hot path) --
        # an even window has no unambiguous center sector for the moving average.
        if self._smoothing_window % 2 == 0:
            raise ParamError(
                f"param error: 'smoothing_window' must be odd, got {self._smoothing_window}"
            )
        self._delta = 2.0 * math.pi / self._num_sectors

    @property
    def name(self) -> str:
        return "vfh"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.OBSTACLE_QUERY}

    def _sector_of(self, angle: float) -> int:
        k = int(_wrap_2pi(angle) / self._delta)
        return min(k, self._num_sectors - 1)  # fp guard: angle ~ 2pi rounds up to n

    def _sector_center(self, index: float) -> float:
        # `index` need not be reduced mod n first -- wrap_to_pi normalizes any
        # angle congruent mod 2*pi to the same (-pi, pi] value.
        return wrap_to_pi((index + 0.5) * self._delta)

    def _build_histogram(self, space: ObstacleQuery, x: float, y: float) -> list[float]:
        h = [0.0] * self._num_sectors
        for ox, oy in space.occupied_within((x, y), self._window_radius):
            beta = math.atan2(oy - y, ox - x)
            k = self._sector_of(beta)
            d = math.hypot(ox - x, oy - y)
            # c^2*(a - b*d) with a=1, b=1/window_radius, c=1 (Borenstein & Koren
            # 1991 eq. 5) for a known/static map: normalized so d=0 -> m=1 and
            # d=window_radius -> m=0, near obstacles dominate the vote.
            m = (1.0 - d / self._window_radius) ** 2
            h[k] += m
        return h

    def _smooth(self, h: list[float]) -> list[float]:
        n = self._num_sectors
        half = self._smoothing_window // 2
        out = [0.0] * n
        for k in range(n):
            total = 0.0
            for j in range(-half, half + 1):
                total += h[(k + j) % n]
            out[k] = total / self._smoothing_window
        return out

    def _find_valleys(self, below: list[bool]) -> list[_Valley]:
        n = self._num_sectors
        if all(below):
            return [(0, n - 1, n)]
        if not any(below):
            return []
        # Rotate the circular scan to start right after a known-False sector, so
        # a single linear pass never has to special-case a run that wraps n-1->0.
        cut = next(i for i in range(n) if not below[i])
        order = [(cut + 1 + i) % n for i in range(n)]
        valleys: list[_Valley] = []
        run_start: int | None = None
        run_end = -1
        for idx in order:
            if below[idx]:
                if run_start is None:
                    run_start = idx
                run_end = idx
            elif run_start is not None:
                width = ((run_end - run_start) % n) + 1
                valleys.append((run_start, run_end, width))
                run_start = None
        return valleys

    @staticmethod
    def _valley_contains(valley: _Valley, k: int, n: int) -> bool:
        start, _end, width = valley
        return ((k - start) % n) < width

    @staticmethod
    def _circular_dist(a: int, b: int, n: int) -> int:
        return min((a - b) % n, (b - a) % n)

    def _nearest_sector_gap(self, valley: _Valley, k_target: int) -> int:
        # Selection key (Borenstein & Koren 1991 §IV): the valley whose nearest
        # BORDER sits closest to k_target in sector count -- not the valley whose
        # resulting steering direction happens to have the smallest angle to the
        # goal. The two differ for a valley classified narrow only because its
        # width fell just under wide_valley_sectors: its "steer to center" rule
        # can aim far from the goal even though its border sits right next to
        # k_target, while a true one-sector sliver directly on the goal bearing
        # would otherwise out-rank it by angle alone despite being farther (and a
        # worse gap to thread).
        start, end, width = valley
        n = self._num_sectors
        if self._valley_contains(valley, k_target, n):
            return 0
        return min(self._circular_dist(start, k_target, n), self._circular_dist(end, k_target, n))

    def _candidate_direction(self, valley: _Valley, k_target: int, goal_dir: float) -> float:
        start, end, width = valley
        n = self._num_sectors
        # Half the wide-valley threshold also doubles as the minimum border
        # standoff a target bearing needs before "steer straight at the goal" is
        # trusted: a target sector right next to a blocked neighbor is one
        # position update away from being outside the valley altogether (the
        # obstacle border does not move with the robot's discretization), so it
        # gets the same border-hugging treatment as a target outside the valley.
        margin = min(self._wide_valley_sectors // 2, (width - 1) // 2)
        if self._valley_contains(valley, k_target, n) and min(
            self._circular_dist(start, k_target, n), self._circular_dist(end, k_target, n)
        ) >= margin:
            return goal_dir
        if width >= self._wide_valley_sectors:
            # Wide (Borenstein & Koren 1991 §IV): don't aim for the far-away full
            # width center -- steer just inside the border nearest the goal
            # bearing, by half the wide-valley threshold, so the path hugs the
            # opening closest to the target instead of detouring through the
            # valley's middle.
            offset = self._wide_valley_sectors // 2
            if self._circular_dist(start, k_target, n) <= self._circular_dist(end, k_target, n):
                idx = start + offset
            else:
                idx = end - offset
            return self._sector_center(float(idx))
        # Narrow: aim for the valley's own center.
        center_idx = start + (width - 1) / 2.0
        return self._sector_center(center_idx)

    def compute_command(
        self,
        space: ObstacleQuery,
        state: RobotState,
        task: LocalTask,
        dt: float,
        recorder: TraceRecorder | None = None,
    ) -> VelocityCommand:
        x, y, theta = state.pose
        h = self._build_histogram(space, x, y)
        smoothed = self._smooth(h)
        below = [v < self._threshold for v in smoothed]
        valleys = self._find_valleys(below)
        # A run narrower than the smoothing kernel is finer than the histogram's
        # own resolution (the moving average can't distinguish it from smoothing
        # ripple around the threshold), so it is discarded as noise rather than
        # trusted as a real gap -- unless it is literally the only opening, in
        # which case reporting it is still better than declaring the sectors
        # blocked outright.
        wide_enough = [v for v in valleys if v[2] >= self._smoothing_window]
        if wide_enough:
            valleys = wide_enough

        goal_dir = math.atan2(task.goal[1] - y, task.goal[0] - x)

        if not valleys:
            # Every sector is at/above threshold: no admissible heading exists.
            # heading_command's cos-gate wouldn't fire on its own (theta_err=0 is
            # possible), so this stops the robot explicitly and lets the
            # simulator's stall detector end the episode.
            if recorder is not None:
                recorder.histogram_updated(
                    [x, y, theta],
                    smoothed,
                    data={
                        "threshold": self._threshold,
                        "target_direction": goal_dir,
                        "selected_direction": goal_dir,
                    },
                )
            return VelocityCommand(0.0, 0.0)

        k_target = self._sector_of(goal_dir)
        candidates = [self._candidate_direction(v, k_target, goal_dir) for v in valleys]
        costs = [abs(wrap_to_pi(d - goal_dir)) for d in candidates]
        gaps = [self._nearest_sector_gap(v, k_target) for v in valleys]
        # Tie-break equally-near valleys by width (wider = more clearance to work
        # with) rather than by raw scan order -- two valleys can sit at the same
        # sector distance from k_target while one is a much safer, roomier gap.
        selected = min(range(len(candidates)), key=lambda i: (gaps[i], -valleys[i][2]))
        theta_sel = candidates[selected]

        if recorder is not None:
            recorder.histogram_updated(
                [x, y, theta],
                smoothed,
                data={
                    "threshold": self._threshold,
                    "target_direction": goal_dir,
                    "selected_direction": theta_sel,
                },
            )
            for i, direction in enumerate(candidates):
                probe = (
                    x + self._window_radius * math.cos(direction),
                    y + self._window_radius * math.sin(direction),
                )
                recorder.candidate_evaluated(
                    probe,
                    costs[i],
                    data={"direction": direction, "selected": 1.0 if i == selected else 0.0},
                )

        h_sel = smoothed[self._sector_of(theta_sel)]
        v_eff = self._max_speed * (1.0 - min(h_sel, self._h_m) / self._h_m)
        return heading_command(wrap_to_pi(theta_sel - theta), self._k_omega, v_eff, self._max_omega)
