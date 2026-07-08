#!/usr/bin/env python3
"""LQR-RRT* demo (Perez et al. 2012).

Binds the occupancy grid as a `SamplingSpace` (the planner owns its double-integrator
dynamics and only queries (x, y) validity), so it reuses the sampling demo flow.
"""

from __future__ import annotations

from demo_common import run_sampling
from navigation.global_planning import LQRRRTStar

if __name__ == "__main__":
    run_sampling("lqr_rrt_star", LQRRRTStar)
