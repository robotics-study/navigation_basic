#!/usr/bin/env python3
"""Kinodynamic RRT* demo (Webb & van den Berg 2013).

Binds the occupancy grid as a `SamplingSpace` (the planner owns its double-integrator
dynamics and only queries (x, y) validity), so it reuses the sampling demo flow.
"""

from __future__ import annotations

from demo_common import run_sampling

# Imported by full module path: the sampling package __init__ is not edited by this
# feature, so the class is not re-exported from navigation.global_planning.
from navigation.global_planning.sampling.kinodynamic_rrt_star import KinodynamicRRTStar

if __name__ == "__main__":
    run_sampling("kinodynamic_rrt_star", KinodynamicRRTStar)
