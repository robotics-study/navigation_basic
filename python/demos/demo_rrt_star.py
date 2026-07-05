#!/usr/bin/env python3
"""RRT* demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import RRTStar

if __name__ == "__main__":
    run_sampling("rrt_star", RRTStar)
