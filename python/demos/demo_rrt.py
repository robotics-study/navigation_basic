#!/usr/bin/env python3
"""RRT demo."""

from __future__ import annotations

from demo_common import run_sampling

from nav_study.global_planning.rrt import RRT

if __name__ == "__main__":
    run_sampling("rrt", RRT)
