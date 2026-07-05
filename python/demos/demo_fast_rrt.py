#!/usr/bin/env python3
"""Fast-RRT demo."""

from __future__ import annotations

from demo_common import run_sampling

from nav_study.global_planning.fast_rrt import FastRRT

if __name__ == "__main__":
    run_sampling("fast_rrt", FastRRT)
