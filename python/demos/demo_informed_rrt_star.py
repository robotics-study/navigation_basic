#!/usr/bin/env python3
"""Informed RRT* demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import InformedRRTStar

if __name__ == "__main__":
    run_sampling("informed_rrt_star", InformedRRTStar)
