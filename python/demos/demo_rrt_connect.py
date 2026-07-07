#!/usr/bin/env python3
"""RRT-Connect demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import RRTConnect

if __name__ == "__main__":
    run_sampling("rrt_connect", RRTConnect)
