#!/usr/bin/env python3
"""A* demo."""

from __future__ import annotations

from demo_common import run_discrete

from navigation.global_planning import AStar

if __name__ == "__main__":
    run_discrete("astar", AStar)
