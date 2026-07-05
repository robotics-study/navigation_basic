#!/usr/bin/env python3
"""BFS demo."""

from __future__ import annotations

from demo_common import run_discrete

from nav_study.global_planning.bfs import BFS

if __name__ == "__main__":
    run_discrete("bfs", BFS)
