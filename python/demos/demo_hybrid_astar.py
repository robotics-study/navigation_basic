#!/usr/bin/env python3
"""Hybrid A* demo."""

from __future__ import annotations

from demo_common import run_kinodynamic

from navigation.global_planning import HybridAStar

if __name__ == "__main__":
    run_kinodynamic("hybrid_astar", HybridAStar)
