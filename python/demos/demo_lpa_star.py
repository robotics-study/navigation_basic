#!/usr/bin/env python3
"""LPA* demo (lifelong replanning driver: plan, apply scenario revisions, re-plan)."""

from __future__ import annotations

from demo_common import run_replan

from navigation.global_planning import LpaStar

if __name__ == "__main__":
    run_replan("lpa_star", LpaStar)
