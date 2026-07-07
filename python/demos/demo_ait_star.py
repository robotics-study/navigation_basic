#!/usr/bin/env python3
"""ait_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning.sampling.ait_star import AITStar

if __name__ == "__main__":
    run_sampling("ait_star", AITStar)
