#!/usr/bin/env python3
"""eit_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning.sampling.eit_star import EITStar

if __name__ == "__main__":
    run_sampling("eit_star", EITStar)
