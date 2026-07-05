#!/usr/bin/env python3
"""bit_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import BITStar

if __name__ == "__main__":
    run_sampling("bit_star", BITStar)
