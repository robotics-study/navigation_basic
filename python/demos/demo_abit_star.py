#!/usr/bin/env python3
"""abit_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import ABITStar

if __name__ == "__main__":
    run_sampling("abit_star", ABITStar)
