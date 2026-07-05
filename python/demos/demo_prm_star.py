#!/usr/bin/env python3
"""prm_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import PRMStar

if __name__ == "__main__":
    run_sampling("prm_star", PRMStar)
