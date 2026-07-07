#!/usr/bin/env python3
"""fcit_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import FCITStar

if __name__ == "__main__":
    run_sampling("fcit_star", FCITStar)
