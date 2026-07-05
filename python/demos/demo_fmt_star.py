#!/usr/bin/env python3
"""fmt_star demo."""

from __future__ import annotations

from demo_common import run_sampling

from navigation.global_planning import FMTStar

if __name__ == "__main__":
    run_sampling("fmt_star", FMTStar)
