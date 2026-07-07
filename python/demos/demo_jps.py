#!/usr/bin/env python3
"""JPS demo."""

from __future__ import annotations

from demo_common import run_discrete

from navigation.global_planning.search.jps import JPS

if __name__ == "__main__":
    run_discrete("jps", JPS)
