#!/usr/bin/env python3
"""AD* (Anytime Dynamic A*) demo."""

from __future__ import annotations

from demo_common import run_discrete

from navigation.global_planning import ADStar

if __name__ == "__main__":
    run_discrete("ad_star", ADStar)
