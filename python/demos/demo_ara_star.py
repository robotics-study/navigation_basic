#!/usr/bin/env python3
"""ARA* demo."""

from __future__ import annotations

from demo_common import run_discrete

from navigation.global_planning import ARAStar

if __name__ == "__main__":
    run_discrete("ara_star", ARAStar)
