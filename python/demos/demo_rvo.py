#!/usr/bin/env python3
"""RVO demo."""

from __future__ import annotations

from demo_common import run_agents

from navigation.local_planning.velocity.rvo import Rvo

if __name__ == "__main__":
    run_agents("rvo", Rvo)
