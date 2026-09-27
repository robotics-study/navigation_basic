#!/usr/bin/env python3
"""VO demo."""

from __future__ import annotations

from demo_common import run_agents

from navigation.local_planning.velocity.vo import Vo

if __name__ == "__main__":
    run_agents("vo", Vo)
