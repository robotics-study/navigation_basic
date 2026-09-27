#!/usr/bin/env python3
"""ORCA demo."""

from __future__ import annotations

from demo_common import run_agents

from navigation.local_planning.velocity.orca import Orca

if __name__ == "__main__":
    run_agents("orca", Orca)
