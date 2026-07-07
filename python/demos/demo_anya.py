#!/usr/bin/env python3
"""Anya demo."""

from __future__ import annotations

from demo_common import run_discrete

# Imported from the submodule directly: the package __init__ re-exports are kept
# minimal here, and the demo only needs the concrete planner.
from navigation.global_planning.search.anya import Anya

if __name__ == "__main__":
    run_discrete("anya", Anya)
