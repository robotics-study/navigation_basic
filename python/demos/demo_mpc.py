#!/usr/bin/env python3
"""MPC demo."""

from __future__ import annotations

from demo_common import run_local

# Imported from the submodule directly, mirroring demo_teb.py's precedent (the
# package re-export lives in navigation/local_planning/predictive/__init__.py).
from navigation.local_planning.predictive.mpc import MpcPlanner

if __name__ == "__main__":
    run_local("mpc", MpcPlanner)
