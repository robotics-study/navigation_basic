#!/usr/bin/env python3
"""Pure Pursuit demo."""

from __future__ import annotations

from demo_common import run_local

# Imported from the submodule directly, mirroring demo_sst.py's precedent (the
# package re-export lives in navigation/local_planning/tracking/__init__.py).
from navigation.local_planning.tracking.pure_pursuit import PurePursuit

if __name__ == "__main__":
    run_local("pure_pursuit", PurePursuit)
