#!/usr/bin/env python3
"""Regulated Pure Pursuit demo."""

from __future__ import annotations

from demo_common import run_local

# Imported from the submodule directly, mirroring demo_pure_pursuit.py's
# precedent (the package re-export lives in navigation/local_planning/tracking/__init__.py).
from navigation.local_planning.tracking.regulated_pure_pursuit import RegulatedPurePursuit

if __name__ == "__main__":
    run_local("regulated_pure_pursuit", RegulatedPurePursuit)
