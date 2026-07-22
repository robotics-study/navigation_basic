#!/usr/bin/env python3
"""DWA demo."""

from __future__ import annotations

from demo_common import run_local

# Imported from the submodule directly, mirroring demo_pure_pursuit.py's
# precedent (the package re-export lives in
# navigation/local_planning/reactive/__init__.py).
from navigation.local_planning.reactive.dwa import Dwa

if __name__ == "__main__":
    run_local("dwa", Dwa)
