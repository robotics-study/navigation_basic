#!/usr/bin/env python3
"""SST / SST* demo."""

from __future__ import annotations

from demo_common import run_sampling

# Imported from the submodule directly: the package re-export lives in
# navigation/global_planning/__init__.py alongside the RRT family.
from navigation.global_planning.sampling.sst import SST

if __name__ == "__main__":
    run_sampling("sst", SST)
