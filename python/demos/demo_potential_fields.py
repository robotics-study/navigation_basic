#!/usr/bin/env python3
"""Potential Fields demo."""

from __future__ import annotations

from demo_common import run_local

from navigation.local_planning.reactive.potential_fields import PotentialFields

if __name__ == "__main__":
    run_local("potential_fields", PotentialFields)
