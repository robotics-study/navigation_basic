#!/usr/bin/env python3
"""Vector Field Histogram demo."""

from __future__ import annotations

from demo_common import run_local

from navigation.local_planning.reactive.vfh import Vfh

if __name__ == "__main__":
    run_local("vfh", Vfh)
