#!/usr/bin/env python3
"""Dijkstra demo."""

from __future__ import annotations

from demo_common import run_discrete

from navigation.global_planning.dijkstra import Dijkstra

if __name__ == "__main__":
    run_discrete("dijkstra", Dijkstra)
