#!/usr/bin/env python3
"""Theta* demo."""

from __future__ import annotations

from demo_common import run_discrete

from navigation.global_planning import ThetaStar

if __name__ == "__main__":
    run_discrete("theta_star", ThetaStar)
