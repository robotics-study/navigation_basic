#!/usr/bin/env python3
"""Lazy Theta* demo."""

from __future__ import annotations

from demo_common import run_discrete

# Imported from the concrete module rather than the package top level so the demo
# runs before the flat re-export is added to navigation.global_planning.__init__.
from navigation.global_planning.search.lazy_theta_star import LazyThetaStar

if __name__ == "__main__":
    run_discrete("lazy_theta_star", LazyThetaStar)
