#!/usr/bin/env python3
"""Elastic Bands demo."""

from __future__ import annotations

from demo_common import run_local

# Imported from the submodule directly, mirroring demo_dwa.py's precedent (the
# package re-export lives in navigation/local_planning/band/__init__.py).
from navigation.local_planning.band.elastic_bands import ElasticBandsPlanner

if __name__ == "__main__":
    run_local("elastic_bands", ElasticBandsPlanner)
