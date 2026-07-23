"""Predictive local planners: predict a horizon with the motion model and
optimize the control sequence U directly against a shared cost J(U) (MPC by
finite-difference gradient descent; MPPI by path-integral sampling).

`_rollout.py` holds the arc integrator and the cost J(U) both planners share,
mirroring the band family's `_band.py` precedent for per-family shared
machinery.
"""

from __future__ import annotations

from .mpc import MpcPlanner
from .mppi import MppiPlanner

__all__ = ["MpcPlanner", "MppiPlanner"]
