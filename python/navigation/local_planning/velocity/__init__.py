"""Velocity-obstacle local planners: choose the next absolute velocity
directly in velocity space, outside every nearby obstacle's forbidden region
(Fiorini & Shiller 1998 / van den Berg, Lin & Manocha 2008 / van den Berg, Guy,
Lin & Manocha 2011) -- a sibling of `reactive`/`tracking`/`band`/`predictive`,
distinguished by requiring dynamic-neighbor state rather than reacting to the
static obstacle field alone.
"""

from __future__ import annotations

from .orca import Orca
from .rvo import Rvo
from .vo import Vo

__all__ = ["Vo", "Rvo", "Orca"]
