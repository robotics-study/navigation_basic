"""Angle utility shared across the local_planning category.

Category-internal `_`-prefixed module, mirroring the `global_planning/sampling/
_sampling.py` precedent for shared machinery that spans more than one family.
"""

from __future__ import annotations

import math


def wrap_to_pi(angle: float) -> float:
    """Normalize ``angle`` (radians) to (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi
