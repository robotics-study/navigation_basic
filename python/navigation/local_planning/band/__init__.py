"""Band local planners: continuously deform a polyline against the obstacle
field each tick (Elastic Bands bubble chain, TEB timed pose chain).

`_band.py` holds the geometry this family's planners share -- nearest-occupied
lookup and arc-length polyline resampling -- mirroring the `tracking/_path.py`
precedent for per-family shared machinery.
"""

from __future__ import annotations

from .elastic_bands import ElasticBandsPlanner

__all__ = ["ElasticBandsPlanner"]
