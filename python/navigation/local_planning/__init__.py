"""Local planning: closed-loop control-tick planners. Depend on `core` abstractions only.

`simulation.py` (the closed-loop tick harness) and `_geometry.py` (shared angle
and path-geometry primitives) live at this package root because they are used
across the families below, not owned by any one of them:
- `reactive` — reactive planners that react to the instantaneous obstacle field
  each tick (Potential Fields, VFH), no reference path required.
- `tracking` — planners that follow a precomputed `reference_path` (Pure Pursuit).
- `band` — planners that keep a deformable band/trajectory between robot and
  goal and re-shape it each tick (Elastic Bands, TEB).

No algorithm module imports another; shared per-family skeletons follow the
`global_planning/{search,sampling}` private `_`-prefixed module precedent.
"""

from __future__ import annotations
