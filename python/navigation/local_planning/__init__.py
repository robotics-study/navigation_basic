"""Local planning: closed-loop control-tick planners. Depend on `core` abstractions only.

`simulation.py` (the closed-loop tick harness) and `_geometry.py` (shared angle
utility) live at this package root because they are used across both families
below, not owned by either:
- `reactive` — reactive planners that react to the instantaneous obstacle field
  each tick (Potential Fields, VFH), no reference path required.
- `tracking` — planners that follow a precomputed `reference_path` (Pure Pursuit).

No algorithm module imports another; shared per-family skeletons follow the
`global_planning/{search,sampling}` private `_`-prefixed module precedent.
"""

from __future__ import annotations
