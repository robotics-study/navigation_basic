---
title: Anya
layout: default
parent: Algorithms
grand_parent: English
nav_order: 8
---

[🇰🇷 한국어](../../ko/algorithms/anya.md) | [🇬🇧 English](anya.md)

# Anya (optimal any-angle)
{: .no_toc }

| Item | Description |
|---|---|
| Category | optimal any-angle graph search (interval search) |
| Required capability | `LineOfSightSpace` (`neighbors` + `heuristic` + `line_of_sight`) |
| Completeness | complete (finite grid, non-negative costs) |
| Optimality | **true continuous Euclidean shortest any-angle path** (Harabor et al. 2016) [^anya] |
| Complexity | one interval-projection sweep per corner (each with LOS checks) |
| Original paper | Harabor, Grastien, Öz & Aksakalli (2016) [^anya] · LOS supercover: Amanatides & Woo (1987) [^aw] · A\*: Hart, Nilsson & Raphael (1968) [^hart] |

1. TOC
{:toc}

## Background

Any-angle planners like Theta\*[^theta] and Visibility A\* keep their paths on grid **cell centres**
and merely take straight-line shortcuts. They are therefore not truly shortest but **cell-centre
approximations**: when the genuinely shortest route must bend at an **obstacle corner** that no cell
centre lands on, a path pinned to cell centres is slightly longer.

**Anya**[^anya] makes a search node a `(root, interval)` pair. The `interval` is a contiguous run of
points on one grid **row** (endpoints are grid-vertex coordinates, float) and the `root` is a grid
**corner (vertex)** visible from every point of that interval. In a polygonal (blocked-cell) domain a
shortest Euclidean path is a taut string that bends **only at convex obstacle corners**, so placing
turning points (roots) exactly on those corners lets Anya return the **true continuous Euclidean
shortest any-angle path**, not a cell-centre approximation. That is the essential difference from
Visibility A\* / Theta\*.

On this repository's `maze01`, Anya reaches **cost 26.802 · 4 waypoints**, **strictly shorter** than
Visibility A\* / Theta\* (27.748) on the same instance — because it turns at corners, not cell centres.

## How It Works

Search on `maze01`. The sky-coloured fans radiating from each expanded corner root are the
**interval projection** (the root's visibility projected row by row); the final path is a sparse
straight-line polyline grazing only obstacle corners.

![Anya on maze01](../../assets/anya/maze01.gif)

Intermediate search progress (left → right: early / middle / final path):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/anya/maze01_snap_02.png) | ![mid](../../assets/anya/maze01_snap_05.png) | ![final](../../assets/anya/maze01_final.png) |

```
ANYA(start, goal):
    g[start] ← 0
    open ← priority queue keyed by f = g(root) + ‖root − goal‖   # admissible straight-line bound
    push start
    while open not empty:
        root ← open.pop_min()                 # a corner (or start), settled once
        if f(root) ≥ best_goal_cost: break
        if line_of_sight(root, goal):          # final leg: corner sees the goal directly
            relax goal via root
        for (corner, interval) in SUCCESSORS(root):   # interval projection
            relax g[corner] ← g[root] + ‖root − corner‖
    return reconstruct(goal)

SUCCESSORS(root):                              # generate (root, interval) nodes
    for dir in {up, down}:                     # --- cone successors (fans) ---
        I ← visible interval on the adjacent row from root
        while I nonempty:
            emit every convex obstacle corner inside I     # = new turning point (root)
            I ← project(I, root, next row) split at walls  # project one row further
    for dir in {left, right}:                  # --- flat successors (same row) ---
        walk along root's row, emitting the reachable corners
```

### Interval projection — cone / flat successors

Expanding a `root` **projects its visibility row by row**. The fan spreading into the adjacent rows
is a **cone successor**; the run extending along the root's own row is a **flat successor**. Each
projected interval is clipped by the obstacle walls of the cell-row it crosses into maximal
*observable* sub-intervals, and the **convex corner a wall creates becomes a new root**. Because
corners are finite, the search is really **A\* over a corner graph** — not per-cell relaxation — with
the interval sweep discovering each root's successors (the first corners it can see around).

### Line of sight — corner geometry

Anya's turning points are grid vertices, not cell centres, so visibility is decided by **corner
geometry**: a segment is traversable iff it crosses no blocked cell **interior** and does not squeeze
through the **pinch corner** between two diagonally blocked cells (the same corner-cut-forbidden model
as `neighbors()`). Crucially, **edge-grazing** is allowed: a segment lying exactly on a grid line
(integer x or y) only touches cell boundaries and enters no interior, so it is traversable when one
adjacent cell is blocked as long as the **other side is free** — this is what lets a taut path hug an
obstacle corner, and it is required for the true Euclidean optimum. (Snapping the boundary sample to
one cell with `floor` would wrongly forbid such grazing legs and miss the optimum.) The capability's
`line_of_sight` answers only cell-centre pairs and cannot express corner endpoints, so corner LOS is
computed here.

### Heuristic — Euclidean straight-line bound

The frontier is ordered by `f = g(root) + ‖root − goal‖`. `‖root − goal‖` is an admissible lower
bound because no feasible path is shorter than the straight-line distance between its endpoints
(triangle inequality). The original paper uses a tighter *per-interval* bound `h(root, I)` that
**reflects** the goal across the interval's row when the goal is on the far side (an optimisation that
expands fewer nodes); the straight-line bound alone already secures admissibility and optimality.

Measurements (Python, trace on · comparison on the same instance):

| map | Anya cost | Visibility A\* cost | Theta\* cost | Anya expanded | Anya waypoints |
|---|---|---|---|---|---|
| maze01 | **26.802** | 27.748 | 27.748 | 38 | 4 |

Reproduce:

```bash
python python/demos/demo_anya.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/anya.yaml --trace out/anya.jsonl
python tools/viz/replay.py out/anya.jsonl --gif out/anya.gif --snapshots out/anya_snaps/
```

## Properties

- **Completeness**: complete on a finite grid with non-negative costs.
- **Optimality**: by allowing corners as turning points it returns the **true continuous Euclidean
  shortest any-angle path**[^anya], with no cell-centre restriction (unlike the Visibility A\* /
  Theta\* approximation). This optimum is taut-string optimal under the **corner-cutting-forbidden
  model** (a path cannot squeeze through a diagonal pinch of two blocked cells): edge-grazing is
  permitted but diagonal pinch traversal is not, i.e. the shortest path under the same traversal rule
  as `neighbors()`.
- **Quality**: the returned cost is always **≤** Visibility A\* / Theta\* on the same grid, since the
  cell-centre optimum is an upper bound on the corner optimum (this repo's maze01: Anya 26.802 ≤
  Visibility 27.748).
- **Observed occupancy**: the planner observes only the free component reachable via `neighbors()`;
  everything else (other components, obstacles, out of bounds) is treated as blocked. Optimality
  therefore holds over the **grid-connected** region.
- **Path representation**: the returned `path` snaps corners to an incident free cell for the shared
  `list[Cell]` / viz contract; `cost` is the **exact corner-geometry Euclidean length**, not the snap.

## True Euclidean Optimality (why corners)

**Notation.** In a planar region $F\subseteq\mathbb{R}^2$ whose obstacles are the union of blocked
unit cells, a feasible path is a polyline inside $F$ and its cost is the sum of Euclidean lengths. Let
$C^\ast$ be the minimum start→goal cost.

**Proposition (a shortest path bends only at convex corners).** When the boundary of $F$ is polygonal
(the boundary of a union of axis-aligned unit squares), every interior turning point of a shortest
path $P^\ast$ is a **reflex vertex of $F$** — a convex obstacle corner.

*Reason.* If a turning point $v$ were not an obstacle vertex, a small neighbourhood of $v$ lies
entirely in $F$, so the two segments through $v$ can be locally straightened and the triangle
inequality **strictly shortens** the path — contradicting the minimality of $P^\ast$. Hence every turn
sits at an obstacle vertex that locally blocks free space convexly, i.e. a grid corner. ∎

**Corollary (Anya finds $C^\ast$).** Each segment of the optimal path is a **taut visible straight
line** between two corners (or start/goal). Anya's interval sweep discovers exactly these taut visible
corners as successors of each root, keeps $g(\text{root})$ equal to the real start→root Euclidean
length, and runs A\* with an admissible $h$. Every edge the optimal path uses is in this successor
set, so best-first expansion converges to $C^\ast$. Where Visibility A\* / Theta\* miss $C^\ast$ by
restricting turns to cell centres, Anya recovers it exactly by allowing corners. ∎

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `vertex_epsilon` | float | 1e-9 | [1e-12, 1e-3] | float tolerance for the grid-vertex / pinch test. Not a behavioural tuning knob; does not affect path optimality |

Anya is an optimal algorithm and so has no quality/speed trade-off knob (weighting it would break
optimality). `vertex_epsilon` is purely for numerical stability.

## Emitted Trace Events

`planning_started` → (`node_expanded`, `candidate_evaluated`, `edge_added`)* → `path_found` → `planning_finished`

`node_expanded` is an expanded corner root; `edge_added(state=corner, parent=root)` is a taut
any-angle edge. Its `data` carries the `(root, interval)` node the corner was discovered through, in
cell-index coordinates `{row, col_lo, col_hi}` (the same convention as Visibility A\*), so the
visualizer can draw the interval.

## References

[^anya]: Harabor, D., Grastien, A., Öz, D., & Aksakalli, V. (2016). "Optimal Any-Angle Pathfinding In Practice." *Journal of Artificial Intelligence Research*, 56, 89–118. [doi:10.1613/jair.5007](https://doi.org/10.1613/jair.5007)
[^theta]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^aw]: Amanatides, J., & Woo, A. (1987). "A Fast Voxel Traversal Algorithm for Ray Tracing." *Proc. Eurographics*, 3–10. [PDF](https://www.cse.yorku.ca/~amana/research/grid.pdf)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
