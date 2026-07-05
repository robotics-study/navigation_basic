---
title: Hybrid A*
layout: default
parent: Algorithms
grand_parent: English
nav_order: 5
---

[🇰🇷 한국어](../../ko/algorithms/hybrid_astar.md) | [🇬🇧 English](hybrid_astar.md)

# Hybrid A* (kinodynamic SE(2))
{: .no_toc }

| Item | Description |
|---|---|
| Category | kinodynamic search (continuous SE(2) pose) |
| Required capability | `SE2CollisionSpace` (`is_collision(footprint, pose)`) |
| Completeness | resolution-complete (finite bin discretization) |
| Optimality | resolution-suboptimal — feasible, not strictly optimal (Dolgov et al. 2008) |
| Complexity | A\* over discretized bins × constant-curvature motion primitives per node |
| Original paper | Dolgov, Thrun, Montemerlo & Diebel (2008) [^dolgov] |

1. TOC
{:toc}

## Background

A\*[^hart] on a grid moves cell-to-cell, so its path ignores a vehicle's **heading and turn radius** —
a car cannot follow a 90° grid staircase. **Hybrid A\***[^dolgov] searches the **continuous SE(2)**
pose space (x, y, θ) instead. It expands a node by simulating short **constant-curvature motion
primitives** (arcs the vehicle can actually drive), so every edge is kinematically feasible. To keep
the continuous search finite, it keeps a **closed set keyed on a discretized (x, y, θ) bin**: two poses
in the same bin are treated as the same search node, but the path itself stays continuous.

The name is "hybrid" because the *state* is continuous while the *visited set* is discrete.

## How It Works

```
HYBRID-A*(start, goal):
    g[bin(start)] ← 0 ; pose_of[bin(start)] ← start
    open ← priority queue keyed by f = g + h        # h = Euclidean straight-line distance
    while open not empty:
        b ← open.pop_min() ; if b settled: continue ; settle b
        p ← pose_of[b]                              # the bin's best continuous pose
        if reached_goal(p): return reconstruct(came_from)
        for (κ, L, reverse) in motion_primitives(): # forward fan, then reverse fan
            subs ← sample_arc(p, κ, L)              # dense sub-poses along the arc
            if any is_collision(footprint, s) for s in subs: continue
            child ← subs[-1] ; b2 ← bin(child)
            cand ← g[b] + arc_cost(L, κ, reverse)
            if cand < g[b2]:
                g[b2] ← cand ; pose_of[b2] ← child ; came_from[b2] ← (b, subs)
                open.push(b2, cand + h(child))
    return failure
```

### Motion primitives — the vehicle model lives in the planner

The map only answers footprint collision; the **vehicle model is entirely in the planner**, driven by
config parameters. With maximum curvature `κ_max = 1 / min_turn_radius`, the planner fans out
`num_steering` curvatures evenly spaced over `[−κ_max, +κ_max]` (an odd count includes κ = 0, straight),
each driven forward by `arc_step` and — if `allow_reverse` — also in reverse. A primitive integrates a
constant-curvature arc (θ is additive, no trig needed for heading):

```
θ' = θ + κ·L
|κ| ≈ 0:  x' = x + L·cosθ ;            y' = y + L·sinθ
else:     x' = x + (sinθ' − sinθ)/κ ;  y' = y − (cosθ' − cosθ)/κ
cost = |L|·(reverse ? reverse_penalty : 1) + steer_penalty·|κ|·|L|
```

### Footprint collision — inscribed disc, on the map

The robot is an **inscribed disc** of radius `footprint_radius`. Because a disc is orientation-invariant,
collision depends only on `(x, y)`, so there is no swept-polygon trig. The map rasterizes the disc against
its cells and reports a collision when any **occupied or out-of-bounds** cell overlaps it — via an exact
disc-vs-cell **squared-distance** test (no `sqrt`, no trig). Each arc is sub-sampled with spacing ≤
`footprint_radius` (`n_sub = max(2, ⌈arc_step / footprint_radius⌉)`) so consecutive footprint discs
overlap and a thin wall cannot slip between two collision checks (no tunnelling). The same sub-poses feed
both collision and the visualization.

### Discretization bin — planner-internal, no grid-index leakage

The closed set keys on a bin computed by the planner alone, from world coordinates and its own
`xy_resolution` / `theta_bins` — it never calls `map.world_to_cell` (that would leak the map's grid
frame into the planner):

```
bin(p) = (⌊x / xy_resolution⌋, ⌊y / xy_resolution⌋, ⌊wrap(θ) / (2π / theta_bins)⌋ mod theta_bins)
```

### Heuristic — Euclidean only

The full paper stacks two heuristics (a non-holonomic Reeds-Shepp/Dubins distance ignoring obstacles, and
a holonomic grid-Dijkstra distance ignoring the vehicle). This study deliberately implements a **single
admissible Euclidean straight-line heuristic** `h = √(Δx² + Δy²)` (`sqrt`, not `hypot`): the Reeds-Shepp
shot needs `atan2`/`acos` and heavy branch logic (a determinism liability), and grid-Dijkstra needs the
map's `neighbors()`, which the standalone `SE2CollisionSpace` deliberately does not expose. Both are noted
as deferred options. There is also **no analytic (Reeds-Shepp) expansion** to the goal; the goal is
reached within a position + heading tolerance.

## Properties

- **Completeness**: resolution-complete — a solution is found if one exists at the chosen bin resolution.
- **Optimality**: resolution-**suboptimal**. The discretized closed set and the finite primitive set mean
  the returned path is feasible and low-cost but not strictly optimal (Dolgov et al. 2008). Without
  analytic expansion the final heading is only met within `goal_heading_tolerance`.
- **Feasibility**: every edge is a constant-curvature arc within `min_turn_radius`, so the whole path is
  drivable — each step stays within one primitive and each heading change respects the curvature bound.
- **Complexity**: A\* over the (x, y, θ) bins, branching by `num_steering` (×2 with reverse) per node.

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `min_turn_radius` | float | 1.0 | [0.1, 50.0] | Minimum turn radius (m). `κ_max = 1 / min_turn_radius` |
| `arc_step` | float | 0.5 | [0.05, 20.0] | Arc length of one motion primitive (m); the g-cost unit increment |
| `num_steering` | int | 5 | [2, 51] | Discrete curvatures over `[−κ_max, +κ_max]`; odd includes straight |
| `theta_bins` | int | 72 | [4, 360] | Heading buckets for the closed-set bin |
| `xy_resolution` | float | 0.5 | [0.01, 10.0] | Closed-set position resolution (m/bin); planner-internal, not the map's |
| `footprint_radius` | float | 0.2 | [0.01, 20.0] | Inscribed-disc footprint radius (m) |
| `allow_reverse` | bool | false | — | Allow reverse primitives |
| `reverse_penalty` | float | 2.0 | [1.0, 100.0] | Cost multiplier for reverse arcs (discourages reversing) |
| `steer_penalty` | float | 0.1 | [0.0, 100.0] | Curvature-use penalty: `cost += steer_penalty·|κ|·|L|` |
| `goal_pos_tolerance` | float | 0.5 | [0.01, 20.0] | Goal position tolerance (m) |
| `goal_heading_tolerance` | float | 0.26 | [0.01, 3.1416] | Goal heading tolerance (rad) |

## Implementation Notes

- C++: `cpp/src/global_planning/search/hybrid_astar.cpp`, Python: `python/navigation/global_planning/search/hybrid_astar.py`
- **Determinism (same-libm, not IEEE cross-platform)**: unlike the pure-arithmetic grid searches, Hybrid A*'s
  `x, y` come from `sin`/`cos`, which IEEE-754 does **not** require to be correctly-rounded. CPython's `math`
  and C++ `std` wrap the **same C libm**, so on one machine the two traces — and `path_cost` /
  `expanded_nodes` — are bit-identical, but this rests on a shared-libm assumption, not an IEEE guarantee.
  `sqrt` (correctly-rounded) is used instead of `hypot`, `math` scalars instead of `numpy`, and a fixed
  primitive emission order with an `(f, insertion order)` tie-break — the maze01 expansion order and cost
  match cell-for-cell between C++ and Python.
- The path returned is the **dense sub-pose polyline** (every arc's sub-poses concatenated), so it is a
  smooth drivable curve rather than a sparse node list.

## Emitted Trace Events

`planning_started` → (`node_expanded`, `candidate_evaluated`, `edge_added`)* → `path_found` → `planning_finished`

Hybrid A* adds **no new trace event and no schema change**. Each accepted arc is emitted as a **chain of
`edge_added` chords** over its dense sub-poses, so `replay.py` draws a smooth curve from straight segments
with no arc-specific logic. States are 3-element `[x, y, θ]` (the schema already allowed a heading), and
`replay.py` overlays a **heading arrow** wherever a state carries a third element — gated so every existing
2-element planner renders byte-identically.

## Demo

Search on `maze01`. The frontier grows through the bins, but the final path is a **smooth curve** whose
heading arrows show the vehicle turning within its radius as it threads the gap.

![Hybrid A* on maze01](../../assets/hybrid_astar/maze01.gif)

Intermediate search progress (left → right: early / middle / final path):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/hybrid_astar/maze01_snap_02.png) | ![mid](../../assets/hybrid_astar/maze01_snap_05.png) | ![final](../../assets/hybrid_astar/maze01_final.png) |

Final result on `open01`:

![Hybrid A* on open01](../../assets/hybrid_astar/open01_final.png)

Reproduce:

```bash
python python/demos/demo_hybrid_astar.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/hybrid_astar.yaml --trace out/hybrid_astar.jsonl
python tools/viz/replay.py out/hybrid_astar.jsonl --gif out/hybrid_astar.gif --snapshots out/hy_snaps/
```

## References

[^dolgov]: Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2008). "Practical Search Techniques in Path Planning for Autonomous Driving." *Proc. STAIR (AAAI Workshop)*. [PDF](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
