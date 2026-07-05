---
title: D* Lite
layout: default
parent: Algorithms
grand_parent: English
nav_order: 4
---

[🇰🇷 한국어](../../ko/algorithms/dstar_lite.md) | [🇬🇧 English](dstar_lite.md)

# D* Lite (dynamic replanning)
{: .no_toc }

| Item | Description |
|---|---|
| Category | incremental / dynamic replanning graph search |
| Required capability | `DynamicGridSpace` (`passable_neighbors` + `is_blocked`) |
| Completeness | complete (finite grids, non-negative costs) |
| Optimality | **optimal for the current belief** — every step follows the shortest path over the known map |
| Complexity | first search is A\* level; each replan repairs only the neighbourhood of a change (far cheaper than a from-scratch search) |
| Original paper | Koenig & Likhachev (2002) [^koenig] · built on LPA\*: Koenig, Likhachev & Furcy (2004) [^lpastar] |

1. TOC
{:toc}

## Background

A\*[^hart] assumes the map is **fully known in advance**. A real robot, however, starts with no map,
**sees only its surroundings through a sensor**, and must re-plan when it meets an unexpected obstacle.
Re-running A\* from scratch every step (naïve replanning) repeats almost all of the work.

**D\* Lite**[^koenig] solves this with **incremental search**. It maintains a **backward** search rooted
at the goal and keeps two values per cell: `g` (the cost-to-goal computed so far) and `rhs` (a one-step
look-ahead value). Only vertices where the two disagree are "inconsistent" and go on a priority queue.
When the sensor finds a cell that contradicts the belief, only **a few vertices around that cell** are
re-queued and the previous search is *repaired* — not restarted.

Here the belief is **planner-internal state**: the blocked set starts empty, so **every in-bounds cell is
assumed free** (the freespace assumption). `plan()` simulates the whole move → sense → repair loop
internally until the goal is reached or proven unreachable, and returns the **executed trajectory** (not
a from-start plan).

## How It Works

Because the search is backward, the heuristic is `h(s_start, s)` — from a search vertex `s` to the
**current robot position** `s_start`. As the robot moves, this reference point shifts, so an offset `k_m`
is accumulated to keep the keys already on the queue monotone (this is what lets the queue be reused
instead of rebuilt).

```
CalcKey(s):                                   # priority = [k1, k2] (lexicographic)
    k2 ← min(g(s), rhs(s))
    return [k2 + h(s_start, s) + k_m,  k2]

Initialize():
    U ← ∅;  k_m ← 0
    for all s: rhs(s) = g(s) = ∞
    rhs(s_goal) ← 0                            # goal is the root of the backward search
    U.insert(s_goal, CalcKey(s_goal))

UpdateVertex(u):
    if u ≠ s_goal:
        rhs(u) ← min over s' ∈ Succ(u) of ( c(u, s') + g(s') )
    remove u from U
    if g(u) ≠ rhs(u): U.insert(u, CalcKey(u))  # only inconsistent vertices sit on U

ComputeShortestPath():
    while U.top_key() < CalcKey(s_start) or rhs(s_start) ≠ g(s_start):
        (k_old, u) ← U.pop_min()
        if k_old < CalcKey(u):        U.insert(u, CalcKey(u))   # stale key: refresh
        else if g(u) > rhs(u):        g(u) ← rhs(u)             # over-consistent: relax
                                      for s ∈ Pred(u): UpdateVertex(s)
        else:                         g(u) ← ∞                  # under-consistent: raise
                                      for s ∈ Pred(u) ∪ {u}: UpdateVertex(s)

Main():
    s_last ← s_start;  Initialize();  sense(s_start);  ComputeShortestPath()
    while s_start ≠ s_goal:
        if g(s_start) = ∞: return "no path"
        s_start ← argmin over s' ∈ Succ(s_start) of ( c(s_start, s') + g(s') )   # take one step
        changed ← sense(s_start)                      # scan the sensor disk, update the belief
        if changed ≠ ∅:
            k_m ← k_m + h(s_last, s_start);  s_last ← s_start
            for c ∈ changed: UpdateVertex the vertices around it
            ComputeShortestPath()                     # local incremental repair
```

Grid motion is symmetric (undirected), so `Succ = Pred = passable_neighbors` (the belief-passable
neighbours). Each step the robot moves to the neighbour with the smallest `g` — i.e. it follows the
**shortest path over the map it currently knows**.

### Sensing and belief — owned by the capability

Every step the robot queries real occupancy (`is_blocked`) for the cells inside a **Euclidean disk** of
radius `sensor_radius` (cells) around itself (`dr² + dc² ≤ r²`). A blocked cell not yet in the belief is
added, an `obstacle_revealed` event is emitted, and only the neighbour vertices that used to route *into*
that cell are repaired with `UpdateVertex`. The **grid geometry (move table + corner-cut rule)** is owned
by the map's `passable_neighbors`, not the algorithm, so D\* Lite never touches raw coordinates.

### Heuristic — octile (backward, toward the robot)

`h(a, b)` is the **octile distance**, admissible for 8-connected motion:

```
h(a, b) = (hi − lo) + √2 · lo,   hi = max(|Δrow|, |Δcol|),  lo = min(|Δrow|, |Δcol|)
```

It is computed in **exactly the same operation order** as the map's A\* heuristic, so the C++ and Python
keys match bit-for-bit.

## Properties

- **Completeness**: complete on a finite grid with non-negative costs. If a path exists in the true map
  the robot reaches the goal; otherwise it reports unreachability via `g(s_start) = ∞`.
- **Optimality**: each move is **optimal for the belief at that instant**. Detouring around a
  first-seen obstacle can make the executed trajectory longer than the (omniscient) A\* optimum — hence
  the **realized cost ≥ the A\* cost on the same instance**. When the belief matches the true map closely
  enough, the two coincide (the maze01 / open01 demos below are such cases).
- **Incrementality**: it reuses LPA\*'s[^lpastar] g/rhs values and inconsistency queue, and the `k_m`
  offset lets the previous search survive robot motion. Each replan repairs only the neighbourhood of a
  change, which is cheaper than a naïve full replan.

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `sensor_radius` | int | 3 | [1, 50] | Sensor radius (cells). Each step senses cells with `dr² + dc² ≤ r²`. A larger radius spots obstacles from further away, so fewer replans are needed |

## Implementation Notes

- C++: `cpp/src/global_planning/search/dstar_lite.cpp`, Python: `python/navigation/global_planning/search/dstar_lite.py`
- Built on the **new `DynamicGridSpace` capability**. `OccupancyGrid2D` shares `neighbors()` (ground
  truth) and `passable_neighbors(s, blocked)` (belief) through **one 8-move + corner-cut worker** — only
  the predicate differs. `is_blocked` (occupied *or* out of bounds) is the **only ground-truth sensor**.
- `PlanResult.path` is the **executed trajectory**, not a completed plan (`cost` is its realized length).
  `stats.expanded_nodes` is cumulative over every replan and `stats.iterations` is the replan count.
- The octile is computed as `(hi − lo) + √2·lo` (sqrt, not `hypot`), so the C++/Python keys are
  bit-identical. The robot-move / sense / repair order is the same in both languages, so the emitted
  trace matches cell-for-cell.

## Emitted Trace Events

`planning_started` → ( `node_expanded`, `candidate_evaluated`, `edge_added`, `robot_moved`, `obstacle_revealed` )\* → `path_found` → `planning_finished`

- `robot_moved` (state = the robot's new executed cell) — emits the trajectory one step at a time.
- `obstacle_revealed` (state = a newly sensed blocked cell) — the moment the sensor finds an obstacle not
  yet in the belief.

When these two events are present, `replay.py` paints the background **all-free (the belief)**, fogs the
discovered obstacles in as black cells as the search progresses, and draws the robot's trail. Renders for
existing planners (no such events) are unchanged.

`planning_finished.metrics`: `path_cost` (realized trajectory cost) · `expanded_nodes` (cumulative) ·
`replan_count` · `sensed_cells` (obstacle cells discovered) · `runtime_sec`. The common metric schema is
reused unchanged.

## Demo

`maze01`. The robot (teal diamond) starts assuming an empty map and re-plans by local repair as it
discovers the walls one by one (fogged in as black cells). On this instance the discovered walls never
push it off the optimal path, so the realized trajectory equals the (omniscient) A\* optimum exactly.

![D* Lite on maze01](../../assets/dstar_lite/maze01.gif)

Intermediate search / motion progress (left → right: early / middle / final trajectory):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/dstar_lite/maze01_snap_02.png) | ![mid](../../assets/dstar_lite/maze01_snap_05.png) | ![final](../../assets/dstar_lite/maze01_final.png) |

Final result on `open01`:

![D* Lite on open01](../../assets/dstar_lite/open01_final.png)

Measurements (Python, `sensor_radius = 3`, trace on · A\* comparison on the same instance):

| map | D\* Lite realized cost | A\* cost | D\* Lite cumulative expanded | A\* expanded | replans | obstacles found |
|---|---|---|---|---|---|---|
| maze01 | 28.728 | 28.728 | 247 | 108 | 20 | 41 |
| open01 | 25.213 | 25.213 | 69 | 71 | 11 | 35 |

The realized cost equals A\*'s (the discovered obstacles never blocked the optimal path), but the
cumulative expanded count is larger — the price of catching up to an unknown map by incremental repair.
Unlike A\*, which is handed the whole true map, D\* Lite **earns that knowledge as it moves**.

Reproduce:

```bash
python python/demos/demo_dstar_lite.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/dstar_lite.yaml --trace out/dstar_lite.jsonl
python tools/viz/replay.py out/dstar_lite.jsonl --gif out/dstar_lite.gif --snapshots out/dstar_snaps/
```

## References

[^koenig]: Koenig, S., & Likhachev, M. (2002). "D\* Lite." *Proc. AAAI Conference on Artificial Intelligence*, 476–483. [PDF](https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf)
[^lpastar]: Koenig, S., Likhachev, M., & Furcy, D. (2004). "Lifelong Planning A\*." *Artificial Intelligence*, 155(1–2), 93–146. [doi:10.1016/j.artint.2003.12.001](https://doi.org/10.1016/j.artint.2003.12.001)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
