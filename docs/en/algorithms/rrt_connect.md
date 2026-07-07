---
title: RRT-Connect
layout: default
parent: Algorithms
grand_parent: English
nav_order: 10
---

[🇰🇷 한국어](../../ko/algorithms/rrt_connect.md) | [🇬🇧 English](rrt_connect.md)

# RRT-Connect

{: .no_toc }

| Item | Description |
|---|---|
| Category | sampling-based, bidirectional, single-query |
| Required capability | `SamplingSpace` |
| Completeness | probabilistically complete |
| Optimality | **non-optimal** (feasible) — does not minimize path length |
| Complexity | one EXTEND + greedy CONNECT (repeated EXTEND) per iteration |
| Original paper | Kuffner & LaValle (2000) [^kuffner] |

1. TOC
{:toc}

## Background

Kuffner and LaValle[^kuffner] extended the single-tree RRT[^lavalle] into a **bidirectional**
planner, RRT-Connect. Two trees grow simultaneously — $T_a$ rooted at the start and $T_b$ rooted at
the goal. Each iteration EXTENDs one tree one step toward a random sample, then **greedily CONNECTs
the other tree toward that new node** until it either reaches the node or is blocked; when the trees
meet, the search stops.

Because the two trees grow toward each other, there is no need to bias samples toward the goal — the
bidirectional growth itself supplies goal-directedness. This implementation therefore has no
`goal_bias` parameter.

## How It Works

The key primitive is **CONNECT**. A plain bidirectional RRT extends each tree by a single step per
iteration, but RRT-Connect greedily runs **consecutive EXTENDs** in CONNECT until it reaches the
target node or hits an obstacle. In free space one CONNECT crosses many steps at once, greatly
increasing tree growth per sample.

$$
\text{CONNECT stride} = \Big\lceil \tfrac{\lVert q_{\text{near}} - q_{\text{new}}\rVert}{\texttt{step\_size}} \Big\rceil \text{ steps advanced in a single iteration}
$$

The trees spread into unexplored space via **Voronoi bias** over the sample space (LaValle & Kuffner
2001), and because both ends spread at once, RRT-Connect threads a map with a single narrow passage
faster than a single tree.

```
RRT_CONNECT(start, goal):
    Ta ← {start};  Tb ← {goal}
    for it in 1..max_iterations:
        q_rand ← sample()                          # no goal bias
        q_new ← EXTEND(Ta, q_rand)
        if q_new ≠ Trapped:                        # Advanced
            if CONNECT(Tb, q_new) = Reached:       # the two trees meet
                return SPLICE(Ta, Tb, q_new)       # start … q_new … goal
        SWAP(Ta, Tb)                               # swap roles next iteration
    return failure

EXTEND(T, q):                                      # one step
    q_near ← nearest(T, q)
    q_new  ← steer(q_near, q, step_size)
    if is_motion_valid(q_near, q_new):
        T.add(q_new, parent = q_near)
        return q_new                               # Advanced
    return Trapped

CONNECT(T, q):                                     # repeated EXTEND until blocked
    repeat:
        s ← EXTEND(T, q)
    until s = Trapped  or  ‖s − q‖ ≤ goal_tolerance
    return Reached if ‖s − q‖ ≤ goal_tolerance else Trapped
```

**Splice.** When CONNECT ends in Reached, $T_a$ holds a branch from its root to $q_{\text{new}}$ and
$T_b$ a branch from its root to the meeting node. Joining them at the meeting point yields

$$
\text{root}(T_a) \to \cdots \to q_{\text{new}} \to \cdots \to \text{root}(T_b).
$$

Since the SWAP each iteration alternates which tree is the start tree, if the extending tree happened
to be the goal tree at splice time the whole path is reversed so the returned path **always begins at
start and ends at goal**.

**Termination.** `steer` clamps to the target once the target is within `step_size`. Each Advanced
step in CONNECT is therefore monotonic progress toward the target, and CONNECT ends in Reached or
Trapped within $\lceil \lVert\cdot\rVert/\texttt{step\_size}\rceil + 1$ steps — the inner loop needs
no separate cap.

Reproduce:

```bash
python python/demos/demo_rrt_connect.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/rrt_connect.yaml --trace out/rrt_connect.jsonl
python tools/viz/replay.py out/rrt_connect.jsonl --gif out/viz/rrt_connect/py/rrt_connect.gif \
  --snapshots out/viz/rrt_connect/py/
```

The animated GIF and intermediate PNG snapshots are generated separately by `replay.py` above
(`out/` is gitignored).

## Properties

| Property | Description |
|---|---|
| Completeness | probabilistically complete[^kuffner] — as iterations grow, the probability of finding an existing solution tends to 1 |
| Optimality | **non-optimal.** Returns the first feasible path and does not improve its length |
| Goal bias | **none** — bidirectional growth replaces it |
| Speed | the greedy CONNECT's long strides usually thread faster than single-tree RRT |
| Failure | on exhausting `max_iterations`, returns an empty path with cost 0.0 |

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `max_iterations` | int | 4000 | [1, 200000] | Maximum EXTEND/CONNECT iterations |
| `step_size` | float | 0.5 | [0.01, 100.0] | steer distance eta (meters); shared by EXTEND and CONNECT |
| `goal_tolerance` | float | 0.3 | [0.0, 100.0] | CONNECT "Reached" distance threshold (meters) |
| `seed` | int | 1 | [0, 2³¹−1] | Random seed (reproducibility) |

## Emitted Trace Events

`planning_started` → `sample_drawn`\* → `edge_added`\* → `path_found` → `planning_finished`

`sample_drawn` marks the uniform sample per iteration, `edge_added` every edge attached to either tree
by EXTEND/CONNECT, and `path_found` is emitted once for the spliced final path.

## References

[^kuffner]: Kuffner, J. J., & LaValle, S. M. (2000). "RRT-Connect: An efficient approach to single-query path planning." *Proc. IEEE ICRA*, 995–1001. [doi:10.1109/ROBOT.2000.844730](https://doi.org/10.1109/ROBOT.2000.844730)
[^lavalle]: LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning." *Technical Report TR 98-11*, Computer Science Dept., Iowa State University.
