---
title: EIT*
layout: default
parent: Algorithms
grand_parent: English
nav_order: 13
---

[🇰🇷 한국어](../../ko/algorithms/eit_star.md) | [🇬🇧 English](eit_star.md)

# EIT\* (Effort Informed Trees)
{: .no_toc }

| Item | Description |
|---|---|
| Category | sampling-based, batch, anytime, asymptotically optimal, asymmetric bidirectional |
| Required capability | `SamplingSpace` |
| Completeness | probabilistically complete |
| Optimality | **almost-surely asymptotically optimal** |
| Extra property | surfaces **cheaper-to-validate** solutions among near-equal-cost candidates |
| Original paper | Strub & Gammell (2022, IJRR) [^strub_eit] |

1. TOC
{:toc}

## Background

Strub & Gammell's[^strub_eit] EIT\* continues the AIT\* lineage. AIT\* runs a **reverse search** from
the goal to produce an adaptive cost-to-go heuristic over the random geometric graph (RGG); a
**forward best-first search** consumes it, performs the real collision checks, and feeds edges found
in collision back so the heuristic adapts. EIT\* adds an estimate of **validation effort** — how
expensive it is to collision-check the remaining path — so that among candidates of near-equal cost
the forward search prefers the ones that are **cheaper to check** (fewer edges, shorter segments),
surfacing feasible solutions sooner. The motivation is that in high-dimensional or
expensive-to-validate spaces the collision checker, not the graph search, dominates runtime.

This module implements the **faithful core** but deliberately scopes it down (see
[Implementation simplification](#implementation-simplification)). The central idea — dual cost+effort
reverse heuristics feeding a lexicographically ordered forward search, with adaptive edge-invalidation
feedback — is implemented as-is; incremental repair, a fully joint multi-objective search, and a
learned validator-cost model are replaced with simple stand-ins.

## How It Works

Each batch performs the following.

1. **Grow the RGG** — draw `batch_size` informed samples (Gammell et al. 2014[^gammell] ellipse),
   keep the valid ones, and append to the accumulated sample array (0 = start, 1 = goal). Build the
   within-radius neighbour graph with `radius = γ·(log n/n)^{1/2}` and filter it against the **set of
   edges already found in collision** (persistent across batches).
2. **Reverse search (two independent Dijkstra passes, both sourced at the goal)** — over the same
   filtered graph compute
   - `ĥ(v)` = cost-to-go (edge weight = `distance`),
   - `ê(v)` = effort-to-go (edge weight = `effort`),
   each with its own **single-criterion** relaxation.
3. **Forward search** — lazy-deletion best-first. The heap key is the lexicographic tuple

   $$
   \bigl(\,g(v)+\hat h(v),\ \ e_g(v)+\hat e(v)\,\bigr),
   $$

   so **cost is primary and effort the tie-break** ("solutions of equal cost differentiated by
   effort"). Popping a vertex $v$, for each neighbour $x$ it emits `candidate_evaluated`, then
   **lazily** checks `is_motion_valid(v,x)`. If invalid, the edge is added to the persistent
   invalid set and skipped; if valid, $g$/$e_g$ are updated and the edge accepted, emitting
   `edge_added` (first connection) / `rewire` (improvement). When $x=\text{goal}$ and the cost
   improves, $c_{\text{best}}$ is updated and `path_found` emitted.

```
EIT_STAR(start, goal):
    points ← {start, goal};  invalid ← ∅;  c_best ← ∞
    for batch in 1..max_batches:
        points ← points ∪ draw(batch_size, c_best)     # informed batch (once a solution exists)
        r ← gamma · sqrt(log n / n);  N ← radius_neighbors(points, r) \ invalid
        ĥ ← dijkstra(goal, N, weight=distance)          # cost-to-go
        ê ← dijkstra(goal, N, weight=effort)            # effort-to-go
        # forward: lazy best-first over key (g+ĥ, e_g+ê)
        g[start] ← 0;  e_g[start] ← 0;  push(start)
        while heap not empty:
            v ← pop_min()                               # lexicographic minimum
            if closed[v]: continue
            closed[v] ← true
            for x in N[v]:
                emit candidate_evaluated(x, g[v]+‖v−x‖)
                if not is_motion_valid(v, x):           # lazy collision check
                    invalid ← invalid ∪ {(v,x)}; continue
                if (g[v]+‖v−x‖, e_g[v]+effort(v,x)) < (g[x], e_g[x]):
                    connect_or_rewire(x, parent=v)
                    if x = goal and g[x] < c_best:
                        c_best ← g[x];  emit path_found
    return path(goal)                                   # from the final batch's forward tree
```

Effort is defined without any new capability:

$$
\text{effort}(u,v) = \max\!\bigl(1,\ \mathrm{round}(\lVert u-v\rVert / \texttt{step\_size})\bigr),
$$

i.e. the number of `step_size`-sized sub-segments a discretized validator would check. It proxies
collision-check cost using only the `SamplingSpace` `distance`, never map internals.

$g,e_g,\text{parent}$ and the forward open heap are recomputed **fresh** each batch over the
accumulated sample array (the same simplification as AIT\* — no incremental carry-forward). Only
$c_{\text{best}}$ and the invalid-edge set persist across batches, and they only improve/grow.

## Properties

- **Completeness**: probabilistically complete[^strub_eit].
- **Optimality**: **almost-surely asymptotically optimal.** As batches accumulate the RGG densifies
  and informed samples concentrate in the solution region[^strub_eit].
- **Anytime**: once the first batch yields a solution, later batches keep tightening the path. On
  exhausting `max_batches` it returns the current best.
- **Effort bias**: among near-equal-cost candidates the lexicographic order expands the
  lower-effort one first, surfacing feasible solutions faster at the same sample budget[^strub_eit].
- **Lazy checking + adaptive feedback**: an edge is checked only when the forward search tries to
  accept it, and invalid edges accumulate in a persistent set that adapts later batches' reverse
  heuristics automatically.

## Dual Heuristics and the Lexicographic Order

**Two reverse Dijkstra passes.** Sourced at the goal over the same filtered graph, run twice — once
with `distance` weights for $\hat h$ (cost-to-go) and once with `effort` weights for $\hat e$
(effort-to-go). Both heuristics are **optimistic** estimates over the RGG without any real collision
checks, so they act as admissible-style guides steering the forward search toward the solution region.

**Lexicographic priority.** Because the forward heap key is a tuple, comparison is lexicographic —
ordered first by estimated total cost $g(v)+\hat h(v)$ and, **only on a tie**, by estimated total
effort $e_g(v)+\hat e(v)$. With continuous distances exact cost ties are rare, so effort acts mainly
as a smooth bias that breaks near-ties — advancing cheaper-to-validate solutions without harming path
optimality.

**Informed ellipse (Gammell et al. 2014).** Once a solution cost $c_{\text{best}}$ exists, later
samples are drawn only within the ellipse with foci at start and goal. With
$c_{\min}=\lVert\text{start}-\text{goal}\rVert$, the semi-axes are $r_1=c_{\text{best}}/2$ and
$r_2=\tfrac12\sqrt{c_{\text{best}}^2-c_{\min}^2}$. Any point outside this ellipse cannot lie on a
path that improves $c_{\text{best}}$, so it is excluded from sampling.

## Implementation Simplification

This module implements the paper's core mechanism (a lexicographically ordered forward search
consuming dual cost+effort reverse heuristics, with edge-invalidation feedback) as-is, but explicitly
simplifies the following.

- **Batch recomputation vs. incremental repair**: the paper repairs the reverse search incrementally
  (LPA\*-style) across batches; here the reverse and forward searches are recomputed from scratch
  each batch over the accumulated sample array. Only $c_{\text{best}}$ and the invalid-edge set carry
  across batches.
- **Two independent single-criterion Dijkstra passes vs. a joint multi-objective search**: the cost
  and effort heuristics are not combined into one multi-objective search; each is a clean
  single-criterion Dijkstra over the same graph from the same source.
- **Simple distance/step_size effort proxy**: effort is approximated by the `distance / step_size`
  discretization count rather than a learned or measured validator-cost model, keeping the planner on
  the existing `SamplingSpace` with no new capability.

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `batch_size` | int | 200 | [1, 100000] | Number of new (informed) samples drawn per batch |
| `max_batches` | int | 15 | [1, 10000] | Maximum number of batches (anytime — current best returned when exhausted) |
| `gamma` | float | 30.0 | [0.01, 1000.0] | RGG connection-radius coefficient γ. r_n = γ·(log n / n)^(1/2) |
| `step_size` | float | 0.5 | [0.01, 100.0] | Effort discretization step. effort=max(1, round(dist/step_size)) |
| `seed` | int | 1 | [0, 2^31−1] | Random seed (reproducibility) |

## Emitted Trace Events

`planning_started` → `sample_drawn`\* → `candidate_evaluated`\* → `edge_added`\* / `rewire`\* → `path_found`\* → `planning_finished`

`sample_drawn` marks a per-batch sample, `candidate_evaluated` a neighbour the forward search
considers (reporting cost as the primary metric), `edge_added` a first connection, `rewire` an
improvement, and `path_found` is emitted whenever the goal cost improves.

## References

[^strub_eit]: Strub, M. P., & Gammell, J. D. (2022). "Adaptively Informed Trees (AIT\*) and Effort Informed Trees (EIT\*): Asymmetric bidirectional sampling-based path planning." *The International Journal of Robotics Research*, 41(4), 390–417. [doi:10.1177/02783649211069572](https://doi.org/10.1177/02783649211069572) · [PDF (arXiv)](https://arxiv.org/abs/2111.01877)
[^gammell]: Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *Proc. IEEE/RSJ IROS*, 2997–3004. [doi:10.1109/IROS.2014.6942976](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
[^karaman]: Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846–894. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
