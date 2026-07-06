---
title: PRM*
layout: default
parent: Algorithms
grand_parent: English
nav_order: 7
---

[🇰🇷 한국어](../../ko/algorithms/prm_star.md) | [🇬🇧 English](prm_star.md)

# PRM\* (PRM-star)
{: .no_toc }

| Item | Description |
|---|---|
| Category | sampling-based, roadmap, asymptotically optimal |
| Required capability | `SamplingSpace` |
| Completeness | probabilistically complete |
| Optimality | **asymptotically optimal** — converges to the optimal path with probability 1 as samples → ∞ |
| Complexity | expected Θ(log n) neighbors → near-linear edges; dominated by radius-graph construction |
| Original paper | Karaman & Frazzoli (2011) [^karaman] |

1. TOC
{:toc}

## Background

[PRM](prm.md) uses a fixed radius and therefore does not converge to the optimum even in the limit.
Karaman & Frazzoli[^karaman] showed that changing a single policy — **shrinking** the roadmap's
connection radius with the sample count — makes PRM asymptotically optimal; this is PRM\*. The
skeleton (sample → radius connect → Dijkstra query) is identical to PRM; the only difference is the
radius.

The key idea: with the radius

$$
r_n=\gamma\left(\frac{\log n}{n}\right)^{1/d},\qquad d=2
$$

each node's **expected neighbor count stays at $\Theta(\log n)$**. That density is exactly enough to
recover the optimal path in the limit while keeping the edge count near-linear. A fixed radius either
produces $O(n^2)$ edges (if large) or breaks connectivity and loses optimality (if small).

## How It Works

`maze01` — samples scatter, the shrunken radius wires each node to an expected logarithmic number of
neighbors into a roadmap, and Dijkstra extracts the shortest path.

![PRM\* on maze01](../../assets/prm_star/maze01.gif)

Intermediate search progress (left → right: early samples/edges / roadmap forming / final path):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/prm_star/maze01_snap_02.png) | ![mid](../../assets/prm_star/maze01_snap_05.png) | ![final](../../assets/prm_star/maze01_final.png) |

Final result on `open01` — nearly a straight line:

![PRM\* on open01](../../assets/prm_star/open01_final.png)

```
PRM_STAR(start, goal):
    R ← roadmap()
    R.add(start); R.add(goal)
    for i in 1..num_samples:                       # learning: sample free nodes
        q ← sample()
        if is_state_valid(q): R.add(q)
    r ← gamma · sqrt(log n / n)                     # radius shrinks with sample count (d = 2)
    for v in R.nodes:                               # learning: connect within r
        for u in R.nodes within r of v (u before v):
            if is_motion_valid(u, v):
                R.add_edge(u, v, distance(u, v))
    return DIJKSTRA(R, start, goal)                 # query
```

The radius is computed **once** from the final node count $n$ and used for every node. That is the
only code difference from PRM.

Measurements (Python, seed = 1, trace on):

| map | path cost | roadmap nodes | expanded (Dijkstra pops) |
|---|---|---|---|
| maze01 | 13.595 | 1,502 | 1,092 |
| open01 | 12.053 | — | — |

The C++ implementation mirrors the same scenario and produces matching results within the variance of
the two languages' random streams.

Reproduce:

```bash
python python/demos/demo_prm_star.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/prm_star.yaml --trace out/prm_star.jsonl
python tools/viz/replay.py out/prm_star.jsonl --gif out/prm_star.gif
```

## Properties

- **Completeness**: probabilistically complete[^karaman].
- **Optimality**: **asymptotically optimal.** The shrinking radius keeps the expected neighbor count
  at $\Theta(\log n)$, giving probability-1 convergence to the optimal cost $c^*$ in the limit[^karaman].
- **Cost**: because the expected neighbor count is held logarithmic, the edge count is near-linear
  compared with PRM's fixed radius. At the same sample count it produces PRM-like solutions, but now
  with an optimality guarantee.

## Asymptotic Optimality

**Connection radius.** For $n$ samples,

$$
r_n=\gamma_{\mathrm{PRM}^*}\left(\frac{\log n}{n}\right)^{1/d},\qquad
\gamma_{\mathrm{PRM}^*}>2\left(1+\frac1d\right)^{1/d}\left(\frac{\mu(X_{\text{free}})}{\zeta_d}\right)^{1/d}
$$

($d$ = dimension, here $d=2$; $\mu(X_{\text{free}})$ = free-space volume; $\zeta_d$ = volume of the
unit $d$-ball).

**Theorem (asymptotic optimality, Karaman & Frazzoli 2011).** Under this radius the roadmap's
shortest-path cost $Y_n$ satisfies

$$
P\!\left[\lim_{n\to\infty}Y_n=c^*\right]=1.
$$

*Intuition.* Shrinking the radius as $(\log n/n)^{1/d}$ while keeping the coefficient $\gamma$ above
the threshold leaves each node with an expected $\Theta(\log n)$ neighbors — enough connectivity to
recover, in the limit, a path in the same homotopy class as the optimum. Shrink faster and the graph
disconnects, breaking optimality; keep it constant and the edges explode to $O(n^2)$. $r_n$ is the
minimal density between these two failures.

**Edge-count derivation.** At sample density $n/\mu(X_{\text{free}})$, the expected number of neighbors
inside a radius-$r$ ball is $\approx\zeta_d\,r^d\,n/\mu(X_{\text{free}})$. Substituting
$r=r_n=\gamma(\log n/n)^{1/d}$,

$$
\mathbb{E}[\deg]\;\approx\;\frac{\gamma^d\zeta_d}{\mu(X_{\text{free}})}\,\log n\;=\;\Theta(\log n),
\qquad |E|=\Theta(n\log n).
$$

A constant radius gives $\mathbb{E}[\deg]=\Theta(n)$ and $|E|=\Theta(n^2)$ (that is [PRM](prm.md));
fixing the neighbor count at $k=O(1)$ instead falls below the random-geometric-graph connectivity
threshold $\Theta(\log n)$, so the graph fragments and optimality breaks. $r_n$ pins exactly the
**minimal density** that preserves connectivity.

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `num_samples` | int | 1500 | [1, 200000] | Number of collision-free samples placed in the roadmap (start/goal excluded) |
| `gamma` | float | 30.0 | [0.01, 1000.0] | Connection-radius coefficient γ. r_n = γ·(log n / n)^(1/2) |
| `seed` | int | 1 | [0, 2^31−1] | Random seed (reproducibility) |

## Emitted Trace Events

`planning_started` → `sample_drawn`\* → `edge_added`\* → `node_expanded`\* → `path_found` → `planning_finished`

The event set is identical to [PRM](prm.md) — the difference is only the radius policy driving
`edge_added`.

## References

[^kavraki]: Kavraki, L. E., Švestka, P., Latombe, J.-C., & Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580. [doi:10.1109/70.508439](https://doi.org/10.1109/70.508439)
[^karaman]: Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846–894. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
