---
title: Theta*
layout: default
parent: Algorithms
grand_parent: English
nav_order: 4
---

[🇰🇷 한국어](../../ko/algorithms/theta_star.md) | [🇬🇧 English](theta_star.md)

# Theta* (any-angle)
{: .no_toc }

| Item | Description |
|---|---|
| Category | any-angle graph search |
| Required capability | `LineOfSightSpace` (`neighbors` + `heuristic` + `line_of_sight`) |
| Completeness | complete (finite graphs, non-negative costs) |
| Optimality | any-angle path — **grid-optimality is not guaranteed** (basic Theta\*, Nash et al. 2007) |
| Complexity | A\* level + one line-of-sight check per relaxation |
| Original paper | Nash, Daniel, Koenig & Felner (2007) [^nash] · LOS: Amanatides & Woo (1987) [^aw] · weighted: Pohl (1970) [^pohl] |

1. TOC
{:toc}

## Background

A\*[^hart] on a grid lays its path only on grid edges (orthogonal and diagonal), so a stretch that
could be crossed by a straight line still becomes a staircase that bends in 45°/90° steps.
**Theta\***[^nash] adds a single rule to A\*: when relaxing a node, it checks whether a candidate's
parent can be **the parent of the current node** rather than the current node itself — that is,
whether there is an obstacle-free **line of sight** between the two. If there is, the path leaves the
grid and takes an arbitrary-angle (any-angle) straight-line shortcut.

The resulting path has far sparser waypoints (it bends only at obstacle corners) and is shorter than
the grid path. In this repository's demo, `maze01` shrinks from A\* 26 waypoints · cost 28.73 to
Theta\* **4 waypoints · cost 27.75**.

## How It Works

Search on `maze01`. Like the A\* frontier it grows toward the goal, but the final path is not a grid
staircase — it is a **sparse straight-line polyline** that only grazes obstacle corners.

![Theta* on maze01](../../assets/theta_star/maze01.gif)

Intermediate search progress (left → right: early / middle / final path):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/theta_star/maze01_snap_02.png) | ![mid](../../assets/theta_star/maze01_snap_05.png) | ![final](../../assets/theta_star/maze01_final.png) |

Final result on `open01` — with few obstacles, start→goal connects as almost a single straight line:

![Theta* on open01](../../assets/theta_star/open01_final.png)

```
THETASTAR(start, goal):
    g[start] ← 0; parent[start] ← start
    open ← priority queue keyed by f = g + w·h        # h = Euclidean straight-line distance
    while open is not empty:
        s ← open.pop_min()
        if s == goal: return reconstruct(parent, s)
        if s already settled: continue                # lazy deletion
        for (s2, edge_cost) in neighbors(s):
            p ← parent[s]
            if line_of_sight(p, s2):                   # Path 2 — any-angle shortcut
                cand ← g[p] + euclid(p, s2);  par ← p
            else:                                      # Path 1 — standard grid step
                cand ← g[s] + edge_cost;      par ← s
            if cand < g[s2]:                           # relaxation
                g[s2] ← cand; parent[s2] ← par
                open.push(s2, cand + w·h(s2, goal))
    return failure
```

The difference from A\* is the single inner `if line_of_sight(...)` **block**. If the LOS check always
fails, it degenerates into exactly weighted A\*. Because parent links are only ever set on pairs whose
LOS has been confirmed (Path 2 verifies visibility, Path 1 is between adjacent cells and therefore
trivial), **every edge of the reconstructed path is an actually traversable straight line**.

### Line of Sight — one collision model with the grid

`line_of_sight(a, b)` decides whether the segment joining two cell centres is actually traversable,
using the **same corner-cut-forbidden rule** as `neighbors()`. The original paper used Bresenham, but
Bresenham visits only one cell per axis and can wrongly report a corner-grazing segment as "visible."
This repository reuses the **supercover**[^aw], which visits every cell the segment touches (delegating
to the map's `is_motion_valid`). Therefore "a pair visible by LOS" ⇔ "a legal straight move," and
Theta\* and grid A\* share **one collision model**.

### Heuristic — Euclidean (not octile)

Theta\*'s g-values are straight-line (Euclidean) distances, so the heuristic is also **Euclidean**:

```
h(a, b) = √((Δrow)² + (Δcol)²)
```

The octile heuristic the map provides for A\* satisfies octile ≥ Euclidean, so it is **inadmissible**
(overestimating) for any-angle costs and is not used. The Euclidean distance is an admissible lower
bound for arbitrary-angle motion.

Measurements (Python, w = 1.0, trace on · A\* comparison on the same instance):

| map | Theta\* cost | A\* cost | Theta\* expanded | A\* expanded | Theta\* waypoints |
|---|---|---|---|---|---|
| maze01 | **27.748** | 28.728 | 104 | 108 | 4 (A\*: 26) |
| open01 | **24.241** | 25.213 | 66 | 71 | 3 (A\*: 20) |

Reproduce:

```bash
python python/demos/demo_theta_star.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/theta_star.yaml --trace out/theta_star.jsonl
python tools/viz/replay.py out/theta_star.jsonl --gif out/theta_star.gif --snapshots out/theta_snaps/
```

## Properties

- **Completeness**: complete on a finite grid with non-negative costs (same as A\*).
- **Optimality**: basic Theta\* guarantees **neither grid-optimality nor true-optimality**[^nash].
  Because the local rule only ever inherits the grandparent as parent, it can miss the truly shortest
  any-angle path (though it stays very close). When strict optimality is required, use later variants
  such as Lazy Theta\* / AP Theta\*.
- **Quality**: the returned path cost is always ≤ the A\* cost on the same grid — LOS shortcuts replace
  grid staircases.
- **Complexity**: the same search as A\* plus one LOS check (linear in segment length) per relaxation.
- w > 1 (weighted Theta\*, Pohl 1970[^pohl]): inflating the heuristic reduces expanded nodes at the
  cost of path quality.

## Any-angle Cost Model and Optimality

**Notation.** Treat grid cell centres as points in the plane $\mathbb{R}^2$. An any-angle path
$P=(v_0,v_1,\dots,v_k)$ is a polyline through cell centres in which every segment
$\overline{v_i v_{i+1}}$ must be obstacle-free (LOS). Its cost is the sum of Euclidean lengths:

$$
\operatorname{cost}(P)=\sum_{i=0}^{k-1}\lVert v_{i+1}-v_i\rVert_2 .
$$

Let $C^\ast$ be the minimum cost over feasible polylines, $h(n)=\lVert n-\text{goal}\rVert_2$, and
$h^\ast(n)$ the true shortest feasible cost from $n$ to the goal.

**Lemma 1 (the Euclidean heuristic is admissible and consistent).**
Let a feasible path $P=(v_0,\dots,v_k)$ have endpoints $a=v_0,\ b=v_k$. Applying the triangle
inequality segment by segment and telescoping,

$$
\lVert b-a\rVert_2=\Bigl\lVert\textstyle\sum_{i}(v_{i+1}-v_i)\Bigr\rVert_2
   \;\le\;\sum_{i}\lVert v_{i+1}-v_i\rVert_2=\operatorname{cost}(P),
$$

so no feasible path is shorter than the straight-line distance between its endpoints. Applying this
lower bound with $a=n,\ b=\text{goal}$,

$$
h(n)=\lVert n-\text{goal}\rVert_2\;\le\;\operatorname{cost}(P)\ \text{ for all feasible }P
\;\Longrightarrow\; h(n)\le h^\ast(n)\qquad(\text{admissible}).
$$

And for any pair $(a,b)$ whose move cost is the Euclidean length $c(a,b)=\lVert a-b\rVert_2$, the
triangle inequality

$$
h(a)=\lVert a-\text{goal}\rVert_2\;\le\;\lVert a-b\rVert_2+\lVert b-\text{goal}\rVert_2
      \;=\;c(a,b)+h(b)
$$

gives consistency. As with A\*, consistency $\Rightarrow$ $f$ is non-decreasing along any path
$\Rightarrow$ each node is expanded at most once. ∎

**Why not octile (quantified).** The octile heuristic A\* uses is, for $\Delta r\ge\Delta c$,
$\text{oct}=\Delta r+(\sqrt2-1)\,\Delta c$. Its ratio to the any-angle true value
$\lVert\cdot\rVert_2=\sqrt{\Delta r^2+\Delta c^2}$ is $\rho(t)=\dfrac{1+(\sqrt2-1)t}{\sqrt{1+t^2}}$
with $t=\Delta c/\Delta r\in[0,1]$. Differentiating, the maximum is at $t^\ast=\sqrt2-1$:

$$
\max_t\ \frac{\text{oct}}{\lVert\cdot\rVert_2}=\sqrt{4-2\sqrt2}\approx 1.0824 .
$$

So octile overestimates any-angle cost by up to 8.24 % (inadmissible), breaking the optimality
argument. Euclidean is the admissible lower bound for arbitrary-angle motion guaranteed above. ∎

**Lemma 2 (every $g$-value is the length of a real feasible path).**
Whenever a node $s$ is settled, its $\text{parent}[s]=p$ is a node with LOS$(p,s)$ and

$$
g[s]=g[p]+\lVert p-s\rVert_2
$$

(Path 1 is between adjacent cells, so LOS is trivial; Path 2 checks it explicitly). Unrolling the
parent chain back to start makes $g[s]$ **exactly the length of the obstacle-free polyline**
$\text{start}\to\cdots\to p\to s$. In particular the returned $g[\text{goal}]$ is always an
achievable upper bound — the path Theta\* emits is always feasible. ∎

**Proposition 3 (no longer than grid A\*).** The path $P_\Theta$ returned by Theta\* is no longer than
the grid path $P_A$ that A\* returns on the same grid:
$\operatorname{cost}(P_\Theta)\le\operatorname{cost}(P_A)$.[^nash]

*Proof sketch.* If every LOS check fails, Path 2 never fires and the algorithm is identical to
weighted A\*, so $P_\Theta=P_A$. At each point where LOS succeeds, Path 2 replaces the two grid legs
$p\to s\to s_2$ with the grandparent–grandchild straight line $p\to s_2$, and the triangle inequality

$$
\lVert p-s_2\rVert_2\;\le\;\lVert p-s\rVert_2+\lVert s-s_2\rVert_2
$$

holds while relaxation only adopts the replacement when it lowers $g$. Hence no shortcut ever increases
length: $P_\Theta$ is the grid path with its line-of-sight stretches straightened, and its cost is
monotonically non-increasing. ∎

**Proposition 4 (not strictly optimal).** Basic Theta\* guarantees neither $C^\ast$ nor the shortest
any-angle path on the grid.[^nash]

*Structural reasoning.* With polygonal obstacles the shortest any-angle path is a taut string whose
turning points lie **only at convex obstacle corners** (straight between them). To recover it, a
relaxation must be able to name "the corner the string is currently wrapped around" as the parent. But
Theta\* chooses the parent of a candidate $s_2$ only from the two values

$$
\text{parent}(s_2)\in\{\,s,\ \text{parent}(s)\,\}.
$$

If the corner $O$ the optimal string must wrap is absent from this 2-element set at the moment of
expansion — e.g. $O$ is an ancestor **two or more generations** before $\text{parent}(s)$, or the
string reaching $O$ came from a different expansion branch — then the straight shortcut $O\to s_2$ is
never created, and Theta\* approximates it by a slightly longer path that steps around $O$ along the
grid from $\text{parent}(s)$.

*Why this is not a heuristic issue.* By Lemma 1 $h$ is admissible, so at $w=1$ the A\* skeleton itself
never misses the optimum; the gap comes solely from the **locality of the parent-candidate set
$\{s,\text{parent}(s)\}$** — the relaxation that defines path cost is locally myopic and cannot pull
the string fully taut. Empirically it stays within about 1 % of $C^\ast$ (this repo's maze01:
Theta\* 27.75 vs A\* grid 28.73). For strict optimality, use later variants that consider **every**
visible ancestor as a candidate (Lazy Theta\*, AP Theta\*). ∎

## Parameters

| Name | Type | Default | Range | Description |
|---|---|---|---|---|
| `heuristic_weight` | float | 1.0 | [1.0, 5.0] | The w in f = g + w·h (h is Euclidean). 1.0 = standard Theta\*, above = weighted |

## Emitted Trace Events

`planning_started` → (`node_expanded`, `candidate_evaluated`, `edge_added`)* → `path_found` → `planning_finished`

In `edge_added(state=s2, parent)`, `parent` becomes a **non-adjacent grandparent** on Path 2 (the
schema and the visualizer impose no adjacency constraint). `replay.py` draws the parent→state straight
line as-is to render the any-angle shortcut, so no new trace event is required.

## References

[^nash]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^aw]: Amanatides, J., & Woo, A. (1987). "A Fast Voxel Traversal Algorithm for Ray Tracing." *Proc. Eurographics*, 3–10. [PDF](https://www.cse.yorku.ca/~amana/research/grid.pdf)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
[^pohl]: Pohl, I. (1970). "Heuristic search viewed as path finding in a graph." *Artificial Intelligence*, 1(3–4), 193–204. [doi:10.1016/0004-3702(70)90007-X](https://doi.org/10.1016/0004-3702%2870%2990007-X)
</content>
</invoke>
