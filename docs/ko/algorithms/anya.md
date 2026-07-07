---
title: Anya
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 7
---

[🇰🇷 한국어](anya.md) | [🇬🇧 English](../../en/algorithms/anya.md)

# Anya (optimal any-angle)
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | optimal any-angle graph search |
| 요구 capability | `LineOfSightSpace` (`neighbors` + `heuristic` + `line_of_sight`) |
| 완전성 | complete (유한 그래프, 비음수 비용) |
| 최적성 | **optimal** any-angle — 정점 모델 위 유클리드 최단 경로 (Harabor et al. 2016) |
| 복잡도 | best-first 탐색; 매 확장마다 root 의 가시영역을 행별 interval 로 투영 |
| 원 논문 | Harabor, Grastien, Öz & Aksakalli (2016) [^harabor] · LOS: Amanatides & Woo (1987) [^aw] · weighted: Pohl (1970) [^pohl] |

1. TOC
{:toc}

## 배경

**Theta\***[^nash] 는 A\* 를 *any-angle* 로 만든다 — 경로가 grid 를 벗어나 직선 지름길을 취한다 — 하지만
**최적이 아니다**: 노드의 지름길 부모가 오직 그 조부모뿐이라, 실을 완전히 팽팽하게 당기지 못해 경로가
진짜 최단 any-angle 경로보다 조금 더 길게 남는다.

**Anya**[^harabor] 는 그 간극을 메운다. 전처리 없이 온라인으로 탐색하면서도 **증명 가능한 유클리드 최단
any-angle 경로**를 반환하는 첫 알고리즘이다. 핵심은 grid 셀 하나씩이 아니라 **interval(구간)** 단위로
추론하는 것이다. 탐색 노드는 `(root, interval)` 쌍이며, interval 은 한 grid 행 위에서 root 로부터 직선으로
모두 보이는 연속 점들의 집합이다. 노드를 확장할 때 그 interval 을 grid 를 통해 **투영(project)** 한다 —
root 에서 뻗어나가는 빛의 원뿔처럼 — 이웃 행에서 볼 수 있는 후계 interval 들을 생성한다. interval 전체를
한 스텝에 처리하므로 셀 단위 탐색보다 훨씬 적은 노드를 방문하면서도, 최단 경로가 감쌀 수 있는 모든
회전점(장애물 모서리)을 빠짐없이 고려한다.

이 저장소 demo 에서 `open01` 은 Theta\* cost 24.241 → Anya **24.208** 로 줄어든다 — Theta\* 가 조금
남겨둔 여분을 Anya 가 없앤 진짜 최단 경로다. `maze01` 은 Theta\* 가 이미 최적(27.748)이라 Anya 가 같은
비용을 내되 더 적은 노드를 확장한다(95 vs 104).

## 동작 원리

`maze01` 에서의 탐색. A\* 처럼 파면이 goal 로 자라지만, 매 확장이 root 의 **가시 interval** 을 밖으로
펼치고, 최종 경로는 장애물 모서리만 스치는 **성근 직선 다각선** 중 최단이다.

![Anya on maze01](../../assets/anya/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/anya/maze01_snap_02.png) | ![mid](../../assets/anya/maze01_snap_05.png) | ![final](../../assets/anya/maze01_final.png) |

`open01` 최종 결과 — 장애물이 적으면 start→goal 가 단일 직선으로 연결되고, Anya 는 더 짧은 경로가 없음을
증명한다:

![Anya on open01](../../assets/anya/open01_final.png)

### interval 과 그 cone 투영

Anya 의 노드는 `(root r, interval I)` 다. `I` 는 `r` 로부터 보이는 같은 행의 최대 연속 구간이며, 그
끝점은 장애물 변 또는 cone 이 스치는 모서리에 의해 고정된다. `(r, I)` 를 확장하면 두 종류의 후계가
생긴다 (Harabor et al. 2016):

- **Observable 후계** — 인접 행에서 `r`→`I` cone 안에 들어오는 부분. root 는 여전히 `r` 이다: 실이 아직
  꺾이지 않았으므로 지금까지의 경로는 `r` 로부터의 직선 그대로다.
- **Non-observable 후계** — `I` 의 끝점이 장애물 **모서리**에 놓일 때 생성된다. 여기서 `r` 로부터의 직선이
  막혀 모서리에서 **꺾여야** 하므로, 그 모서리가 후계 interval 의 **새 root** 가 되고 `g` 값이 직선거리
  `‖r − corner‖` 만큼 늘어난다.

회전은 오직 모서리에서만 일어나며, 이는 다각형 세계에서 최단 경로가 갖는 taut-string(팽팽한 실) 성질
그대로다 — 여기서 Anya 의 최적성이 나온다.

### interval 휴리스틱 — goal 을 행에 대해 반사

큐를 admissible 하게 정렬하려면 노드의 `f = g(r) + h(I)` 에서, 경로가 goal 로 가며 interval 을 가장 싸게
통과하는 값 `h(I) = min_{p ∈ I} ‖r − p‖ + ‖p − goal‖` 이 필요하다. `r` 과 `goal` 이 interval 행의 같은
쪽에 있으면 이 최솟값은 **goal 을 그 행에 대해 반사**해 구한다: 꺾인 경로 `r → p → goal` 이 단일 선분
`r → goal'` 로 펴지고, `h(I)` 는 그 선분이 `I` 를 가로지르면 `‖r − goal'‖`, 아니면 가까운 끝점에서의
값이다. 반사는 `h` 를 admissible·consistent 한 하한으로 유지하므로, Anya 가 goal 에 처음 도달한 경로가
곧 최적이다.

### 이 저장소에서의 적응

이 저장소는 상태를 셀 중심(`Cell = (row, col)`)으로, 경로를 `list[Cell]` 로 저장한다 — Theta\* 와
동일하다. 따라서 회전점은 셀 중심이고 반환되는 모든 leg 은 LOS-clear 직선 구간이다. Anya 는 그 정점
모델 위에서 표현된다: **root 는 셀 중심**이고, 한 행에서 root 의 후계 interval 은 root 로부터 모두
LOS-visible 인 행-인접 free 셀들의 최대 연속 구간이다. root 는 Theta\* 와 같은 유클리드 `h` 및
`(f, 삽입순서)` tie-break 로 `f = g + w·h` best-first 순서로 settle 되고, 각 가시 interval 의 모든 셀이
relax 된다. 가시 집합 전체를 relax 하므로 탐색은 **셀-중심 visibility graph** 위 최단 경로 — 유클리드
any-angle 최적 — 로 수렴하며, 따라서 같은 인스턴스에서 비용은 항상 Theta\* 이하다. 후보 정점 집합은
`neighbors()` capability 만으로 발견한 start 의 연결 free 성분이다(planner 는 구체 맵 클래스를 건드리지
않는다).

```
ANYA(start, goal):
    g[start] ← 0; parent[start] ← start
    open ← priority queue keyed by f = g + w·h        # h = 유클리드 직선거리
    reachable ← start 에 연결된 free 셀 (neighbors() 로 발견)
    while open is not empty:
        r ← open.pop_min()
        if r == goal: return reconstruct(parent, r)
        if r already settled: continue                # lazy deletion
        for each grid row y in reachable:             # root 의 가시영역을 행별로 투영
            for each maximal interval [a, b] of cells on y visible from r:   # cone 후계
                for cell in a..b:
                    cand ← g[r] + euclid(r, cell)      # root 로부터의 직선 leg
                    if cand < g[cell]:                 # relaxation
                        g[cell] ← cand; parent[cell] ← r
                        open.push(cell, cand + w·h(cell, goal))
    return failure
```

### Line of sight — grid 와 하나의 충돌 모델

`line_of_sight(a, b)` 는 두 셀 중심을 잇는 선분이 통과 가능한지를, `neighbors()` 와 **동일한
corner-cut 금지 supercover**[^aw] 규칙으로 판정한다(맵의 `is_motion_valid` 에 위임). 따라서 "LOS 로 보임"
⇔ "합법적 직선 이동"이며, Anya·Theta\*·grid A\* 가 **하나의 충돌 모델**을 공유한다.

## 최적성과 비용 모델

**표기.** 셀 중심을 $\mathbb{R}^2$ 의 점으로 본다. any-angle 경로 $P=(v_0,\dots,v_k)$ 는 셀 중심을 잇는
다각선으로 모든 구간 $\overline{v_i v_{i+1}}$ 이 장애물 없음(LOS)이며, 비용은
$\operatorname{cost}(P)=\sum_i\lVert v_{i+1}-v_i\rVert_2$ 다. $C^\ast$ 를 그 최소 비용,
$h(n)=\lVert n-\text{goal}\rVert_2$, $h^\ast(n)$ 을 $n$ 에서 goal 까지의 실제 최단 비용이라 하자.

**Lemma 1 (유클리드 $h$ 는 admissible·consistent).** 삼각부등식을 구간별로 적용하면 임의의 feasible
$P$ 에 대해 $\lVert b-a\rVert_2\le\operatorname{cost}(P)$; $a=n,\ b=\text{goal}$ 로 두면
$h(n)\le h^\ast(n)$ (admissible). 비용 $\lVert a-b\rVert_2$ 인 임의의 LOS 간선 $(a,b)$ 에 대해
$h(a)\le\lVert a-b\rVert_2+h(b)=c(a,b)+h(b)$ (consistent). 따라서 $w=1$ 에서 best-first 탐색은 노드의
최적 $g$ 가 확정되기 전에 그 노드를 확장하지 않는다. ∎

**Lemma 2 (모든 $g$ 는 실제 feasible 길이).** relaxation 은 `line_of_sight(r, cell)` 일 때만
$g[\text{cell}]=g[r]+\lVert r-\text{cell}\rVert_2$ 로 설정하므로, `parent` 를 펼치면 $g[\text{cell}]$ 은
장애물 없는 다각선 $\text{start}\to\cdots\to r\to\text{cell}$ 의 길이 그 자체다. 반환된 $g[\text{goal}]$
은 달성 가능한 상한이다 — Anya 의 경로는 항상 feasible 하다. ∎

**Proposition 3 (Anya 는 정점 모델 위 최적).** settle 된 모든 root 는 자신으로부터 LOS-visible 한
**모든** free 셀로 뻗으므로, 탐색은 정확히 *셀-중심 visibility graph* $G=(V,E)$ 위의 A\* 다: $V$ = reachable
free 셀, $E$ = 상호 가시 쌍, weight $=\lVert\cdot\rVert_2$. Lemma 1 의 consistent 휴리스틱으로 A\* 는 $G$
의 최단 경로를 반환하고, Lemma 2 로 그 값은 달성 가능하다. 따라서 셀-중심 다각선 위에서
$\operatorname{cost}(P_{\text{Anya}})=C^\ast$ — faithful Anya 가 최적화하는 바로 그 양이며, 그 모서리
회전점은 가시성이 끊기는 interval 끝점으로 회복된다 (Harabor et al. 2016). ∎

**Proposition 4 (Theta\* 보다 길지 않음).** Theta\* 의 출력은 하나의 feasible 셀-중심 다각선이므로
$C^\ast\le\operatorname{cost}(P_\Theta)$; 따라서 모든 인스턴스에서
$\operatorname{cost}(P_{\text{Anya}})\le\operatorname{cost}(P_\Theta)$. Theta\* 의 근시안적 조부모 규칙이
실을 조금 느슨하게 남기는 곳에서 Anya 는 완전히 팽팽하게 당긴다 (이 저장소 `open01`: Anya 24.208 vs
Theta\* 24.241). ∎

측정 (Python, w = 1.0, trace on · 같은 인스턴스에서 Theta\* / A\*):

| map | Anya cost | Theta\* cost | A\* cost | Anya expanded | Theta\* expanded | Anya waypoints |
|---|---|---|---|---|---|---|
| maze01 | **27.748** | 27.748 | 28.728 | 95 | 104 | 4 |
| open01 | **24.208** | 24.241 | 25.213 | 38 | 66 | 3 |

재현:

```bash
python python/demos/demo_anya.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/anya.yaml --trace out/anya.jsonl
python tools/viz/replay.py out/anya.jsonl --gif out/anya.gif --snapshots out/anya_snaps/
```

## 성질

- **완전성**: 유한 grid + 비음수 비용에서 complete (A\* 와 동일).
- **최적성**: `w = 1` 에서 **최적** — 반환 경로가 정점 모델 위 유클리드 최단 any-angle 경로다. any-angle
  이지만 최적이 아닌 Theta\* 와 다르다[^harabor].
- **Theta\* 대비 품질**: 같은 grid 에서 비용이 항상 Theta\* 이하 (Proposition 4).
- **가중치**: `w > 1` (weighted, Pohl 1970[^pohl]) 은 휴리스틱을 부풀려 더 적은 노드를 확장하지만 최적성
  보장을 포기한다 — bounded-suboptimal any-angle.

## 파라미터

| 이름 | 타입 | 기본 | 범위 | 설명 |
|---|---|---|---|---|
| `heuristic_weight` | float | 1.0 | [1.0, 5.0] | f = g + w·h 의 w (h 는 유클리드). 1.0 = 최적 Anya; 1.0 초과 = weighted (더 빠르나 최적성 포기) |

## 방출 Trace 이벤트

`planning_started` → (`node_expanded`, `candidate_evaluated`, `edge_added`)* → `path_found` → `planning_finished`

`node_expanded(state=r)` 는 settle 된 root 마다 한 번 방출된다. 투영된 interval 안의 각 relaxation 은
`candidate_evaluated` 와 `edge_added(state=cell, parent=r)` 를 방출하며, `parent` 는 (비인접일 수 있는)
interval root 다 — 시각화기는 parent→state 직선을 그대로 그려 any-angle leg 을 렌더하므로 새 trace
이벤트가 필요없다(root 에서 뻗는 edge 다발이 곧 투영된 가시 interval 을 보여준다).

## 참고 문헌

[^harabor]: Harabor, D., Grastien, A., Öz, D., & Aksakalli, V. (2016). "Optimal Any-Angle Pathfinding In Practice." *Journal of Artificial Intelligence Research (JAIR)*, 56, 89–118. [doi:10.1613/jair.5007](https://doi.org/10.1613/jair.5007)
[^nash]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^aw]: Amanatides, J., & Woo, A. (1987). "A Fast Voxel Traversal Algorithm for Ray Tracing." *Proc. Eurographics*, 3–10. [PDF](https://www.cse.yorku.ca/~amana/research/grid.pdf)
[^pohl]: Pohl, I. (1970). "Heuristic search viewed as path finding in a graph." *Artificial Intelligence*, 1(3–4), 193–204. [doi:10.1016/0004-3702(70)90007-X](https://doi.org/10.1016/0004-3702%2870%2990007-X)
