---
title: JPS
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 7
---

[🇰🇷 한국어](jps.md) | [🇬🇧 English](../../en/algorithms/jps.md)

# JPS (Jump Point Search)
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | informed graph search (grid 대칭 제거) |
| 요구 capability | `DiscreteSpace` + `DynamicGridSpace` (`is_blocked` 를 jump oracle 로) |
| 완전성 | complete (유한 8-connected grid) |
| 최적성 | **cost 최적** — 8-connected A* 와 동일한 경로 |
| 복잡도 | 최악은 A* 와 동일, 단 jump point 만 확장 |
| 원 논문 | Harabor & Grastien (2011) [^harabor] |

1. TOC
{:toc}

## 배경

**uniform-cost 8-connected grid** 에서 A* 는 대부분의 일을 *대칭(symmetry)* 에 낭비한다. 두 열린 셀 사이에는
같은 길이의 grid 경로가 무수히 많고, 이들은 서로 교환 가능한데도 A* 는 전부 탐색한다. Jump Point
Search[^harabor] 는 결과를 바꾸지 않으면서 이 중복을 제거한다. heuristic·최적 경로·`g`/`f` 기계는 그대로 둔 채
"모든 이웃을 확장" 을 "**의미 있는 다음 셀로 점프**" 로 바꾼 A* 의 엄밀한 최적화다.

한 노드에서 JPS 는 고정 방향을 훑으며 "그냥 같은 방향으로 계속" 인 셀을 전부 건너뛴다. 멈추는 곳은
**jump point** 뿐이다 — goal, 또는 *forced neighbour* 를 가진 셀. forced neighbour 란 인접한 장애물 때문에
**여기서 꺾어야만** 최적으로 도달할 수 있게 되는 이웃이다. 그 사이의 셀은 결정 지점이 아님이 증명되므로
open list 에 절대 들어가지 않는다. 결과적으로 8-connected A* 와 같은 경로를, 극히 일부 노드만 확장해 얻는다.

## 동작 원리

`maze01` 에서의 탐색. 셀 파면이 자라는 대신, 확장되는 것은 몇 개의 jump point(꺾이는 셀)뿐이다 — 그 사이의
긴 직선/대각 구간은 통째로 건너뛴다.

![JPS on maze01](../../assets/jps/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/jps/maze01_snap_02.png) | ![mid](../../assets/jps/maze01_snap_05.png) | ![final](../../assets/jps/maze01_final.png) |

`open01` 최종 결과 — 장애물이 없으면 forced neighbour 도 전혀 없어 JPS 는 start 에서 벽과 goal 로 곧장 점프한다:

![JPS on open01](../../assets/jps/open01_final.png)

```
JPS(start, goal):
    g[start] ← 0; open ← f = g + h 로 정렬된 priority queue          # h = octile
    while open 이 비지 않음:
        u ← open.pop_min()
        if u == goal: return reconstruct(u)
        for d in pruned_directions(u, parent[u]):                   # natural + forced
            p ← jump(u, d)                                          # 다음 jump point 로 건너뜀
            if p is null: continue
            if g[u] + octile(u, p) < g[p]:                          # A* relaxation
                g[p] ← g[u] + octile(u, p); parent[p] ← u
                open.push(p, g[p] + octile(p, goal))
    return failure

jump(x, d):
    n ← step(x, d)                          # 한 칸 이동; 막힘/코너컷이면 null
    if n is null: return null
    if n == goal: return n
    if has_forced_neighbour(n, d): return n
    if d 가 대각:
        if jump(n, d.horizontal) or jump(n, d.vertical): return n   # 성분이 점프하면 n 이 중요
    return jump(n, d)                        # 같은 방향으로 계속 훑음
```

A* 와 다른 것은 successor 생성기 한 곳뿐이다 — A* 는 ≤ 8 개 이웃을, JPS 는 jump point 를 push 한다. `g`/`f`/open
로직은 동일하므로 반환 경로와 비용이 정확히 같다.

### 가지치기와 점프 — 코너컷 금지

규칙은 grid 의 대각 정책에 좌우된다. 이 저장소의 grid 는 **코너컷을 금지**한다: 대각 한 스텝은 공유하는 두
직교 셀이 모두 free 여야 한다(`OccupancyGrid2D.neighbors` 참조). 따라서 forced-neighbour 규칙은 Harabor &
Grastien (2011) 의 *no-corner-cutting* 변형을 쓴다:

- **직선 구간** (예: `x` 에서 동쪽 이동): natural successor 는 "동쪽" 하나뿐이다. 진행 방향의 **대각 뒤쪽**에
  장애물이 있고 그 옆 셀이 열려 있으면 **forced neighbour** 가 생긴다 — 예를 들어 동쪽 이동 중 `(r-1, c-1)` 은
  막혔지만 `(r-1, c)` 는 free 인 경우. 그 옆 셀(과 그 너머 대각)은 `x` 에서 꺾어야만 최적 도달이 되므로 `x`
  가 jump point 가 된다.
- **대각 구간**: 대각 점프는 먼저 두 **직교 성분**을 훑는다. 둘 중 하나라도 jump point 를 찾으면 현재 대각
  셀 자체가 jump point 다. 코너컷이 불가하므로 대각은 직교 스캔이 이미 커버하지 못하는 옆 셀을 새로 열지 않아,
  별도의 forced-neighbour 규칙이 필요 없다.
- **점프 비용.** jump point 는 항상 *순수* 직선 또는 대각 직선으로 도달하므로, `x` 에서 jump point `p` 까지의
  실제 이동 비용은 정확히 **octile distance** 다.

$$
\text{octile}(x,p)=\bigl(\max(\Delta r,\Delta c)-\min(\Delta r,\Delta c)\bigr)+\sqrt2\cdot\min(\Delta r,\Delta c),
$$

  이는 8-connected A* 가 한 스텝씩 누적했을 값과 같다. 그래서 JPS 의 `g` 값, 나아가 반환 비용이 A* 와 비트
  단위로 일치한다.

점프에 필요한 "이 셀이 막혔거나 범위 밖인가?" 단일 셀 질의가 바로 `DynamicGridSpace.is_blocked` 이므로, JPS 는
그 capability 를 oracle 로 재사용하고 자체 capability 를 추가하지 않는다.

측정치 (Python, trace on):

| map | path cost | expanded nodes | 참고: A\* expanded |
|---|---|---|---|
| maze01 | 28.728 (최적, A\* 와 동일) | **8** | 108 |
| open01 | 25.213 (최적, A\* 와 동일) | **8** | 71 |

A* 와 같은 최적 비용을, 확장 노드는 한 자릿수로.

재현:

```bash
python python/demos/demo_jps.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/jps.yaml --trace out/jps.jsonl
python tools/viz/replay.py out/jps.jsonl --gif out/jps.gif --snapshots out/jps_snaps/
```

## 성질

- **완전성**: 유한 8-connected grid 에서 완전 — 점프 규칙은 필요한 꺾임 지점을 절대 건너뛰지 않는다.
- **최적성**: 정확히 8-connected A* 의 최적해를 반환한다. JPS 는 A* 의 무손실 successor 가지치기이지 근사가 아니다.
- **복잡도**: 최악은 A* 와 동일(한 칸 폭 복도로 된 미로는 모든 셀을 확장하는 것으로 퇴화)하지만, 열린 지형에서는
  긴 구간을 open list 를 건드리지 않는 O(구간 길이) 스캔으로 건너뛰어 확장 수가 급감한다.

## 가지치기가 최적성을 보존하는 이유

JPS 가 유지하는 불변식은 **모든 최적 경로에 대해, jump point 에서만 꺾는 최적 경로가 존재한다** 는 것이다.
forced neighbour 가 없는 셀을 지나는 직선/대각 구간에는, 그 셀에 멈추지 않고 같은 successor 에 같은 비용으로
도달하는 *대칭* 대안이 있으므로 그 셀은 안전하게 건너뛸 수 있다. forced neighbour 는 바로 그 대칭 대안이
장애물로 막힌 상황이라, 그런 셀(과 goal)만 결정 지점으로 남긴다. (i) 남긴 모든 successor 는 코너컷 없는 합법
직선으로 도달하며 그 비용이 octile distance 와 같고, (ii) 최적 꺾임 지점을 절대 건너뛰지 않으므로, jump point
위에서 돌린 A* 는 여전히 최적해를 포함한 부분그래프를 탐색한다. 따라서 JPS 는 8-connected A* 와 동일한 비용
$C^\ast$ 의 경로를 반환한다.

## 파라미터

JPS 는 **튜닝 파라미터가 없다** — uniform-cost 8-connected grid 에서 가지치기는 장애물만으로 완전히 결정되고,
weight 1 의 octile heuristic 이 이미 최적이다. `configs/global_planning/jps.yaml` 는 빈 파라미터 집합을 선언한다.

## 방출 trace 이벤트

`planning_started` → (`node_expanded`, `edge_added`)* → `path_found` → `planning_finished`

`node_expanded` 는 확장된 **jump point** 마다 한 번, `edge_added` 는 부모 jump point 와 각 jump-point successor 를
잇는다(비용 = 점프의 octile distance). 그래서 재생 화면은 조밀한 파면이 아니라 성긴 jump-point 그래프를 보여준다.
복원된 `path` 는 A* 의 경로 보고 방식과 맞추어 전체 계단형 grid 셀로 보간된다.

## References

[^harabor]: Harabor, D., & Grastien, A. (2011). "Online Graph Pruning for Pathfinding on Grid Maps." *Proc. AAAI Conference on Artificial Intelligence*, 1114–1119. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/7994)
