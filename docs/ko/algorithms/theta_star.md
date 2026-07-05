---
title: Theta*
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 4
---

[🇰🇷 한국어](theta_star.md) | [🇬🇧 English](../../en/algorithms/theta_star.md)

# Theta* (any-angle)
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | any-angle graph search |
| 요구 capability | `LineOfSightSpace` (`neighbors` + `heuristic` + `line_of_sight`) |
| 완전성 | complete (유한 그래프, 비음수 비용) |
| 최적성 | any-angle 경로 — **grid-optimal 은 보장하지 않음** (basic Theta*, Nash et al. 2007) |
| 복잡도 | A* 수준 + relaxation 마다 line-of-sight 검사 1회 |
| 원 논문 | Nash, Daniel, Koenig & Felner (2007) [^nash] · LOS: Amanatides & Woo (1987) [^aw] · weighted: Pohl (1970) [^pohl] |

1. TOC
{:toc}

## 배경

격자 위 A*[^hart] 는 경로를 grid 간선(직교·대각)에만 얹으므로, 실제로는 직선으로 갈 수 있는
구간도 45°·90° 단위로 꺾인 계단형 경로를 만든다. **Theta\***[^nash] 는 A* 에 단 하나의 규칙을
더한다: 노드를 완화(relax)할 때 후보의 부모를 현재 노드가 아니라 **현재 노드의 부모**로 삼을 수
있는지 — 즉 그 둘 사이에 장애물 없는 **직선 시야(line of sight)** 가 있는지 — 를 검사한다. 있으면
경로가 grid 를 벗어나 임의 각도(any-angle)의 직선 지름길을 취한다.

결과 경로는 waypoint 가 훨씬 성글고(장애물 모서리에서만 꺾임) grid 경로보다 짧다. 이 저장소의
demo 에서 maze01 은 A* 26개 waypoint·cost 28.73 → Theta\* **4개 waypoint·cost 27.75** 로 줄어든다.

## 동작 원리

```
THETASTAR(start, goal):
    g[start] ← 0; parent[start] ← start
    open ← priority queue keyed by f = g + w·h        # h = 유클리드 직선거리
    while open is not empty:
        s ← open.pop_min()
        if s == goal: return reconstruct(parent, s)
        if s already settled: continue                # lazy deletion
        for (s2, edge_cost) in neighbors(s):
            p ← parent[s]
            if line_of_sight(p, s2):                   # Path 2 — any-angle 지름길
                cand ← g[p] + euclid(p, s2);  par ← p
            else:                                      # Path 1 — 표준 grid step
                cand ← g[s] + edge_cost;      par ← s
            if cand < g[s2]:                           # relaxation
                g[s2] ← cand; parent[s2] ← par
                open.push(s2, cand + w·h(s2, goal))
    return failure
```

A* 와의 차이는 안쪽 `if line_of_sight(...)` **한 블록**뿐이다. LOS 검사가 항상 실패하면 정확히
weighted A* 로 퇴화한다. 부모 링크는 항상 LOS 가 확인된 쌍에만 설정되므로(Path 2 는 시야 확인,
Path 1 은 인접 셀이라 자명), 재구성된 경로의 **모든 간선은 실제로 통과 가능한 직선**이다.

### Line of sight — grid 충돌 모델과 일치

`line_of_sight(a, b)` 는 두 셀 중심을 잇는 선분이 실제로 지나갈 수 있는지를, `neighbors()` 와
**동일한 corner-cut 금지 규칙**으로 판정한다. 원 논문은 Bresenham 을 썼지만 Bresenham 은 축당
한 셀만 방문해 모서리를 스치는 선분을 "보인다"고 잘못 판정할 수 있다. 이 저장소는 선분이 닿는 모든
셀을 방문하는 **supercover**[^aw] 를 재사용한다(맵의 `is_motion_valid` 위임). 따라서 "LOS 로
보이는 쌍" ⇔ "합법적인 직선 이동" 이 되어, Theta\* 와 grid A* 가 **하나의 충돌 모델**을 공유한다.

### Heuristic — 유클리드 (octile 아님)

Theta\* 의 g 값은 직선(유클리드) 거리이므로 heuristic 도 **유클리드**를 쓴다:

```
h(a, b) = √((Δrow)² + (Δcol)²)
```

맵이 A* 용으로 제공하는 octile heuristic 은 octile ≥ 유클리드 이므로 any-angle 비용에 대해
**inadmissible**(과대평가)이라 사용하지 않는다. 유클리드는 임의 각도 이동에 대한 admissible 하한이다.

## 성질

- **완전성**: 유한 grid + 비음수 비용에서 완전 (A* 와 동일).
- **최적성**: basic Theta\* 는 **grid-optimal 도 true-optimal 도 보장하지 않는다**[^nash]. 부모를
  조부모로만 승계하는 국소 규칙이라, 진짜 최단 any-angle 경로를 놓칠 수 있다(대신 매우 근접). 엄밀
  최적이 필요하면 Lazy Theta\*/AP Theta\* 등 후속 변형을 쓴다.
- **품질**: 반환 경로 비용은 항상 같은 grid 의 A* 이하다 — LOS 지름길이 grid 계단을 대체하기 때문.
- **복잡도**: A* 와 같은 탐색에 relaxation 마다 LOS 검사(선분 길이에 선형) 1회가 더해진다.
- w > 1 (weighted Theta\*, Pohl 1970[^pohl]): heuristic 을 부풀려 확장 노드를 줄이되 경로 품질을 완화.

## 파라미터

| 이름 | 타입 | 기본값 | 범위 | 설명 |
|---|---|---|---|---|
| `heuristic_weight` | float | 1.0 | [1.0, 5.0] | f = g + w·h 의 w (h 는 유클리드). 1.0 = 표준 Theta\*, 초과 = weighted |

## 구현 노트

- C++: `cpp/src/global_planning/search/theta_star.cpp`, Python: `python/navigation/global_planning/search/theta_star.py`
- 유클리드 거리는 `hypot` 이 아니라 **`sqrt(Δrow² + Δcol²)`** 로 계산한다. `hypot` 은 IEEE-754
  정확 반올림이 아니라 런타임 간 1 ULP 차이가 날 수 있어, f 값·tie-break·방출 trace 스트림이 두
  언어에서 어긋난다(벤치의 C++/Python 비교가 이 스트림에 의존). `sqrt` 는 정확 반올림이며
  `sqrt(2.0)` 가 `neighbors()` 의 대각 간선 비용과 정확히 일치해, Path 2 대각 지름길과 Path 1
  대각 스텝이 bit 단위로 같다.
- 경로 비용은 `g[goal]` 로 보고한다. 인접 간선을 합산하는 방식은 any-angle 점프(비인접)에서 0 이
  되어 틀린다.
- tie-break 은 A* 와 동일한 `(f, 삽입순서)` 이라 두 언어가 같은 경로로 수렴한다 — demo 의
  maze01 확장 노드 순서가 C++/Python 에서 셀 단위로 일치한다.

## 방출 trace 이벤트

`planning_started` → (`node_expanded`, `candidate_evaluated`, `edge_added`)* → `path_found` → `planning_finished`

`edge_added(state=s2, parent)` 의 `parent` 는 Path 2 에서 **비인접 조부모**가 된다(스키마·시각화는
인접 제약을 두지 않는다). `replay.py` 는 parent→state 직선을 그대로 그려 any-angle 지름길을
표현하므로 새 trace 이벤트가 필요 없다.

## Demo

`maze01` 에서의 탐색. A* 파면과 비슷하게 goal 쪽으로 자라지만, 최종 경로가 grid 계단이 아니라
장애물 모서리만 스치는 **성근 직선 다각선**이다.

![Theta* on maze01](../../assets/theta_star/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/theta_star/maze01_snap_02.png) | ![mid](../../assets/theta_star/maze01_snap_05.png) | ![final](../../assets/theta_star/maze01_final.png) |

`open01` 최종 결과 — 장애물이 적으면 start→goal 가 거의 단일 직선으로 연결된다:

![Theta* on open01](../../assets/theta_star/open01_final.png)

측정치 (Python, w = 1.0, trace on · 같은 인스턴스의 A* 비교):

| map | Theta\* cost | A\* cost | Theta\* expanded | A\* expanded | Theta\* waypoints |
|---|---|---|---|---|---|
| maze01 | **27.748** | 28.728 | 104 | 108 | 4 (A\*: 26) |
| open01 | **24.241** | 25.213 | 66 | 71 | 3 (A\*: 20) |

재현:

```bash
python python/demos/demo_theta_star.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/theta_star.yaml --trace out/theta_star.jsonl
python tools/viz/replay.py out/theta_star.jsonl --gif out/theta_star.gif --snapshots out/theta_snaps/
```

## References

[^nash]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^aw]: Amanatides, J., & Woo, A. (1987). "A Fast Voxel Traversal Algorithm for Ray Tracing." *Proc. Eurographics*, 3–10. [PDF](https://www.cse.yorku.ca/~amana/research/grid.pdf)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
[^pohl]: Pohl, I. (1970). "Heuristic search viewed as path finding in a graph." *Artificial Intelligence*, 1(3–4), 193–204. [doi:10.1016/0004-3702(70)90007-X](https://doi.org/10.1016/0004-3702%2870%2990007-X)
