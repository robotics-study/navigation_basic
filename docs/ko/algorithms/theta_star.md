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

`maze01` 에서의 탐색. A* 파면과 비슷하게 goal 쪽으로 자라지만, 최종 경로가 grid 계단이 아니라
장애물 모서리만 스치는 **성근 직선 다각선**이다.

![Theta* on maze01](../../assets/theta_star/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/theta_star/maze01_snap_02.png) | ![mid](../../assets/theta_star/maze01_snap_05.png) | ![final](../../assets/theta_star/maze01_final.png) |

`open01` 최종 결과 — 장애물이 적으면 start→goal 가 거의 단일 직선으로 연결된다:

![Theta* on open01](../../assets/theta_star/open01_final.png)

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

## 성질

- **완전성**: 유한 grid + 비음수 비용에서 완전 (A* 와 동일).
- **최적성**: basic Theta\* 는 **grid-optimal 도 true-optimal 도 보장하지 않는다**[^nash]. 부모를
  조부모로만 승계하는 국소 규칙이라, 진짜 최단 any-angle 경로를 놓칠 수 있다(대신 매우 근접). 엄밀
  최적이 필요하면 Lazy Theta\*/AP Theta\* 등 후속 변형을 쓴다.
- **품질**: 반환 경로 비용은 항상 같은 grid 의 A* 이하다 — LOS 지름길이 grid 계단을 대체하기 때문.
- **복잡도**: A* 와 같은 탐색에 relaxation 마다 LOS 검사(선분 길이에 선형) 1회가 더해진다.
- w > 1 (weighted Theta\*, Pohl 1970[^pohl]): heuristic 을 부풀려 확장 노드를 줄이되 경로 품질을 완화.

## Any-angle 비용 모델과 최적성

**기호.** 격자 셀 중심을 평면 $\mathbb{R}^2$ 위의 점으로 본다. any-angle 경로
$P=(v_0,v_1,\dots,v_k)$ 는 셀 중심을 잇는 꺾은선이며, 각 선분 $\overline{v_i v_{i+1}}$ 은 장애물을
지나지 않아야(LOS) 한다. 비용은 유클리드 길이의 합이다:

$$
\operatorname{cost}(P)=\sum_{i=0}^{k-1}\lVert v_{i+1}-v_i\rVert_2 .
$$

$C^\ast$ 는 feasible 한 꺾은선 중 최소 비용, $h(n)=\lVert n-\text{goal}\rVert_2$, $h^\ast(n)$ 는
$n$ 에서 goal 까지의 참 최단 feasible 비용이다.

**보조정리 1 (유클리드 heuristic 은 admissible·consistent).**
feasible 경로 $P=(v_0,\dots,v_k)$ 의 양 끝점을 $a=v_0,\ b=v_k$ 라 하자. 각 선분에 삼각부등식을
반복 적용해 텔레스코핑하면

$$
\lVert b-a\rVert_2=\Bigl\lVert\textstyle\sum_{i}(v_{i+1}-v_i)\Bigr\rVert_2
   \;\le\;\sum_{i}\lVert v_{i+1}-v_i\rVert_2=\operatorname{cost}(P),
$$

즉 어떤 feasible 경로도 두 끝점의 직선거리보다 짧을 수 없다. 이 하한을 $a=n,\ b=\text{goal}$ 에
적용하면

$$
h(n)=\lVert n-\text{goal}\rVert_2\;\le\;\operatorname{cost}(P)\ \text{ for all feasible }P
\;\Longrightarrow\; h(n)\le h^\ast(n)\qquad(\text{admissible}).
$$

또 이동 비용이 유클리드 길이 $c(a,b)=\lVert a-b\rVert_2$ 인 임의의 쌍 $(a,b)$ 에 대해 삼각부등식

$$
h(a)=\lVert a-\text{goal}\rVert_2\;\le\;\lVert a-b\rVert_2+\lVert b-\text{goal}\rVert_2
      \;=\;c(a,b)+h(b)
$$

이 성립하므로 consistent 하다. A\* 의 성질과 마찬가지로 consistent $\Rightarrow$ 임의 경로에서 $f$
가 비감소 $\Rightarrow$ 각 노드는 최대 한 번만 확장된다. ∎

**왜 octile 을 쓰지 않는가 (정량).** A\* 용 octile heuristic 은 $\Delta r\ge\Delta c$ 일 때
$\text{oct}=\Delta r+(\sqrt2-1)\,\Delta c$ 이다. any-angle 참값 $\lVert\cdot\rVert_2=\sqrt{\Delta r^2+\Delta c^2}$
에 대한 비율 $\rho(t)=\dfrac{1+(\sqrt2-1)t}{\sqrt{1+t^2}}\ (t=\Delta c/\Delta r\in[0,1])$ 를
미분해 극값을 구하면 $t^\ast=\sqrt2-1$ 에서 최대

$$
\max_t\ \frac{\text{oct}}{\lVert\cdot\rVert_2}=\sqrt{4-2\sqrt2}\approx 1.0824 .
$$

즉 octile 은 any-angle 비용을 최대 8.24 % 과대평가(inadmissible)하므로 최적성 논거가 깨진다.
유클리드는 위 하한이 보장하는, 임의 각도 이동에 대한 admissible 하한이다.

**보조정리 2 ($g$ 값은 실재하는 feasible 경로의 길이).**
노드 $s$ 가 settle 될 때 항상 $\text{parent}[s]=p$ 는 LOS$(p,s)$ 가 성립하는 노드이고

$$
g[s]=g[p]+\lVert p-s\rVert_2
$$

이다 (Path 1 은 인접 셀이라 LOS 가 자명, Path 2 는 명시적으로 검사). 부모 사슬을 start 까지 펼치면
$g[s]$ 는 $\text{start}\to\cdots\to p\to s$ 라는 **충돌 없는 꺾은선의 실제 길이**와 정확히 같다.
특히 반환된 $g[\text{goal}]$ 는 언제나 달성 가능한 상계다 — 즉 Theta\* 가 내놓는 경로는 항상
실행 가능하다. ∎

**명제 3 (grid A\* 이하).** Theta\* 가 반환한 경로 $P_\Theta$ 는 같은 격자에서 A\* 가 반환한
격자 경로 $P_A$ 보다 길지 않다: $\operatorname{cost}(P_\Theta)\le\operatorname{cost}(P_A)$.[^nash]

*증명 스케치.* 모든 LOS 검사가 실패하면 Path 2 는 한 번도 발동하지 않고 알고리즘은 weighted A\* 와
완전히 동일해 $P_\Theta=P_A$. LOS 가 성공하는 지점마다 Path 2 는 조부모–손자 직선 $p\to s_2$ 로
두 격자 다리 $p\to s\to s_2$ 를 대체하는데, 삼각부등식

$$
\lVert p-s_2\rVert_2\;\le\;\lVert p-s\rVert_2+\lVert s-s_2\rVert_2
$$

가 성립하고 relaxation 은 $g$ 가 더 작아질 때만 이 대체를 채택한다. 따라서 어떤 지름길도 길이를
늘리지 않으며, $P_\Theta$ 는 격자 경로에서 시야가 트인 구간을 곧게 편 결과로 비용이 단조 비증가한다. ∎

**명제 4 (엄밀 최적은 아니다).** basic Theta\* 는 $C^\ast$ 도, 격자상 최단 any-angle 경로도
보장하지 않는다.[^nash]

*구조적 근거.* 장애물이 다각형일 때 최단 any-angle 경로는 팽팽히 당긴 실(taut string)로, 꺾임점이
**볼록 장애물 모서리에만** 놓인다(그 사이 구간은 직선). 최적 경로를 복원하려면 relaxation 이
"현재까지의 실이 마지막으로 감긴 모서리"를 부모로 지목할 수 있어야 한다. 그러나 Theta\* 는 후보
$s_2$ 의 부모를 오직

$$
\text{parent}(s_2)\in\{\,s,\ \text{parent}(s)\,\}
$$

두 값 중에서만 고른다. 최적 실이 감겨야 할 모서리 $O$ 가 확장 순간의 이 2-원소 집합에 없으면 —
예컨대 $O$ 가 $s$ 의 부모보다 **두 세대 이상 앞선** 조상이거나, $O$ 로 오는 실이 다른 확장 가지에서
왔으면 — 직선 지름길 $O\to s_2$ 는 결코 생성되지 않고, Theta\* 는 $\text{parent}(s)$ 에서 격자를
따라 $O$ 근방을 우회하는 약간 긴 경로로 근사한다.

*왜 heuristic 문제가 아닌가.* 보조정리 1 로 $h$ 는 admissible 이므로 $w=1$ 에서 A\* 골격 자체는
최적을 놓치지 않는다. 격차는 오직 **부모 후보 집합 $\{s,\text{parent}(s)\}$ 의 국소성**에서
온다 — 즉 경로 비용을 정의하는 relaxation 이 국소적으로 근시안적이라 실을 완전히 당기지 못한다.
실측에서는 $C^\ast$ 의 1 % 안쪽이 보통이다(이 저장소 maze01: Theta\* 27.75 vs A\* 격자 28.73).
엄밀 최적이 필요하면 **모든** 가시 조상을 후보로 삼는 후속 변형(Lazy Theta\*, AP Theta\*)을 쓴다. ∎

## 파라미터

| 이름 | 타입 | 기본값 | 범위 | 설명 |
|---|---|---|---|---|
| `heuristic_weight` | float | 1.0 | [1.0, 5.0] | f = g + w·h 의 w (h 는 유클리드). 1.0 = 표준 Theta\*, 초과 = weighted |

## 방출 trace 이벤트

`planning_started` → (`node_expanded`, `candidate_evaluated`, `edge_added`)* → `path_found` → `planning_finished`

`edge_added(state=s2, parent)` 의 `parent` 는 Path 2 에서 **비인접 조부모**가 된다(스키마·시각화는
인접 제약을 두지 않는다). `replay.py` 는 parent→state 직선을 그대로 그려 any-angle 지름길을
표현하므로 새 trace 이벤트가 필요 없다.

## References

[^nash]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^aw]: Amanatides, J., & Woo, A. (1987). "A Fast Voxel Traversal Algorithm for Ray Tracing." *Proc. Eurographics*, 3–10. [PDF](https://www.cse.yorku.ca/~amana/research/grid.pdf)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
[^pohl]: Pohl, I. (1970). "Heuristic search viewed as path finding in a graph." *Artificial Intelligence*, 1(3–4), 193–204. [doi:10.1016/0004-3702(70)90007-X](https://doi.org/10.1016/0004-3702%2870%2990007-X)
