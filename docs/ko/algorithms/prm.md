---
title: PRM
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 6
---

[🇰🇷 한국어](prm.md) | [🇬🇧 English](../../en/algorithms/prm.md)

# PRM (Probabilistic Roadmap)
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | sampling-based, multi-query roadmap (여기서는 single-query 로 사용) |
| 요구 capability | `SamplingSpace` |
| 완전성 | probabilistically complete |
| 최적성 | **점근 최적 아님** — 고정 반경이라 해의 품질이 샘플 수·반경에 의존 |
| 복잡도 | 소박한 구현 기준 간선 검사 O(n²) + 질의 Dijkstra O(E log V) |
| 원 논문 | Kavraki, Švestka, Latombe & Overmars (1996) [^kavraki] |

1. TOC
{:toc}

## 배경

Kavraki 등[^kavraki] 은 고차원 configuration space 에서 여러 질의(multi-query)를 빠르게
답하기 위한 **로드맵(roadmap)** 방식을 제안했다. 자유 공간을 미리 무작위로 표본화해 그래프를
만들어 두면, 이후의 start→goal 질의는 그래프 위 최단 경로 탐색으로 환원된다. 이 프로젝트는
로드맵을 매 질의마다 새로 짓는 **single-query** 형태로 사용하지만, 알고리즘 골격은 원 논문 그대로다.

PRM 은 뒤이은 점근 최적 변형([PRM\*](prm_star.md), [FMT\*](fmt_star.md), [BIT\*](bit_star.md))이
공통으로 딛고 선 **로드맵의 토대**다. 이들은 "무엇을 어떻게 연결하는가"의 반경 정책만 바꾼다.

두 단계로 나뉜다:

1. **학습(learning)** — 충돌 없는 노드를 `num_samples` 개 표본화하고, 서로 `connection_radius`
   이내인 모든 쌍을 국소 계획기(직선 motion 의 충돌 검사)로 연결한다.
2. **질의(query)** — start·goal 을 노드로 추가해 같은 반경으로 이은 뒤, 로드맵 위에서 Dijkstra 로
   최단 경로를 찾는다.

## 동작 원리

`maze01` — 자유 공간에 표본이 흩뿌려지고, 반경 이내 쌍이 간선으로 이어져 로드맵이 형성된 뒤
Dijkstra 가 그 위에서 최단 경로를 뽑는다.

![PRM on maze01](../../assets/prm/maze01.gif)

탐색 중간 과정 (좌 → 우: 표본/간선 초반 / 로드맵 형성 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/prm/maze01_snap_02.png) | ![mid](../../assets/prm/maze01_snap_05.png) | ![final](../../assets/prm/maze01_final.png) |

`open01` 최종 결과 — 거의 직선에 가깝다:

![PRM on open01](../../assets/prm/open01_final.png)

```
PRM(start, goal):
    R ← roadmap()
    R.add(start); R.add(goal)
    for i in 1..num_samples:                       # 학습: 자유 노드 표본화
        q ← sample()
        if is_state_valid(q): R.add(q)
    for v in R.nodes:                               # 학습: 고정 반경 연결
        for u in R.nodes within connection_radius of v (u before v):
            if is_motion_valid(u, v):
                R.add_edge(u, v, distance(u, v))
    return DIJKSTRA(R, start, goal)                 # 질의
```

이 구현은 노드를 하나씩 추가하며 "자신보다 먼저 추가된 노드"에 대해서만 연결을 시도한다 —
무향 간선이 정확히 한 번만 생긴다. start·goal 도 로드맵의 일부이므로 별도 연결 단계 없이
같은 반경 규칙으로 이어진다.

측정치 (Python, seed = 1, trace on):

| map | path cost | roadmap 노드 | expanded (Dijkstra pop) |
|---|---|---|---|
| maze01 | 13.595 | 1,502 | 1,091 |
| open01 | 12.053 | — | — |

C++ 구현도 동일 시나리오를 미러링하며, 언어 간 난수 스트림 차이 범위 안에서 같은 결과를 낸다.

재현:

```bash
python python/demos/demo_prm.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/prm.yaml --trace out/prm.jsonl
python tools/viz/replay.py out/prm.jsonl --gif out/prm.gif
```

## 성질

- **완전성**: probabilistically complete — 해가 존재하면 표본 수 → ∞ 에서 확률 1 로 발견한다[^kavraki].
- **최적성**: **점근 최적이 아니다.** 반경이 상수로 고정되어 있어 그래프의 간선 밀도가 표본 수에
  비례해 증가한다. 해의 품질은 `num_samples` 와 `connection_radius` 에 의존하며, 극한에서도
  최단 경로로의 수렴이 보장되지 않는다. 이 한계를 반경 정책으로 고친 것이 [PRM\*](prm_star.md) 다.
- **비용**: 소박한 구현은 모든 쌍 거리 검사로 O(n²) 간선 후보를 본다. 다중 질의 상황에서는
  로드맵을 재사용하므로 질의당 비용이 Dijkstra 로 상각된다.

## 로드맵 구성과 질의

**로드맵.** 표본 집합 $V=\{v_1,\dots,v_n\}\subset X_{\text{free}}$ 에 대해 간선 집합은

$$
E=\bigl\{(u,v)\in V\times V:\lVert u-v\rVert\le r,\ \text{motion}(u,v)\subset X_{\text{free}}\bigr\},
$$

즉 **고정 반경** $r=\texttt{connection\_radius}$ 이내이면서 직선 motion 이 충돌하지 않는 모든 쌍이다.
간선 비용은 유클리드 거리 $c(u,v)=\lVert u-v\rVert$.

**질의.** start $s$, goal $g$ 를 $V$ 에 넣고 로드맵 위에서 Dijkstra 로

$$
Y_n=\min_{\pi:\,s\rightsquigarrow g\ \text{in}\ (V,E)}\ \sum_{k}c(\pi_k,\pi_{k+1})
$$

를 푼다. $r$ 이 고정이므로 $n\to\infty$ 에서 $Y_n$ 은 그래프가 조밀해질수록 개선되지만
최적값 $c^*$ 로의 수렴은 보장되지 않는다 — 이 점이 점근 최적 변형과의 결정적 차이다.

**확률적 완전성 (도출).** clearance $\delta>0$ 경로를 반지름 $\delta/2$ 공 $B_1,\dots,B_m$ 으로 덮고
$r=\texttt{connection\_radius}\ge\delta$ 라 하자. 한 공에 표본이 하나 이상 들어갈 확률은 $n$ 표본에서
$1-(1-p)^n$ ($p=\mu(B)/\mu(X_{\text{free}})$). 이웃한 두 공의 표본은 중심 거리 $\le\delta\le r$ 라
간선으로 이어지므로, 모든 $m$ 개 공이 표본을 가지면 로드맵에 $s\rightsquigarrow g$ 경로가 반드시
존재한다. 그 확률은

$$
P[\text{연결}]\;\ge\;1-m(1-p)^n\;\xrightarrow{\,n\to\infty\,}\;1.
$$

즉 PRM 은 probabilistically complete 다(품질은 별개 — 위 참조). ∎

## 파라미터

| 이름 | 타입 | 기본값 | 범위 | 설명 |
|---|---|---|---|---|
| `num_samples` | int | 1500 | [1, 200000] | 로드맵에 배치할 충돌 없는 샘플 수 (start/goal 제외) |
| `connection_radius` | float | 2.0 | [0.01, 100.0] | 두 노드를 간선으로 이을 최대 거리 (m) |
| `seed` | int | 1 | [0, 2^31−1] | 난수 시드 (재현성) |

## 방출 trace 이벤트

`planning_started` → `sample_drawn`\* → `edge_added`\* → `node_expanded`\* → `path_found` → `planning_finished`

`sample_drawn` 은 학습 단계의 표본, `edge_added` 는 로드맵 간선, `node_expanded` 는 질의
단계에서 Dijkstra 가 pop 하는 노드다.

## References

[^kavraki]: Kavraki, L. E., Švestka, P., Latombe, J.-C., & Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580. [doi:10.1109/70.508439](https://doi.org/10.1109/70.508439)
