---
title: RRT*
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 5
---

[🇰🇷 한국어](rrt_star.md) | [🇬🇧 English](../../en/algorithms/rrt_star.md)

# RRT* (RRT-star)
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | sampling-based, single-query, anytime |
| 요구 capability | `SamplingSpace` |
| 완전성 | probabilistically complete |
| 최적성 | **asymptotically optimal** — 샘플 수 → ∞ 에서 최적 경로에 확률 1 로 수렴 |
| 복잡도 | 반복당 near-neighbor 질의 지배, O(log n) 개 이웃 검사 시 총 O(n log n) |
| 원 논문 | Karaman & Frazzoli (2011) [^karaman] |

1. TOC
{:toc}

## 배경

Karaman & Frazzoli[^karaman] 는 RRT 가 **어떤 경우에도 최적 경로에 수렴하지 않음**(수렴 확률 0)을
증명하고, 두 가지 국소 수선 연산을 더해 asymptotic optimality 를 획득한 RRT* 를 제안했다.
이후 sampling 기반 optimal planning 연구(Informed RRT*, BIT*, FMT* 등)의 출발점이 된
기념비적 논문이다.

RRT 와의 차이는 새 노드를 트리에 붙일 때의 두 연산이다:

1. **Choose-parent** — nearest 노드가 아니라, 반경 r 내 이웃 중 **start 로부터의 누적 비용이
   최소가 되는** 노드를 부모로 선택한다.
2. **Rewire** — 반경 r 내 기존 이웃들에 대해, 새 노드를 경유하는 편이 더 싸면 부모를 새 노드로
   교체한다. 트리가 사후적으로 계속 "곧게 펴진다".

## 동작 원리

`maze01` — 8,000 샘플 동안 트리가 자유 공간을 빽빽하게 덮고, rewire 로 경로가 점점 곧아진다.
최종 경로가 [RRT](rrt.md) 의 지그재그와 뚜렷이 대비된다.

![RRT* on maze01](../../assets/rrt_star/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/rrt_star/maze01_snap_02.png) | ![mid](../../assets/rrt_star/maze01_snap_05.png) | ![final](../../assets/rrt_star/maze01_final.png) |

`open01` 최종 결과 — 거의 직선에 가깝다:

![RRT* on open01](../../assets/rrt_star/open01_final.png)

```
RRT_STAR(start, goal):
    T ← {start}
    for i in 1..max_iterations:                       # anytime — 끝까지 돈다
        x_rand ← (goal with prob. goal_bias) else sample()
        x_near ← nearest(T, x_rand)
        x_new  ← steer(x_near, x_rand, step_size)
        if not is_motion_valid(x_near, x_new): continue
        N ← near(T, x_new, neighbor_radius)
        parent ← argmin_{x ∈ N ∪ {x_near}} cost(x) + c(x, x_new)   # choose-parent
        T.add(x_new, parent)
        for x ∈ N:                                                 # rewire
            if cost(x_new) + c(x_new, x) < cost(x) and is_motion_valid(x_new, x):
                x.parent ← x_new
        if distance(x_new, goal) ≤ goal_tolerance:
            best ← min(best, path through x_new)       # 해를 갱신하며 계속 탐색
    return best
```

RRT 는 첫 해에서 멈추지만 RRT* 는 **반복 예산을 끝까지 소진**하며 현재 최선 해(incumbent)를
계속 개선한다 — anytime 알고리즘이다.

측정치 (seed = 1, 8000 iterations, trace on):

| map | 언어 | path cost | tree size | runtime |
|---|---|---|---|---|
| maze01 | Python | 13.458 | 5,915 | 9.15 s |
| maze01 | C++ | 13.471 | 5,949 | 1.09 s |
| open01 | Python | 12.047 | 5,483 | 8.35 s |
| open01 | C++ | 12.048 | 5,481 | 0.98 s |

[RRT](rrt.md) 의 첫 해(18.41 / 14.37) 대비 16–27% 짧다. 언어 간 비용 차이는 난수 스트림 차이로,
0.1% 이내다. (runtime 은 trace 방출 포함 수치 — 상대 비교용.)

재현:

```bash
python python/demos/demo_rrt_star.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/rrt_star.yaml --trace out/rrt_star.jsonl
python tools/viz/replay.py out/rrt_star.jsonl --gif out/rrt_star.gif
```

## 성질

- **완전성**: probabilistically complete (RRT 와 동일)[^karaman].
- **최적성**: asymptotically optimal. 이론적으로 이웃 반경이 r(n) = γ(log n / n)^(1/d) 로
  줄어들 때 성립한다[^karaman]. 이 구현은 단순화를 위해 **고정 반경** `neighbor_radius` 를
  쓴다 — 충분히 큰 고정 반경에서도 asymptotic optimality 가 유지되지만 반복당 비용이 커진다.
- **비용**: RRT 대비 반복당 near 질의 + rewire 검사만큼 느리다. "품질 ↔ 시간" 트레이드오프.

## 점근적 최적성 근거와 증명

**핵심 두 연산.** near 집합 $X_{\text{near}}=\{x\in T:\lVert x-x_{\text{new}}\rVert\le r_n\}$ 에 대해

- **Choose-parent** —

$$
x_{\min}=\arg\min_{x\in X_{\text{near}}\cup\{x_{\text{nearest}}\}}\!\Bigl(\mathrm{cost}(x)+c(x,x_{\text{new}})\Bigr),\qquad
\mathrm{cost}(x_{\text{new}})=\mathrm{cost}(x_{\min})+c(x_{\min},x_{\text{new}}).
$$

- **Rewire** — $x\in X_{\text{near}}$ 에 대해 $\mathrm{cost}(x_{\text{new}})+c(x_{\text{new}},x)<\mathrm{cost}(x)$
  이고 선분이 collision-free 면 $x$ 의 부모를 $x_{\text{new}}$ 로 교체.

여기서 $c(\cdot,\cdot)$ 는 collision-free 선분 비용이다.

**보조정리 (cost 단조성).** 임의 노드 $v$ 의 $\mathrm{cost}(v)$ 는 반복이 진행돼도 **비증가**한다.
choose-parent 는 $v$ 를 붙일 때 가능한 최소 cost 부모를 고르고, rewire 는
$\mathrm{cost}(x_{\text{new}})+c(x_{\text{new}},x)<\mathrm{cost}(x)$ 일 때만 부모를 바꾼다 — 두 연산
모두 cost 를 낮추거나 유지할 뿐, 절대 올리지 않는다. 따라서 트리가 표현하는 각 정점의 경로 비용은
단조 개선되며, incumbent(현재 최선 해)가 나빠지지 않는 anytime 성질이 여기서 나온다. ∎

**정리 (점근적 최적성, Karaman & Frazzoli 2011).** near 반경을

$$
r_n=\min\!\left\{\gamma_{\text{RRT}^*}\left(\frac{\log n}{n}\right)^{1/d},\;\eta\right\},\qquad
\gamma_{\text{RRT}^*}>\gamma^*=2\left(1+\frac1d\right)^{1/d}\left(\frac{\mu(X_{\text{free}})}{\zeta_d}\right)^{1/d}
$$

($\zeta_d$: 단위 $d$-볼 부피) 로 두면

$$
P\!\left[\lim_{n\to\infty}Y_n^{\text{RRT}^*}=c^*\right]=1.
$$

*직관.* 반경이 $(\log n/n)^{1/d}$ 로 줄되 충분히 크면 각 노드가 기대상 $\Theta(\log n)$ 개의 near
이웃을 가져, rewire 가 극한에서 최적과 같은 homotopy 류의 경로를 복원한다. 너무 빨리 줄이면
(예: 고정 $k=1$) 연결성이 끊겨 최적성이 깨진다.

{: .note }
> 이 구현은 단순화를 위해 **고정 반경** `neighbor_radius` 를 쓴다. 충분히 큰 상수 반경도 거의 확실한
> 최적성을 유지하지만 반복당 near 질의 비용이 커진다 (그래서 8,000 반복에서 RRT 보다 느리다).

## 파라미터

| 이름 | 타입 | 기본값 | 범위 | 설명 |
|---|---|---|---|---|
| `max_iterations` | int | 8000 | [1, 200000] | 반복 예산 (anytime — 소진 시 현재 best 반환) |
| `step_size` | float | 0.5 | [0.01, 100.0] | steer 확장 거리 η (m) |
| `goal_bias` | float | 0.05 | [0.0, 1.0] | goal 을 직접 sample 할 확률 |
| `goal_tolerance` | float | 0.3 | [0.0, 100.0] | goal 도달 판정 반경 (m) |
| `neighbor_radius` | float | 1.5 | [0.01, 100.0] | choose-parent / rewire 근방 반경 (m) |
| `seed` | int | 1 | [0, 2^31−1] | 난수 시드 (재현성) |

## 방출 trace 이벤트

`planning_started` → (`sample_drawn`, `edge_added`, `rewire`*)* → `path_found`* → `planning_finished`

`path_found` 가 여러 번 방출될 수 있다 (incumbent 개선 시마다).

## References

[^karaman]: Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846–894. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
