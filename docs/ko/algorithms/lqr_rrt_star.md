---
title: LQR-RRT*
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 16
---

[🇰🇷 한국어](lqr_rrt_star.md) | [🇬🇧 English](../../en/algorithms/lqr_rrt_star.md)

# LQR-RRT*
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | sampling-based, single-query, anytime, **kinodynamic** |
| 요구 capability | `SamplingSpace` |
| 완전성 | probabilistically complete |
| 최적성 | asymptotically optimal — LQR 비용 기준 (Perez et al. 2012) |
| 복잡도 | 반복당 near-neighbor 마다 LQR feedback roll(적분) 계산이 지배. 단, metric·gain 은 Riccati 를 **1회** 풀어 재사용 |
| 원 논문 | Perez, Platt, Konidaris, Kaelbling & Lozano-Pérez (2012) [^lqr] |

1. TOC
{:toc}

## 배경

RRT* 를 동역학이 있는 시스템에 쓰려면 두 가지 **확장 휴리스틱(extension heuristic)** 이 필요하다:
두 상태 사이 **거리 metric** 과, 한 상태에서 다른 상태로 뻗는 **steering** 함수다. 기하 RRT* 는
이를 유클리드 거리와 직선으로 손쉽게 얻지만, 동역학이 붙으면 둘 다 손으로 설계하기 어렵다.
Kinodynamic RRT*[^webb] 는 fixed-final-state OBVP 를 정확히 풀어 이를 얻지만(2013), 시스템마다
닫힌 형태 해를 유도해야 한다.

Perez et al.[^lqr] 은 이 둘을 **LQR(Linear-Quadratic Regulator) 로 자동 유도** 한다. 동역학을
국소 선형화하고 2차 비용 $J=\int(x^\top Q x + u^\top R u)\,dt$ 를 정하면, LQR 해가

- **거리 metric** $\mathrm{dist}(a,b)=(a-b)^\top S\,(a-b)$ — $S$ 는 Riccati cost-to-go 행렬,
- **steering** — LQR feedback 정책 $u=-K(x-x_\text{ref})$, $K=(R+B^\top P B)^{-1}B^\top P A$ 를 전파,

를 자동으로 준다. 손으로 짠 metric/steer 가 필요 없다. 계보상 기하 RRT*(2011, 직선·유클리드)와
Kinodynamic RRT*(2013, 정확 OBVP) **사이의 다리** 다: LQR 은 값싸고 일반적인 *feedback* 확장을
주는 대신, 정확한 2-point 해가 아니라 **점근적(asymptotic)** 이다.

이 구현은 repo 의 Kinodynamic RRT* 와 **동일한 2D double integrator** 를 소유한다: 상태
$(x, y, v_x, v_y)$, 제어 = 가속도. 같은 벤치마크 위에서 두 알고리즘을 비교하기 위해서다.
`SamplingSpace` capability 에만 의존한다 — 맵은 $(x, y)$ 투영에 대해 `is_motion_valid` 만 답하고
속도는 모른다.

## 동작 원리

`maze01` — 트리 edge 가 LQR feedback 궤적(곡선)이다. 노드는 정지(rest) 상태로, 각 edge 는 부모에서
자식 rest 상태로 매끄럽게 가·감속하는 실행 가능 궤적이다. rewire 가 LQR 비용 공간에서 incumbent 를
곧게 편다.

![LQR-RRT* on maze01](../../assets/lqr_rrt_star/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/lqr_rrt_star/maze01_snap_02.png) | ![mid](../../assets/lqr_rrt_star/maze01_snap_05.png) | ![final](../../assets/lqr_rrt_star/maze01_final.png) |

`open01` 최종 결과:

![LQR-RRT* on open01](../../assets/lqr_rrt_star/open01_final.png)

```
LQR_RRT_STAR(start, goal):
    S, K ← SOLVE_RICCATI(A, B, Q, R)                   # DARE 1회 — metric·gain 유도
    x_start ← (start, v=0);  x_goal ← (goal, v=0)      # 정지 상태로 들어올림
    T ← {x_start}
    for i in 1..max_iterations:                        # anytime — 끝까지 돈다
        x_rand ← (확률 goal_bias 로 goal 정지상태) else (위치 sample + 임의 속도)
        x_near ← argmin_{x∈T} (x−x_rand)ᵀ S (x−x_rand)  # LQR metric
        x_new  ← x_near 에서 x_rand 방향 step_size 거리의 rest waypoint
        (edge, c) ← LQR_STEER(x_near, x_new)           # u=−K(x−x_new) 적분, 충돌검사
        if edge = ∅: continue
        N ← near(T, x_new, neighbor_radius)
        parent ← argmin_{x∈N∪{x_near}} cost(x) + LQR_STEER(x, x_new).cost   # choose-parent
        T.add(x_new, parent)
        for x ∈ N:                                     # rewire
            if cost(x_new) + LQR_STEER(x_new, x).cost < cost(x): x.parent ← x_new
        if ‖x_new − goal‖ ≤ goal_tolerance:
            best ← min(best, path through x_new to x_goal)
    return best
```

`LQR_STEER(a,b)` 는 feedback 정책 $u=-K(x-b)$ 를 clamp 하며 $a$ 에서 rest 목표 $b$ 로 적분해
궤적과 realised LQR 비용 $\sum(x^\top Q x + u^\top R u)\,dt$ 를 낸다. 각 궤적은 waypoint 를 샘플링해
`is_motion_valid` 로 충돌 검사한다(기하 RRT* 가 직선 edge 에 하던 검증과 동일).

측정치 (Python, seed = 1, 4000 iterations, trace on):

| map | path cost (LQR) | tree size | runtime |
|---|---|---|---|
| maze01 | 4.643 | 2,904 | ~1.9 s |
| open01 | 4.269 | 2,775 | ~1.8 s |

비용 스케일이 Kinodynamic RRT* 와 다른 것은 정상이다 — LQR 의 stage cost $\int x^\top Q x+u^\top R u$ 는
Webb 의 $\int 1+u^\top R u$ 와 다른 비용 함수다. C++ 구현은 동일 알고리즘이며, 난수 스트림이 Python 과
달라 정확한 비용은 다른 planner 들과 마찬가지로 미세하게 다르다.

재현:

```bash
python python/demos/demo_lqr_rrt_star.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/lqr_rrt_star.yaml --trace out/lqr.jsonl
python tools/viz/replay.py out/lqr.jsonl --gif out/lqr.gif
```

## 자동 유도되는 확장 휴리스틱 — LQR

**시스템.** 위치 $p\in\mathbb{R}^2$, 속도 $v\in\mathbb{R}^2$, 상태 $x=(p,v)$ 의 double integrator 는
축별로 분리된다:

$$
A=\begin{bmatrix}0 & 1\\ 0 & 0\end{bmatrix},\quad
B=\begin{bmatrix}0\\ 1\end{bmatrix},\quad
Q=\mathrm{diag}(q_\text{pos}, q_\text{vel}),\quad
R=r_\text{ctrl}.
$$

**Riccati.** 무한지평 LQR 의 cost-to-go 행렬 $S$ 는 대수 Riccati 방정식을 만족한다. 연속시간형은

$$
A^\top S + S A - S B R^{-1} B^\top S + Q = 0,
$$

이고 최적 gain 은 $K=R^{-1}B^\top S$ 다. 구현은 시스템을 $dt$ 로 이산화($A_d=[[1,dt],[0,1]]$,
$B_d=[[dt^2/2],[dt]]$)한 뒤 **이산 Riccati 반복(DARE)** 의 고정점을 취한다:

$$
P \leftarrow Q + A_d^\top P A_d - A_d^\top P B_d\,(R + B_d^\top P B_d)^{-1} B_d^\top P A_d,\qquad
K = (R + B_d^\top P B_d)^{-1} B_d^\top P A_d.
$$

double integrator 는 가제어이고 $Q\succ0$ 이므로 $P_0=Q$ 에서 반복은 유일한 안정화 해로 수렴한다.
LTI 이고 $Q$ 가 블록대각이라 $S,K$ 는 **상태 무관** — 축마다, 그리고 매 반복 동일하므로 planner 시작
시 **한 번만** 푼다. (Kinodynamic RRT* 와의 결정적 차이: 그쪽은 Riccati 를 풀지 않고 정확 도착시간을
위한 quartic 을 매 후보 edge 마다 근찾기 한다. 여기서는 metric·steer 가 $S,K$ 에서 나온다.)

기본값 $q_\text{pos}=q_\text{vel}=r_\text{ctrl}=1$, $dt=0.2$ 에서
$S\approx\begin{bmatrix}9.19 & 5.02\\ 5.02 & 9.23\end{bmatrix}$, $K\approx[0.841,\ 1.546]$.

**거리 metric.** 두 축 합으로

$$
\mathrm{dist}(a,b)=(a-b)^\top S\,(a-b)
= S_{00}(\Delta p_x^2+\Delta p_y^2) + 2S_{01}(\Delta p_x \Delta v_x + \Delta p_y \Delta v_y)
+ S_{11}(\Delta v_x^2 + \Delta v_y^2).
$$

$S_{00}=0$ iff $a=b$, 그 외 양수. sample 이 속도를 가지므로 nearest 선택은 위치뿐 아니라 속도
방향까지 반영한다(유클리드와 달리 방향 의존). $Q/R$ 을 바꾸면 $S$ 가 바뀌어 같은 기동의 비용이 달라진다.

**Steering.** feedback 정책 $u=-K(x-x_\text{ref})$ 를 $dt$ 스텝으로 적분한다(입력 clamp
$|u|\le u_\text{max}$). 노드를 **정지(rest)** 상태로 두면 rest 는 double integrator 의 평형점이라
닫힌 루프 $A-BK$ 가 Hurwitz — steady-state 오차 없이 목표에 정확히 regulate 한다. 따라서 저장되는
모든 edge 는 실제로 자식에 도달하는 실행 가능·충돌 없는 궤적이고, RRT* 의 rewire 가 정확히 성립한다.
sample 은 여전히 임의 속도를 실어 nearest metric 을 full-state 로 만들지만, 확장 자체는 sample
방향으로 `step_size` 이내 rest waypoint 로 steer 한다.

## 성질

- **완전성**: probabilistically complete[^lqr].
- **최적성**: LQR 비용 기준 asymptotically optimal — 선형화가 국소적으로 타당한 영역에서 incumbent 가
  최소 LQR 비용 궤적으로 수렴한다[^lqr].
- **Kinodynamic RRT* 와의 차이**: 확장이 정확한 2-point OBVP 가 아니라 **LQR feedback** 이다.
  값싸고(Riccati 1회 + 후보마다 적분) 일반적이지만 점근적이다. 이 구현은 노드를 정지 상태로 두어
  reach 를 정확히 만들고(정지-정지 regulation), 그 대가로 경로가 "waypoint 마다 정지" 하는
  가·감속열이다 — 관성을 궤적 내내 실어 나르는 Kinodynamic RRT* 와 대비된다.
- **실무 주의**: 고정 `neighbor_radius`(위치 prefilter)를 쓰고 near/nearest 후보를 상한으로 캡한다
  (k-nearest RRT* 변형, Karaman & Frazzoli 2011). 후보 사이 선택은 여전히 정확한 LQR 비용으로 한다.

## 파라미터

| 이름 | 타입 | 기본값 | 범위 | 설명 |
|---|---|---|---|---|
| `max_iterations` | int | 4000 | [1, 200000] | 반복 예산 (anytime — 소진 시 현재 best 반환) |
| `step_size` | float | 1.5 | [0.01, 100.0] | 확장 스텝 상한 $\eta$ (m). sample 방향 rest waypoint 까지의 거리 |
| `goal_bias` | float | 0.1 | [0.0, 1.0] | goal 정지상태를 직접 sample 할 확률 |
| `goal_tolerance` | float | 1.0 | [0.0, 100.0] | goal 연결을 시도하는 위치 근접 반경 (m) |
| `neighbor_radius` | float | 2.0 | [0.01, 100.0] | choose-parent / rewire 위치 근방 반경 (m) |
| `q_pos` | float | 1.0 | [0.001, 1000.0] | LQR 위치 상태비용 $q_\text{pos}$ ($Q=\mathrm{diag}(q_\text{pos},q_\text{vel})$) |
| `q_vel` | float | 1.0 | [0.001, 1000.0] | LQR 속도 상태비용 $q_\text{vel}$ |
| `r_ctrl` | float | 1.0 | [0.001, 1000.0] | LQR 제어비용 $r$ ($R=r\,I$) |
| `lqr_dt` | float | 0.2 | [0.01, 10.0] | LQR 이산화·적분 스텝 (s) |
| `control_limit` | float | 10.0 | [0.01, 1000.0] | 축별 제어 입력 clamp $|u|\le$ (m/s²) |
| `max_velocity` | float | 1.5 | [0.01, 100.0] | nearest metric 용 sample 속도 범위 $[-v_{\max}, v_{\max}]$ (m/s) |
| `seed` | int | 1 | [0, 2^31−1] | 난수 시드 (재현성) |

## 방출 trace 이벤트

`planning_started` → (`sample_drawn`, `candidate_evaluated`, `edge_added`, `rewire`*)* → `path_found`* → `planning_finished`

`path_found` 는 여러 번 방출될 수 있다 (incumbent 개선 시마다). edge 는 각 LQR feedback 궤적을 따라
chord 사슬로 방출되어 viz 가 곡선을 렌더링한다.

## References

[^lqr]: Perez, A., Platt, R., Konidaris, G., Kaelbling, L., & Lozano-Pérez, T. (2012). "LQR-RRT\*: Optimal Sampling-Based Motion Planning with Automatically Derived Extension Heuristics." *IEEE International Conference on Robotics and Automation (ICRA)*, 2537–2542. [doi:10.1109/ICRA.2012.6225177](https://doi.org/10.1109/ICRA.2012.6225177) · [PDF](https://lis.csail.mit.edu/pubs/perez-icra12.pdf)

[^webb]: Webb, D. J., & van den Berg, J. (2013). "Kinodynamic RRT\*: Asymptotically Optimal Motion Planning for Robots with Linear Dynamics." *IEEE International Conference on Robotics and Automation (ICRA)*, 5054–5061. [doi:10.1109/ICRA.2013.6631299](https://doi.org/10.1109/ICRA.2013.6631299) · [PDF (arXiv)](https://arxiv.org/abs/1205.5088)
