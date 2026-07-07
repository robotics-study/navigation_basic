---
title: SST
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 12
---

[🇰🇷 한국어](sst.md) | [🇬🇧 English](../../en/algorithms/sst.md)

# SST / SST* (Stable Sparse RRT)
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | sampling-based, kinodynamic, single-query, anytime |
| 요구 capability | `SamplingSpace` |
| 완전성 | probabilistically complete (forward propagation 하에서 점근적) |
| 최적성 | SST: near-optimal (bounded); **SST\***: asymptotically optimal (반경 축소) |
| 복잡도 | 반복당: sparse active 집합에 대한 BestNear + forward propagation 1회 + witness 질의 |
| 원 논문 | Li, Littlefield & Bekris (2016) [^li] |

1. TOC
{:toc}

## 배경

[RRT\*](rrt_star.md) 같은 optimal sampling planner 는 **steering function** — 임의의 두 상태를 잇는
two-point boundary-value problem(BVP)의 정확한 해 — 에 의존한다. 동역학이 복잡한 시스템에서는 이 BVP
가 비싸거나 닫힌 해가 없다. Li, Littlefield & Bekris[^li] 는 트리를 **sparse** 하게 유지하기만 하면
**forward propagation 만으로** (제어를 앞으로 적분하고 충돌만 검사, steering 없이) 점근적 (near-)optimal
을 유지할 수 있음을 보였다.

SST 는 신중히 고른 노드에서 **랜덤 제어를 랜덤 시간 동안** propagate 하여 트리를 키운다. "stable" +
"sparse" 를 만드는 두 아이디어:

1. **BestNear 선택** — 단일 nearest 노드가 아니라 sample 주변 반경 `delta_bn` 볼 안의 **최소 cost
   active 노드**를 확장 부모로 고른다. 값싼 노드를 확장하니 incumbent cost 가 내려간다 (stability).
2. **Witness-set pruning** — 서로 최소 `delta_s` 떨어진 **witness** 점들이 각각 단일 **active
   representative**(그 `delta_s` 볼 안 최소 cost 노드)를 유지한다. 새 노드는 자신의 witness
   representative 를 *개선할 때만* 유지되고, 밀려난 representative 는 비활성화되어 leaf 가 된 비활성
   조상들과 함께 pruning 된다. 반복 수와 무관하게 **active 노드 수를 유한하게 묶는다** (sparsity).

**SST\*** 는 `delta_bn` 과 `delta_s` 를 반복에 따라 축소(schedule)해 sparse 트리가 계속 정제되게 하여
**asymptotic optimality** 를 회복한다. 같은 클래스가 둘 다 제공한다: `sst_star = false` 는 SST(고정
반경), `sst_star = true` 는 SST\*.

## 동작 원리

`maze01` — unicycle(상태 (x, y, θ), 제어 (v, ω))이 랜덤 arc 를 forward-propagate 한다. 빽빽한 초록
구름은 뽑힌 sample 이고, 그 아래 성긴 하늘색 트리가 witness 가 묶어 둔 **active** 집합이다. 최종 경로는
매끈한, 동역학적으로 실현 가능한 궤적이다 (직선 steer 경로가 아니다).

![SST on maze01](../../assets/sst/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/sst/maze01_snap_02.png) | ![mid](../../assets/sst/maze01_snap_05.png) | ![final](../../assets/sst/maze01_final.png) |

`open01` 최종 결과 — 직선 최적에 가까운 매끈한 곡선 궤적:

![SST on open01](../../assets/sst/open01_final.png)

```
SST(start, goal):
    V_active ← {start};  S ← {witness(start) → rep=start}          # witness 집합
    for i in 1..max_iterations:                                     # anytime
        s_sample ← (goal with prob. goal_bias) else sample()
        x_sel    ← BestNear(V_active, s_sample, delta_bn)           # 볼 안 최소 cost (없으면 nearest)
        x_new, arc ← MonteCarloProp(x_sel)                          # 랜덤 (v, ω) 를 랜덤 시간 propagate
        if arc collides: continue                                   # forward-propagation 만, steer 없음
        w ← nearest_witness(x_new)
        if dist(x_new, w) > delta_s: S.add(witness(x_new) → rep=∅)  # 새 witness
        x_peer ← rep(witness of x_new)
        if x_peer = ∅ or cost(x_new) < cost(x_peer):                # locally best?
            V_active.add(x_new);  rep(witness) ← x_new
            if x_peer ≠ ∅: deactivate x_peer; prune inactive leaf ancestors   # 트리를 묶는다
            if dist(x_new, goal) ≤ goal_tolerance:
                best ← min(best, cost(x_new))                        # 해를 갱신하며 계속
    return best
```

측정치 (seed = 1, 30,000 iterations, trace on):

| map | 언어 | path cost | active nodes | nodes added | runtime |
|---|---|---|---|---|---|
| maze01 | Python | 13.683 | 198 | 1,286 | 1.20 s |
| open01 | Python | 12.005 | 186 | 1,226 | 1.19 s |

**active nodes**(198 / 186)와 **nodes added**(1,286 / 1,226)의 격차가 곧 witness pruning 의 효과다.
받아들인 노드 다섯 중 넷 이상이 나중에 밀려나 pruning 되므로 active 트리는 작게 유지되고, 순수 RRT
라면 모든 노드를 보유했을 것이다. `open01` 의 cost(12.005)는 직선 하한(≈12.02)의 ~0.1% 이내다. C++
포트는 알고리즘을 그대로 미러링하지만 독립적인 `std::mt19937` 스트림을 쓰므로 정확한 수치는 다르다
(CMake 로 빌드해 재현).

재현:

```bash
python python/demos/demo_sst.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/sst.yaml --trace out/sst.jsonl
python tools/viz/replay.py out/sst.jsonl --gif out/sst.gif
```

## 성질

- **Steering / BVP solver 불필요.** SST 는 forward propagation + 충돌 검사만 필요하므로 두 상태를
  정확히 잇기 어려운 시스템에도 적용된다[^li]. 여기 모델은 unicycle 이며, 동역학은 planner 가 소유하고
  맵은 state / motion validity(`SamplingSpace`)만 답한다.
- **Sparsity 가 유한하다.** witness 집합이 active 노드 수를 `delta_s` 로 분리된 witness 개수
  (≈ 자유공간 넓이 / `delta_s`²)로 묶으며, 이는 `max_iterations` 와 *무관*하다.
- **Anytime, 단조 incumbent.** BestNear + "representative 를 개선할 때만 유지" 규칙이 각 representative
  의 cost 를 비증가로 만들어, best 해가 나빠지지 않는다.
- **최적성.** 고정 반경(SST)은 `delta_bn` / `delta_s` 에 묶인 near-optimal 해를, 축소 schedule
  (SST\*)은 asymptotic optimality 를 준다[^li].

## 파라미터

| 이름 | 타입 | 기본값 | 범위 | 설명 |
|---|---|---|---|---|
| `max_iterations` | int | 30000 | [1, 2000000] | 반복 예산 (anytime — 소진 시 현재 best 반환) |
| `goal_bias` | float | 0.1 | [0.0, 1.0] | goal 을 직접 sample 할 확률 |
| `goal_tolerance` | float | 0.6 | [0.0, 100.0] | goal 도달 판정 반경 (m, 위치 기준) |
| `delta_bn` | float | 1.2 | [0.01, 100.0] | BestNear 반경 δ_BN (m) — 최소 cost active 노드 볼 |
| `delta_s` | float | 0.5 | [0.01, 100.0] | witness / sparsification 반경 δ_s (m) — active 노드 밀도 상한 |
| `max_velocity` | float | 1.5 | [0.01, 100.0] | 전진 속도 v 상한 (m/s); [0, v_max] 에서 샘플 |
| `max_omega` | float | 1.5 | [0.0, 100.0] | 각속도 ω 상한 (rad/s); [−ω_max, ω_max] 에서 샘플 |
| `prop_duration_min` | float | 0.2 | [0.001, 100.0] | propagation 지속 시간 하한 (s) |
| `prop_duration_max` | float | 0.8 | [0.001, 100.0] | propagation 지속 시간 상한 (s) |
| `sst_star` | bool | false | — | true → SST\* (δ_BN / δ_s 를 반복에 따라 축소, asymptotic optimality) |
| `seed` | int | 1 | [0, 2³¹−1] | 난수 시드 (재현성) |

## 방출 trace 이벤트

`planning_started` → (`sample_drawn`, `edge_added`*, `rewire`?)* → `path_found`* → `planning_finished`

`edge_added` 는 arc chord 마다 방출되어 곡선 propagation 이 매끈하게 렌더링된다. `rewire` 는 witness
representative 가 더 싼 노드로 옮겨감(밀려난 branch 는 pruning)을 표시해, 재생 시 트리가 sparse 하게
유지되는 모습을 보여준다. `path_found` 는 여러 번 방출될 수 있다 (incumbent 개선 시마다).

## References

[^li]: Li, Y., Littlefield, Z., & Bekris, K. E. (2016). "Asymptotically optimal sampling-based kinodynamic planning." *The International Journal of Robotics Research*, 35(5), 528–564. [doi:10.1177/0278364915614386](https://doi.org/10.1177/0278364915614386) · [PDF (arXiv)](https://arxiv.org/abs/1407.2896)
