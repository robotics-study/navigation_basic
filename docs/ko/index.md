---
title: 한국어
layout: default
nav_order: 2
has_children: true
permalink: /ko/
---

[🇰🇷 한국어](index.md) | [🇬🇧 English](../en/index.md)

# navigation study 개요
{: .fs-8 }

로봇 navigation planning 알고리즘 스터디 — 같은 추상화 설계를 **C++20 / Python 3.10+ 으로 독립 이중 구현**하고,
언어 공용 trace 포맷 기반의 **step-by-step 시각화**와 **(map × algorithm) 매트릭스 벤치마크**를 제공한다.
{: .fs-5 .fw-300 }

## 한눈에 보기

같은 미로(`maze01`, 20×20 occupancy grid)를 세 알고리즘이 푸는 과정.
색은 시간 순서를 인코딩한다 — expanded 노드는 노랑→진갈색, 샘플링 트리는 하늘색 계열, 최종 경로는 보라(start)→빨강(goal).

| A* (informed search) | RRT* (asymptotically optimal) | Fast-RRT (2021) |
|:---:|:---:|:---:|
| ![A* demo](../assets/astar/maze01.gif) | ![RRT* demo](../assets/rrt_star/maze01.gif) | ![Fast-RRT demo](../assets/fast_rrt/maze01.gif) |
| 108 nodes expanded | 8,000 samples + rewire | Fast-Sampling + shortcut |

## 핵심 설계

- **공통 추상화** — 모든 planner 는 `GlobalPlanner` / `LocalPlanner` / `MultiAgentPlanner` 추상 클래스를 상속하고, 구체 맵이 아닌 **capability 인터페이스**(`DiscreteSpace`, `SamplingSpace`, `ObstacleQuery`)만 요구한다. 새 맵 타입을 추가해도 알고리즘 코드는 바뀌지 않는다 (OCP).
- **언어 미러링** — C++ / Python 이 같은 설계·같은 파라미터·같은 trace 이벤트를 각자 idiomatic 하게 구현한다. trace / param / map 포맷은 저장소 `spec/` 아래 언어 공용 계약이 single source of truth.
- **Trace 기반 시각화** — 알고리즘은 탐색 진행을 JSON Lines 이벤트로 방출하고, 시각화 도구(`tools/viz/replay.py`)는 **언어당 하나가 아니라 하나**다. C++ 데모의 trace 도 같은 도구로 재생한다.
- **벤치마크 매트릭스** — `tools/bench/run_matrix.py` 가 (scenario × algorithm) 조합을 실행해 성공 여부, runtime, path cost, expanded/samples 를 수집한다.

## 구현 현황

| 카테고리 | 구현 완료 | 계획 |
|---|---|---|
| **global_planning** | [BFS](algorithms/bfs.md) · [Dijkstra](algorithms/dijkstra.md) · [A*](algorithms/astar.md) · [RRT](algorithms/rrt.md) · [RRT*](algorithms/rrt_star.md) · [Fast-RRT](algorithms/fast_rrt.md) | RRT-Connect, Informed RRT* |
| **local_planning** | — | DWA, Pure Pursuit, VFH, MPC |
| **multi_agent** | — | Prioritized A*, Joint-space A*, CBS |

구현된 6종은 전부 **C++ / Python parity** 를 만족한다 — 같은 맵·같은 파라미터에서 discrete 계열은 동일한 결과를,
sampling 계열은 seed 기반으로 통계적으로 동등한 결과를 낸다. 세부 수치는 [벤치마크](benchmarks.md) 참고.

## 빠른 시작

```bash
# Python (>= 3.10)
cd python && pip install -e ".[dev,viz]" && cd ..
pytest python/tests                                  # 79 tests

# C++ (C++20, CMake >= 3.20)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build                           # 45 tests

# 데모 실행 (두 언어 CLI 인자 동일) + 시각화
python python/demos/demo_astar.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/astar.yaml --trace out/astar.jsonl
python tools/viz/replay.py out/astar.jsonl --gif out/astar.gif
```

## 문서 구성

| 페이지 | 내용 |
|---|---|
| [알고리즘](algorithms/index.md) | 알고리즘별 상세 — 이론, pseudocode, 성질(완전성·최적성·복잡도), 파라미터, demo GIF/PNG, 원 논문 각주 |
| [아키텍처](architecture.md) | 저장소 구조, 의존 방향, capability 모델, trace 계약, 파라미터 추상화 |
| [벤치마크](benchmarks.md) | (map × algorithm) 매트릭스 실측 결과 + C++/Python 비교 |
| [참고 문헌](references.md) | 전체 참고 문헌 |
