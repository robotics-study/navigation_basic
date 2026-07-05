---
title: 아키텍처
layout: default
parent: 한국어
nav_order: 2
---

[🇰🇷 한국어](architecture.md) | [🇬🇧 English](../en/architecture.md)

# 아키텍처
{: .no_toc }

1. TOC
{:toc}

## 저장소 구조

```
.
├── spec/                        # 언어 공용 계약 (구현보다 우선하는 single source of truth)
│   ├── trace_schema.json        #   step-by-step trace 이벤트 JSON Schema
│   ├── param_schema.json        #   알고리즘 파라미터 선언(name/type/range/default) 스키마
│   └── map_formats.md           #   맵 파일 포맷 정의 (grid / graph / topology / continuous)
├── maps/                        # 공용 벤치마크 맵 + start/goal 시나리오
├── configs/                     # 알고리즘별 파라미터 yaml (언어 공용)
├── cpp/                         # C++20 구현 (include/navigation + src + demos + GoogleTest)
├── python/                      # Python 구현 (navigation 패키지 + demos + pytest)
└── tools/                       # viz(trace 재생기) + bench(매트릭스 러너) — Python
```

C++ 과 Python 은 **같은 설계를 각자 idiomatic 하게** 구현한 독립 트리다. 클래스/메서드 개념 이름,
파라미터 이름, trace 이벤트가 동일하므로 한쪽을 읽으면 다른 쪽도 읽힌다. 언어 간 공유물(trace schema,
param yaml, map 데이터, 시나리오)은 반드시 `spec/`, `configs/`, `maps/` 에 두고 양쪽에서 로드한다.

## 의존 방향

```
demos ──▶ algorithms ──▶ core ◀── maps ◀── tools/viz·bench
                          ▲                     │
                          └── spec (포맷 계약) ──┘
```

- `core` 는 stdlib(+ numpy / Eigen)만 의존한다. 알고리즘·맵 모듈을 알지 못한다.
- `maps` 는 `core` 만 의존한다.
- 알고리즘 모듈은 `core` 의 추상 인터페이스에만 의존한다. **구체 맵 클래스 직접 참조 금지**, 알고리즘 모듈 간 상호 의존 금지.
- `tools/viz`, `tools/bench` 는 spec 포맷과 `core`/`maps` 로더에만 의존한다. 알고리즘 내부 상태 접근 금지 — 시각화에 필요한 모든 정보는 trace 이벤트로 방출되어야 한다.
- `demos` 는 최상위 조립 계층: 알고리즘 + maps + configs 를 묶기만 한다. 로직 금지.

## Capability 모델

알고리즘은 구체 맵 타입이 아니라 **capability 인터페이스**를 요구한다. 맵 타입은 지원 가능한
capability 를 adapter 로 구현하고, 하나의 맵을 여러 알고리즘에 붙여 테스트할 수 있다.

| capability | 핵심 메서드 | 요구 알고리즘 |
|---|---|---|
| `DiscreteSpace` | `neighbors(state) -> [(state, cost)]`, `heuristic(a, b)` | BFS, Dijkstra, A*, CBS low-level |
| `SamplingSpace` | `sample()`, `is_state_valid(s)`, `is_motion_valid(a, b)`, `distance(a, b)`, `steer(a, b, eta)` | RRT 계열 |
| `ObstacleQuery` | `is_collision(footprint, pose)`, `distance_to_nearest(p)` | DWA, VFH, MPC 등 local planner |

맵 타입 × capability 지원 매트릭스:

| 맵 타입 | DiscreteSpace | SamplingSpace | ObstacleQuery |
|---|:---:|:---:|:---:|
| `OccupancyGrid2D` [^elfes] | O (4/8-connected) | O | O |
| `GraphMap` | O | X | X |
| `TopologyMap` | O (semantic edge cost) | X | X |
| `ContinuousMap` | X (격자화 adapter 경유 O) | O | O |

Planner 는 `required_capabilities()` 를 선언하고, bench runner / demo 는 실행 전
`map.supports(capability)` 로 호환성을 검사한다. 비호환 조합은 에러가 아니라 "incompatible" 로
리포트에 기록된다.

`OccupancyGrid2D` 의 `DiscreteSpace` adapter 는 8-connected 이동(직교 1.0 / 대각 √2 × resolution)을
제공하며, 대각 이동은 양쪽 직교 셀이 모두 free 일 때만 허용한다 (corner cutting 방지). heuristic 은
octile distance 로, 이 이동 모델에서 admissible 하다.

## Trace — step-by-step 시각화의 계약

알고리즘은 탐색 진행을 `TraceRecorder` 로 방출한다. 이벤트 목록/필드는 `spec/trace_schema.json` 이
정의하는 언어 공용 계약이다.

| 이벤트 | 의미 | 방출 알고리즘 |
|---|---|---|
| `planning_started` | 실행 시작 (algorithm, map, params 스냅샷) | 전체 |
| `node_expanded` | 노드를 닫힌 집합으로 확정 | BFS, Dijkstra, A* |
| `edge_added` | 트리/탐색 그래프에 간선 추가 | 전체 |
| `sample_drawn` | 무작위 샘플 추출 | RRT 계열 |
| `rewire` | 기존 노드의 부모 교체 (비용 개선) | RRT*, Fast-RRT |
| `candidate_evaluated` | 후보 평가 (local planner 용) | DWA 등 (계획) |
| `constraint_added` / `conflict_found` | 제약/충돌 (multi-agent 용) | CBS (계획) |
| `path_found` | (개선된) 경로 발견 | 전체 |
| `planning_finished` | 종료 + 공통 metric | 전체 |

- trace 는 JSON Lines 파일로 저장되고 `tools/viz/replay.py` 가 맵 위에 재생한다. C++ 데모도 같은
  포맷을 출력하므로 **시각화 코드는 하나**다.
- trace 방출은 기본 off(성능 측정 시) / demo·viz 시 on. hot loop 에서 recorder 가 null 이면
  zero-cost 다.
- 상태 표현: 연속 공간은 `[x, y(, theta)]` world 좌표(float), grid 는 `[row, col]`(int),
  graph/topology 는 노드 id 문자열. world ↔ grid 변환은 맵 클래스만 담당한다.

## 파라미터 추상화

- 각 알고리즘은 자신의 `ParamSet` 을 선언한다: 이름, 타입, 기본값, 유효 범위. 선언 형식은
  `spec/param_schema.json` 을 따른다.
- 값은 `configs/<category>/<algorithm>.yaml` 에서 로드하고 로드 시점에 선언 기반 검증(범위 밖 → 에러)을
  수행한다. 코드에 매직 넘버로 파라미터를 심지 않는다.
- 같은 yaml 을 C++/Python 양쪽이 그대로 읽는다.

## References

[^elfes]: Elfes, A. (1989). "Using occupancy grids for mobile robot perception and navigation." *Computer*, 22(6), 46–57. [doi:10.1109/2.30720](https://doi.org/10.1109/2.30720)
