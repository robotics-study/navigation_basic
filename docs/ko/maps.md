---
title: 맵 표현
layout: default
---

# 맵 표현 (Map representations)

알고리즘은 구체 맵 타입이 아니라 **capability 인터페이스**를 요구한다. 맵 타입은 지원 가능한
capability 를 adapter 로 구현하고, 하나의 맵을 여러 알고리즘에 붙여 테스트할 수 있다. 새 맵 타입을
추가해도 알고리즘 코드는 바뀌지 않는다 (OCP). 포맷의 정식 정의는 저장소 `spec/map_formats.md` 가
single source of truth 이고, C++/Python 로더가 이를 그대로 읽는다.

공통 규칙: 모든 맵 파일은 최상위 `type` 필드로 판별한다 (`occupancy_grid | graph | topology | continuous`,
확장자·디렉토리에 의존하지 않음). world 좌표는 미터 float `(x, y)`·각도는 라디안, grid 인덱스는
`(row, col)` int (`row 0 = 이미지 최상단`).

## Capability 모델

| capability | 핵심 메서드 | 요구 알고리즘 |
|---|---|---|
| `DiscreteSpace` | `neighbors(state)`, `heuristic(a, b)` | BFS, Dijkstra, A*, CBS low-level |
| `SamplingSpace` | `sample()`, `is_state_valid`, `is_motion_valid`, `distance`, `steer` | RRT 계열 |
| `ObstacleQuery` | `is_collision(footprint, pose)`, `distance_to_nearest(p)` | DWA, VFH, MPC |

맵 타입 × capability 지원 매트릭스:

| 맵 타입 | DiscreteSpace | SamplingSpace | ObstacleQuery | 상태 |
|---|:---:|:---:|:---:|:---:|
| OccupancyGrid2D | O (4/8-connected) | O | O | ✅ 구현 |
| GraphMap | O | X | X | 예정 |
| TopologyMap | O (semantic edge) | X | X | 예정 |
| ContinuousMap | 격자화 adapter 경유 O | O | O | 예정 |

Planner 는 `required_capabilities()` 를 선언하고, bench/demo 는 실행 전 `map.supports(cap)` 로
호환성을 검사한다. 비호환 조합(예: `GraphMap` × RRT)은 에러가 아니라 "incompatible" 로 리포트된다.
설계 배경은 [아키텍처](architecture.md) 참고.

## OccupancyGrid2D — occupancy grid ✅

ROS `map_server` 스타일. yaml + 그레이스케일 이미지(pgm/png). `0(검정)=occupied`, `255(흰색)=free`.

```yaml
type: occupancy_grid
image: maze01.pgm
resolution: 0.05           # meters / pixel
origin: [0.0, 0.0, 0.0]    # 이미지 좌하단 픽셀의 world pose [x, y, theta]
occupied_thresh: 0.65      # (1 - pixel/255) >= 이 값 → occupied
free_thresh: 0.196         # (1 - pixel/255) <= 이 값 → free (사이 값 = unknown = 통행 불가)
```

- **좌표계**: world `(x, y)` float(미터) ↔ grid `(row, col)` int. 변환은 맵 클래스만 담당한다.
- **DiscreteSpace**: 8-connected 이동(직교 1.0, 대각 √2 × resolution). 대각은 양쪽 직교 셀이 free 일
  때만 허용해 corner cutting 을 막는다. heuristic 은 octile distance 로 이 이동 모델에서 admissible.
- **SamplingSpace / ObstacleQuery**: 셀 중심이 아닌 연속 world 좌표를 샘플하고, 선분을 resolution
  간격으로 보간해 충돌을 검사한다.
- 세 capability 를 모두 제공하므로 [구현된 전 알고리즘](algorithms/index.md)을 이 맵 위에서 돌릴 수
  있다 — 벤치마크 맵 `maze01`(좁은 통로 미로)·`open01`(열린 필드)이 이 타입이다.

## GraphMap — 명시적 가중 그래프 (예정)

roadmap·도로망처럼 노드와 간선이 명시된 그래프. 상태는 노드 id 문자열이다.

```yaml
type: graph
directed: false
nodes:
  - { id: n1, pos: [1.0, 2.0] }   # pos = 시각화/휴리스틱용 world 좌표
edges:
  - { from: n1, to: n2 }          # cost 생략 시 pos 간 euclidean
  - { from: n2, to: n3, cost: 7.5 }
```

- **DiscreteSpace** 만 제공 (휴리스틱 = `pos` 간 euclidean). sampling·obstacle 계열은 미지원 →
  RRT/local planner 와 incompatible.

## TopologyMap — topological map (예정)

장소(place)와 연결성 중심. graph 와 달리 노드가 의미 단위(방·교차로)이고 간선에 semantic label 을 갖는다.

```yaml
type: topology
places:
  - { id: kitchen, name: "Kitchen", pos: [2.0, 3.0] }   # pos 선택 (시각화용)
  - { id: hallway, name: "Hallway" }
connections:
  - { from: kitchen, to: hallway, cost: 1.0, label: door }
  - { from: hallway, to: lobby, cost: 3.0, label: corridor }
```

- 무방향, `cost` 필수 (기하 정보가 없을 수 있어 euclidean fallback 없음). **DiscreteSpace** 제공.
  `pos` 가 전 노드에 있으면 euclidean heuristic, 아니면 0 (Dijkstra 로 퇴화).

## ContinuousMap — 기하 장애물 (예정)

원/사각형/다각형 장애물 리스트. sampling 기반 planner·local planner 용.

```yaml
type: continuous
bounds: { x: [0.0, 10.0], y: [0.0, 10.0] }
obstacles:
  - { shape: circle, center: [3.0, 4.0], radius: 0.5 }
  - { shape: rectangle, center: [7.0, 2.0], size: [2.0, 1.0], theta: 0.0 }
  - { shape: polygon, vertices: [[1.0, 1.0], [2.0, 1.0], [1.5, 2.0]] }
```

- **SamplingSpace / ObstacleQuery** 제공. **DiscreteSpace** 는 격자화 adapter(해상도 파라미터)
  경유로만 제공한다.

## Scenario — 문제 정의

맵 위에서 실제로 풀 문제. 단일 agent 는 `start`/`goal`, multi-agent 는 `agents` 를 쓴다 (둘 중 하나만).

```yaml
map: ../grid/maze01.yaml      # 시나리오 파일 기준 상대 경로
start: [0.5, 0.5]            # grid/continuous: world 좌표 · graph/topology: 노드 id
goal: [9.5, 9.5]
# --- multi-agent ---
# agents:
#   - { start: [0.5, 0.5], goal: [9.5, 9.5] }
#   - { start: [9.5, 0.5], goal: [0.5, 9.5] }
```
