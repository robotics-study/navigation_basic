# 맵 파일 포맷

언어 공용 계약. C++/Python 의 `maps/` 로더는 이 문서를 기준으로 구현하며, 포맷 변경은 이 문서를 먼저 갱신한 뒤 양 언어에 반영한다.

공통 규칙:
- 모든 맵 파일은 최상위에 `type` 필드를 가진다: `occupancy_grid | graph | topology | continuous`. 로더는 이 필드로 맵 타입을 판별한다 (확장자/디렉토리에 의존하지 않는다).
- world 좌표는 미터 단위 float `(x, y)`, 각도는 라디안. grid 인덱스는 `(row, col)` int이며 `row 0 = 이미지 최상단`.
- 경로 참조(이미지 등)는 맵 파일 기준 상대 경로.

## occupancy_grid (`maps/grid/`)

ROS map_server 스타일. yaml + 그레이스케일 이미지(pgm/png).

```yaml
type: occupancy_grid
image: maze01.pgm          # 0(검정)=occupied, 255(흰색)=free
resolution: 0.05           # meters / pixel
origin: [0.0, 0.0, 0.0]    # 이미지 좌하단 픽셀의 world pose [x, y, theta]
occupied_thresh: 0.65      # (1 - pixel/255) >= 이 값 → occupied
free_thresh: 0.196         # (1 - pixel/255) <= 이 값 → free (사이 값은 unknown = 통행 불가 취급)
```

- capability: `DiscreteSpace`(4/8-connected), `SamplingSpace`, `ObstacleQuery`.

## graph (`maps/graph/`)

명시적 가중 그래프 (roadmap, 도로망 등).

```yaml
type: graph
directed: false
nodes:
  - { id: n1, pos: [1.0, 2.0] }   # pos 는 시각화/휴리스틱용 world 좌표
  - { id: n2, pos: [4.0, 2.0] }
edges:
  - { from: n1, to: n2 }           # cost 생략 시 pos 간 euclidean distance
  - { from: n2, to: n3, cost: 7.5 }
```

- capability: `DiscreteSpace`. 상태는 노드 id 문자열, 휴리스틱은 `pos` 간 euclidean.

## topology (`maps/topology/`)

장소(place) + 연결성 중심의 topological map. graph 와 달리 노드가 의미 단위(방, 교차로)이고 edge 에 semantic label 을 가진다.

```yaml
type: topology
places:
  - { id: kitchen, name: "Kitchen", pos: [2.0, 3.0] }   # pos 는 선택 (시각화용)
  - { id: hallway, name: "Hallway" }
connections:
  - { from: kitchen, to: hallway, cost: 1.0, label: door }
  - { from: hallway, to: lobby, cost: 3.0, label: corridor }
```

- 무방향. `cost` 필수 (기하 정보가 없을 수 있으므로 euclidean fallback 없음).
- capability: `DiscreteSpace`. 휴리스틱은 `pos` 가 전 노드에 있으면 euclidean, 아니면 0 (Dijkstra 로 퇴화).

## continuous (`maps/continuous/`)

기하 장애물 리스트. sampling 기반 planner / local planner 용.

```yaml
type: continuous
bounds: { x: [0.0, 10.0], y: [0.0, 10.0] }
obstacles:
  - { shape: circle, center: [3.0, 4.0], radius: 0.5 }
  - { shape: rectangle, center: [7.0, 2.0], size: [2.0, 1.0], theta: 0.0 }
  - { shape: polygon, vertices: [[1.0, 1.0], [2.0, 1.0], [1.5, 2.0]] }
```

- capability: `SamplingSpace`, `ObstacleQuery`. `DiscreteSpace` 는 격자화 adapter(해상도 파라미터) 경유로만 제공.

## scenario (`maps/scenarios/`)

맵 위에서 실행할 문제 정의. 단일 agent 는 `start`/`goal`, multi-agent 는 `agents` 를 쓴다 (둘 중 하나만).

```yaml
map: ../grid/maze01.yaml     # 시나리오 파일 기준 상대 경로
start: [0.5, 0.5]            # grid/continuous: world 좌표. graph/topology: 노드 id
goal: [9.5, 9.5]
# --- SE(2) heading (선택, kinodynamic planner 용) ---
start_theta: 0.0            # start 방향 (라디안, world). 생략 시 0.0 (하위호환)
goal_theta: 0.0            # goal 방향 (라디안, world). 생략 시 0.0
# --- reference path (선택, 추종 계열 local planner 용) ---
# reference_path:
#   - [0.5, 0.5]
#   - [2.0, 3.0]
#   - [9.5, 9.5]
# --- multi-agent 형식 ---
# agents:
#   - { start: [0.5, 0.5], goal: [9.5, 9.5] }
#   - { start: [9.5, 0.5], goal: [0.5, 9.5] }
```

- `start_theta`/`goal_theta` 는 선택. Hybrid A\* 같은 SE(2) kinodynamic planner 만 사용하며, discrete/sampling planner 는 무시한다. 생략 시 heading 0.0 으로 로드된다.
- `reference_path` 는 선택: world 좌표 웨이포인트 리스트 `[[x, y], ...]`. Pure Pursuit 같은 추종(tracking) 계열 local planner 만 사용하며, 생략 시 참조 경로 없음(goal-seek 전용)으로 로드된다.
