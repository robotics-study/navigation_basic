# navigation

로봇 navigation 알고리즘 구현체 + demo 모음. C++ / Python 독립 이중 구현.

## 프로젝트 개요

세 카테고리의 planning 알고리즘을 공통 추상화 위에 구현한다:

| 카테고리 | 알고리즘 (계획) | 베이스 클래스 |
|---|---|---|
| global_planning | Dijkstra, A*, RRT, RRT-Connect, RRT*, Informed RRT* | `GlobalPlanner` |
| local_planning | DWA, Pure Pursuit, VFH, MPC | `LocalPlanner` |
| multi_agent | Prioritized A*, Joint-space A*, CBS | `MultiAgentPlanner` |

모든 알고리즘은 추상 클래스 기반으로 다음 세 가지가 자동으로 성립해야 한다:
1. **Performance estimate** — 공통 metric(runtime, path length/cost, expanded nodes, success rate)을 benchmark runner가 수집.
2. **Step-by-step visualization** — 알고리즘이 방출하는 trace 이벤트(공용 JSON 포맷)를 시각화 도구가 재생.
3. **Demo** — 맵 + 파라미터 파일만 지정하면 실행되는 데모.

## 저장소 구조

```
.
├── CLAUDE.md
├── spec/                        # 언어 공용 계약 (구현보다 우선하는 single source of truth)
│   ├── trace_schema.json        #   step-by-step trace 이벤트 JSON Schema
│   ├── param_schema.json        #   알고리즘 파라미터 선언(name/type/range/default) 스키마
│   └── map_formats.md           #   맵 파일 포맷 정의 (grid / graph / topology / continuous)
├── maps/                        # 공용 벤치마크 맵 데이터
│   ├── grid/                    #   occupancy grid (ROS 스타일 yaml + pgm/png)
│   ├── graph/                   #   명시적 그래프 (nodes/edges yaml)
│   ├── topology/                #   topological map (place + connectivity yaml)
│   ├── continuous/              #   기하 장애물 리스트 (yaml)
│   └── scenarios/               #   start/goal/agents 시나리오 (yaml, 맵 참조)
├── configs/                     # 알고리즘별 파라미터 yaml (언어 공용)
│   ├── global_planning/         #   astar.yaml, rrt_star.yaml, ...
│   ├── local_planning/
│   └── multi_agent/
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/navigation/
│   │   ├── core/                # planner.hpp, params.hpp, trace.hpp, types.hpp, capabilities.hpp
│   │   ├── maps/                # occupancy_grid.hpp, graph_map.hpp, topology_map.hpp, continuous_map.hpp, loader.hpp
│   │   ├── global_planning/
│   │   ├── local_planning/
│   │   └── multi_agent/
│   ├── src/                     # include/와 동일 구조의 구현
│   ├── demos/                   # demo_astar.cpp 등 — 실행 시 trace 파일 출력
│   └── tests/                   # GoogleTest
├── python/
│   ├── pyproject.toml
│   ├── navigation/
│   │   ├── core/                # planner.py, params.py, trace.py, types.py, capabilities.py
│   │   ├── maps/                # cpp include/navigation/maps/ 와 1:1 미러
│   │   ├── global_planning/
│   │   ├── local_planning/
│   │   └── multi_agent/
│   ├── demos/
│   └── tests/                   # pytest
└── tools/                       # Python. navigation 패키지에 의존 (설치 후 사용)
    ├── viz/                     # trace 재생기: replay.py (matplotlib step-by-step / 애니메이션 저장)
    └── bench/                   # matrix runner: (map × algorithm × params) 조합 실행 + 리포트
```

## 아키텍처 원칙

### 의존 방향 (위반은 리뷰 Critical)
- `core` 는 stdlib(+ Eigen / numpy)만 의존한다. 알고리즘·맵 모듈을 알지 못한다.
- `maps` 는 `core` 만 의존한다.
- 알고리즘 모듈(`global_planning`, `local_planning`, `multi_agent`)은 `core` 의 추상 인터페이스에만 의존한다. **구체 맵 클래스 직접 참조 금지**, 알고리즘 모듈 간 상호 의존 금지.
- `tools/viz`, `tools/bench` 는 trace/param/map 포맷(spec)과 `core`/`maps` 로더에만 의존한다. 알고리즘 내부 상태 접근 금지 — 시각화에 필요한 모든 정보는 trace 이벤트로 방출되어야 한다.
- `demos` 는 최상위 조립 계층: 알고리즘 + maps + configs 를 묶기만 한다. 로직 금지.

### 언어 미러링
- C++과 Python은 **같은 설계를 각자 idiomatic 하게** 구현한다. 클래스/메서드 개념 이름, 파라미터 이름, trace 이벤트는 동일해야 한다 (표기만 언어 컨벤션: C++ `snake_case` 멤버 / Python `snake_case`).
- 알고리즘 추가/변경은 원칙적으로 두 언어 동시 반영. 한쪽만 구현된 상태는 README parity 표에 명시하고 남겨두지 않는 것을 원칙으로 한다.
- 언어 간 공유물(trace schema, param yaml, map 데이터, 시나리오)은 반드시 `spec/`, `configs/`, `maps/` 에 두고 양쪽에서 로드한다. 언어 디렉토리 안에 복제 금지.

### 맵 추상화 — capability 모델
알고리즘은 구체 맵 타입이 아니라 **capability 인터페이스**를 요구한다. 맵 타입은 지원 가능한 capability 를 구현하고, 하나의 맵을 여러 알고리즘에 붙여 테스트할 수 있다.

| capability | 핵심 메서드 | 요구 알고리즘 |
|---|---|---|
| `DiscreteSpace` | `neighbors(state) -> [(state, cost)]`, `heuristic(a, b)` | Dijkstra, A*, CBS low-level |
| `SamplingSpace` | `sample()`, `is_state_valid(s)`, `is_motion_valid(a, b)`, `distance(a, b)`, `steer(a, b, eta)` | RRT 계열 |
| `ObstacleQuery` | `is_collision(footprint, pose)`, `distance_to_nearest(p)` | DWA, VFH, MPC 등 local planner |

맵 타입 × capability 지원 매트릭스 (adapter 로 구현):

| 맵 타입 | DiscreteSpace | SamplingSpace | ObstacleQuery |
|---|---|---|---|
| `OccupancyGrid2D` | O (4/8-connected) | O | O |
| `GraphMap` | O | X | X |
| `TopologyMap` | O (semantic edge cost) | X | X |
| `ContinuousMap` | X (격자화 adapter 경유 O) | O | O |

- Planner 는 `required_capabilities()` 를 선언하고, bench runner / demo 는 실행 전 `map.supports(capability)` 로 호환성을 검사한다. 비호환 조합은 에러가 아니라 "incompatible" 로 리포트에 기록한다.
- 새 맵 타입 추가 시 기존 알고리즘 코드는 수정되지 않아야 한다 (OCP).

### 파라미터 추상화
- 각 알고리즘은 자신의 `ParamSet` 을 선언한다: 파라미터 이름, 타입, 기본값, 유효 범위/제약. 선언 형식은 `spec/param_schema.json` 을 따른다.
- 값은 `configs/<category>/<algorithm>.yaml` 에서 로드하고 로드 시점에 선언 기반 검증(범위 밖 → 에러)을 수행한다. 코드에 매직 넘버로 파라미터를 심지 않는다.
- 같은 yaml 을 C++/Python 양쪽이 그대로 읽는다.

### Trace (step-by-step 시각화의 계약)
- 알고리즘은 탐색 진행을 `TraceRecorder` 로 방출한다: 예) `node_expanded`, `edge_added`, `sample_drawn`, `rewire`, `candidate_evaluated`, `path_found`, `constraint_added`(CBS). 이벤트 목록/필드는 `spec/trace_schema.json` 이 정의한다.
- trace 는 JSON Lines 파일로 저장되며 `tools/viz/replay.py` 가 맵 위에 step-by-step 재생한다. C++ 데모도 같은 포맷을 출력하므로 시각화 코드는 언어당 하나가 아니라 **하나**다.
- trace 방출은 기본 off(성능 측정 시) / demo·viz 시 on. hot loop 에서 recorder 가 null 이면 zero-cost 여야 한다.
- **데모 산출물 형식 (룰)**: 모든 알고리즘의 demo trace 는 `replay.py` 로 (1) 애니메이션 **GIF** (`--gif`, 탐색 진행 + 최종 경로) 와 (2) 탐색 중간 과정 **PNG 스냅샷** 세트 (`--snapshots`, 진행률 균등 분할) 로 렌더링 가능해야 한다. 산출물은 두 언어 데모 각각에 대해 `out/viz/<algo>/py/`, `out/viz/<algo>/cpp/` 아래에 둔다 (`out/` 은 gitignore — 커밋하지 않는다).

### Benchmark
- `tools/bench` 는 (map, scenario, algorithm, params) 조합 매트릭스를 실행하고 metric 을 수집한다: 성공 여부, wall time, path cost/length, expanded nodes / samples, (multi-agent) sum-of-costs·makespan.
- 동일 시나리오에 대한 C++ vs Python 비교도 이 러너로 수행한다 (각 언어의 CLI runner 를 subprocess 로 호출, 결과는 공용 JSON 으로 수집).

## 빌드 / 테스트 / 실행

```bash
# C++ (C++20, CMake ≥ 3.20, GoogleTest)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build

# Python (≥ 3.10)
cd python && pip install -e ".[dev]"
pytest python/tests

# Demo (예시 — 두 언어가 동일한 인자 형태를 갖는다)
./cpp/build/demos/demo_astar --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
    --params configs/global_planning/astar.yaml --trace out/astar.trace.jsonl
python python/demos/demo_astar.py --map ... --scenario ... --params ... --trace ...

# 시각화 / 벤치마크
python tools/viz/replay.py out/astar.trace.jsonl                                        # interactive 재생
python tools/viz/replay.py out/astar.trace.jsonl --gif out/viz/astar/astar.gif \
    --snapshots out/viz/astar/                                                          # GIF + 중간 과정 PNG
python tools/bench/run_matrix.py --maps maps/ --algos global_planning --out out/report.md
```

## 새 알고리즘 추가 체크리스트

1. `configs/<category>/<algo>.yaml` 에 파라미터 선언 + 기본값 작성.
2. 해당 카테고리 베이스 클래스를 상속해 C++/Python 양쪽 구현 (`required_capabilities()` 선언 포함).
3. 탐색 단계마다 trace 이벤트 방출. 새 이벤트 타입이 필요하면 `spec/trace_schema.json` 먼저 갱신.
4. 두 언어 각각 demo 추가.
5. 단위 테스트: 최소 (a) 알려진 맵에서 최적/유효 경로 검증, (b) 경로 없음 케이스, (c) 파라미터 검증 실패 케이스.
6. `tools/bench` 매트릭스에서 호환 맵 전체에 대해 1회 실행 확인.
7. demo trace 를 `replay.py --gif` / `--snapshots` 로 렌더링해 GIF 애니메이션 + 중간 과정 PNG 가 정상 생성되는지 확인.
8. README parity 표 갱신.

새 **맵 타입** 추가 시: `spec/map_formats.md` 에 포맷 정의 → 양 언어 `maps/` 에 로더 + capability adapter 구현 → `maps/` 에 샘플 데이터 → capability 매트릭스 표 갱신. 알고리즘 코드는 수정하지 않는다.

## 코딩 컨벤션

- **C++**: C++20. 헤더는 `include/navigation/`, 구현은 `src/` 동일 경로. 네임스페이스 `navigation::<module>`. 소유권은 `unique_ptr`/값 타입 우선, raw new/delete 금지. 예외는 로드/검증 단계에서만, planning hot path 에서는 사용하지 않는다.
- **Python**: 전 함수 type hint 필수. `numpy` 기반 좌표 연산. 추상 클래스는 `abc.ABC`. `any`/무타입 dict 전달 금지 — 파라미터는 `ParamSet`, 상태는 `types.py` 의 dataclass 를 쓴다.
- 좌표계: world 좌표 (x, y[, theta]) 는 float, grid 인덱스는 (row, col) int — 변환은 맵 클래스만 담당한다. 이 구분을 흐리는 코드 금지.
- 주석은 WHY 만. 알고리즘 수식/휴리스틱 선택 근거는 논문 인용(저자, 연도)으로 남긴다.

## 문서 사이트 (document/)

React 18 + Vite + TS + Tailwind SPA. 2D 는 Konva, 3D 는 Babylon(필요 시 도입), 수식은 KaTeX, 이중언어는 `<T en ko>`. 대분류(Global/Local Planning, Multi-Agent)는 저장소 최상위 카테고리와 1:1 미러. 빌드/검증: `cd document && yarn build`, dev 서버 `yarn dev`.

### 알고리즘 페이지 규칙 (순서 고정)

인트로 → 개념/유도(From X to Y 등) → **Properties and Complexity** → **The Algorithm** → 증명(collapsible) → (반례 등 이론 보조) → **Demo** → **Implementation** → **References**. registry `sections[]` 도 같은 순서로.

- **The Algorithm**: 자료구조·루프 요약 문단 → `Pseudocode` 블록(`# 1~n` 스텝 마커) → 바로 아래 "1. ~한다" 번호 목록으로 각 스텝의 무엇/왜 해설 (pop 시점 goal 검사 같은 함정 포함).
- **증명**: 산문 서술 금지. 가정 → BlockMath 부등식 체인 → 모순/결론의 단계형.
- **Parameters 섹션 금지** — 웹은 알고리즘 설명이지 코드 문서가 아니다. parameter 개념(예: weighted A* 의 w)은 이론 산문에서 다룬다.
- **Demo**: 간단한 알고리즘은 TS 라이브 엔진(trace 이벤트 계약 공유), 무거운 것은 기록 trace 재생. TraceReplay 는 py trace 한 벌만(`<map>.py.jsonl.gz`, C++/Python 은 동일 이벤트 열), 배속 버튼 없이 고정 2×.
- **Implementation**: 실제 저장소 소스를 vite `?raw` 로 embed (사본 금지), python/c++ 탭 토글 + 파일별 GitHub 링크.
- **References**: 실제 논문 링크(DOI) 필수.
- **시각 자료 적극 배치**: 페이지·소개마다 Konva figure (CanvasFigure 래핑, 테마 색은 useCanvasColors). 데이터 표는 가운데 정렬(전역 CSS 처리됨).

### 한국어 작문 규칙

- 기술 용어 과잉 번역 금지: 헤딩·UI 는 Demo / Parameters / References 처럼 영어 유지. "인터랙티브 데모" 같은 음차 금지.
- **직역 금지**: 한국어는 영어 번역이 아니라 같은 내용을 한국어로 새로 쓴다. "어른이 된 BFS"(BFS grown up), "비용 물결"(cost ripple) 류 번역투 금지 — 자연스러운 표현(비용 등고선, 맹목 탐색 등)으로.
- 한국어 산문에서 em-dash 삽입구("A — B — C") 금지. 문장 분리나 쉼표/괄호로 재구성.
- **조사는 선행 영어 토큰에 붙인다**: "BFS 는" ❌ → "BFS는" ✅, "frontier 를" ❌ → "frontier를" ✅. 수식 컴포넌트(`<InlineMath/>`) 뒤 조사도 동일.
- 세미콜론 문장 연결("~한다; ~한다") 금지 — 마침표로 분리.

### PR 워크플로우

- **머지된 브랜치에 후속 커밋을 push 하지 않는다.** push 전에 해당 브랜치 PR 상태를 확인하고, 이미 머지됐으면 main 에서 새 브랜치를 파서 새 PR 로 올린다.
