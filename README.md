<div align="center">

# 🤖 navigation study

### 🌐 [robotics-study.github.io/navigation](https://robotics-study.github.io/navigation/)

문서 사이트가 라이브입니다 — 아래 링크는 모두 호스팅된 위키로 연결됩니다.

**로봇 navigation planning 알고리즘 — C++ / Python 독립 이중 구현 스터디**

같은 추상화 설계를 두 언어로 미러링하고, 언어 공용 trace 포맷으로 탐색 과정을 재생하며,<br>
(map × algorithm × language) 매트릭스로 벤치마크한다.

*Robot navigation planning algorithms, mirrored in C++20 and Python — with step-by-step
visualization and a benchmark matrix. Docs available in [Korean](https://robotics-study.github.io/navigation/ko/index.html) and [English](https://robotics-study.github.io/navigation/en/index.html).*

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)
![CMake](https://img.shields.io/badge/CMake-%E2%89%A53.20-064F8C.svg)
![Tests](https://img.shields.io/badge/tests-62%20py%20%2B%2035%20cpp-brightgreen.svg)

| A* (1968) | RRT* (2011) | Fast-RRT (2021) |
|:---:|:---:|:---:|
| ![A*](docs/assets/astar/maze01.gif) | ![RRT*](docs/assets/rrt_star/maze01.gif) | ![Fast-RRT](docs/assets/fast_rrt/maze01.gif) |

*같은 미로, 세 가지 탐색. 색 = 시간 순서 (expanded: 노랑→갈색 · tree: 하늘색 · path: 보라→빨강)*

</div>

---

## ✨ 특징

- **📐 공통 추상화** — 모든 planner 는 `GlobalPlanner`/`LocalPlanner`/`MultiAgentPlanner` 를 상속하고, 구체 맵이 아닌 **capability 인터페이스**(`DiscreteSpace` · `SamplingSpace` · `ObstacleQuery`)만 요구한다. 새 맵 타입을 추가해도 알고리즘 코드는 바뀌지 않는다.
- **🪞 언어 미러링** — C++ 과 Python 이 같은 설계·같은 파라미터·같은 trace 이벤트를 각자 idiomatic 하게 구현한다. 공유 계약(`spec/`)·파라미터(`configs/`)·맵(`maps/`)은 언어 밖에 두고 양쪽에서 로드한다.
- **🎬 Trace 기반 시각화** — 알고리즘은 탐색 진행을 JSON Lines 이벤트로 방출하고, 재생기는 언어당 하나가 아니라 **하나**(`tools/viz/replay.py`)다. GIF 애니메이션 + 중간 과정 PNG 스냅샷을 만든다.
- **📊 벤치마크 매트릭스** — `tools/bench/run_matrix.py` 가 (scenario × algorithm) 전 조합을 실행해 성공 여부·runtime·path cost·expanded/samples 를 수집하고 리포트를 쓴다. C++ vs Python 비교 포함.

## 📚 문서 (GitHub Pages)

**[📖 한국어 위키](https://robotics-study.github.io/navigation/ko/index.html)** · **[📖 English Wiki](https://robotics-study.github.io/navigation/en/index.html)**

알고리즘별 상세 페이지: 이론적 배경 + pseudocode + 성질(완전성·최적성·복잡도) + 파라미터 +
demo GIF/PNG + 실측 metric + **원 논문 각주**.

| | | |
|---|---|---|
| [BFS](https://robotics-study.github.io/navigation/ko/algorithms/bfs.html) — Moore 1959 | [Dijkstra](https://robotics-study.github.io/navigation/ko/algorithms/dijkstra.html) — Dijkstra 1959 | [A*](https://robotics-study.github.io/navigation/ko/algorithms/astar.html) — Hart et al. 1968 |
| [RRT](https://robotics-study.github.io/navigation/ko/algorithms/rrt.html) — LaValle 1998 | [RRT*](https://robotics-study.github.io/navigation/ko/algorithms/rrt_star.html) — Karaman & Frazzoli 2011 | [Fast-RRT](https://robotics-study.github.io/navigation/ko/algorithms/fast_rrt.html) — Wu et al. 2021 |

> **문서 사이트는 손수 디자인한 정적 HTML** 이다 (Jekyll 테마 없음, `docs/.nojekyll`).
> 콘텐츠 소스는 `docs/{ko,en}/**.md`, 공통 크롬·CSS·수식은 한 곳에서 관리하고
> `docs/build.py` 가 `docs/**.html` 로 렌더링한다 (MathJax 수식 · 다크모드 · 검색 · 반응형 포함).
>
> ```bash
> pip install markdown pymdown-extensions   # 최초 1회
> python docs/build.py                       # docs/ 재생성 (콘텐츠 수정 후 실행)
> ```
>
> GitHub Pages 활성화: **Settings → Pages → Deploy from a branch → `main` / `docs/`**.
> `.nojekyll` 덕분에 생성된 HTML 이 그대로 서빙된다.

## 🗺️ 구현 현황 (parity)

| 카테고리 | 알고리즘 | C++ | Python | 원 논문 |
|---|---|:---:|:---:|---|
| global_planning | BFS | ✅ | ✅ | Moore (1959) |
| global_planning | Dijkstra | ✅ | ✅ | Dijkstra (1959) |
| global_planning | A* | ✅ | ✅ | Hart, Nilsson & Raphael (1968) |
| global_planning | RRT | ✅ | ✅ | LaValle (1998) |
| global_planning | RRT-Connect | ⬜ | ⬜ | Kuffner & LaValle (2000) |
| global_planning | RRT* | ✅ | ✅ | Karaman & Frazzoli (2011) |
| global_planning | Informed RRT* | ⬜ | ⬜ | Gammell et al. (2014) |
| global_planning | Fast-RRT | ✅ | ✅ | Wu et al. (2021) |
| local_planning | DWA | ⬜ | ⬜ | Fox, Burgard & Thrun (1997) |
| local_planning | Pure Pursuit | ⬜ | ⬜ | Coulter (1992) |
| local_planning | VFH | ⬜ | ⬜ | Borenstein & Koren (1991) |
| local_planning | MPC | ⬜ | ⬜ | — |
| multi_agent | Prioritized A* | ⬜ | ⬜ | Erdmann & Lozano-Pérez (1987) |
| multi_agent | Joint-space A* | ⬜ | ⬜ | — |
| multi_agent | CBS | ⬜ | ⬜ | Sharon et al. (2015) |

⬜ planned · 🔶 in progress · ✅ done — 전체 서지는 [참고 문헌](https://robotics-study.github.io/navigation/ko/references.html) 참고.

## 🚀 빠른 시작

```bash
# Python (>= 3.10) — nav_study 패키지 + viz/dev extras
cd python && pip install -e ".[dev,viz]" && cd ..
pytest python/tests            # 62 tests

# C++ (C++20, CMake >= 3.20, GoogleTest 는 FetchContent 자동)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build     # 35 tests
```

### 데모 실행 — 두 언어가 동일한 CLI 인자

```bash
# Python
python python/demos/demo_astar.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/astar.yaml --trace out/astar.jsonl

# C++ (동일 인자)
./cpp/build/demos/demo_astar \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/astar.yaml --trace out/astar.jsonl
```

stdout 에 한 줄 JSON metric, `--trace` 경로에 step-by-step JSONL trace 가 남는다.
옵션: `[--seed N] [--connectivity 4|8]`

### 시각화 — C++/Python trace 를 같은 도구로 재생

```bash
python tools/viz/replay.py out/astar.jsonl                    # interactive 재생
python tools/viz/replay.py out/astar.jsonl --gif out/astar.gif --snapshots out/snaps/
```

### 벤치마크

```bash
python tools/bench/run_matrix.py --out out/report.md
```

실측 요약 (maze01, seed 1 — 자세한 수치·해석은 [벤치마크 문서](https://robotics-study.github.io/navigation/ko/benchmarks.html)):

| algorithm | path cost | 탐색량 | 특성 |
|---|---|---|---|
| BFS | 28.73 | 221 expanded | 간선 수 최소 |
| Dijkstra | 28.73 | 211 expanded | 비용 최적 |
| A* | 28.73 | **108 expanded** | 비용 최적 + heuristic |
| RRT | 18.41 | 229 samples | 첫 feasible 해 |
| RRT* | **13.46** | 8,000 samples | anytime 최적 수렴 |
| Fast-RRT | 13.47 | 8,000 samples | + shortcut (waypoint 5개) |

## 📁 저장소 구조

```
├── spec/          # 언어 공용 계약 — trace/param 스키마, 맵 포맷 (single source of truth)
├── maps/          # 벤치마크 맵 (grid/graph/topology/continuous) + start/goal 시나리오
├── configs/       # 알고리즘별 파라미터 yaml — C++/Python 이 같은 파일을 읽는다
├── cpp/           # C++20 구현 (include + src + demos + GoogleTest)
├── python/        # Python 구현 (nav_study 패키지 + demos + pytest)
├── tools/         # viz(trace 재생기) · bench(매트릭스 러너)
└── docs/          # GitHub Pages 위키 (한국어/English)
```

아키텍처 원칙(의존 방향, capability 모델, trace 계약)은 [아키텍처 문서](https://robotics-study.github.io/navigation/ko/architecture.html)
와 [CLAUDE.md](CLAUDE.md) 참고.

## 🧭 새 알고리즘 추가

1. `configs/<category>/<algo>.yaml` 파라미터 선언 → 2. 두 언어 구현 (`required_capabilities()` 포함)
→ 3. trace 이벤트 방출 → 4. 두 언어 demo → 5. 단위 테스트 (최적성/no-path/param 검증)
→ 6. bench 매트릭스 통과 → 7. `replay.py --gif/--snapshots` 렌더 확인 → 8. parity 표 + 문서 갱신.

상세 체크리스트는 [CLAUDE.md](CLAUDE.md) 참고.
