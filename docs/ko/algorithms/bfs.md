---
title: BFS
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 1
---

[🇰🇷 한국어](bfs.md) | [🇬🇧 English](../../en/algorithms/bfs.md)

# BFS — Breadth-First Search
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | uninformed graph search |
| 요구 capability | `DiscreteSpace` |
| 완전성 | complete (유한 그래프) |
| 최적성 | **edge 수 최소** — unit cost 그래프에서만 cost 최적 |
| 복잡도 | 시간 O(V + E), 공간 O(V) |
| 원류 | Moore (1959) [^moore], Lee (1961) [^lee] |

1. TOC
{:toc}

## 배경

BFS 는 시작 노드에서 가까운 순서(간선 수 기준)로 파면(frontier)을 넓혀가는 가장 기본적인 그래프 탐색이다.
경로 탐색 알고리즘으로서의 기원은 Moore 의 미로 탐색 연구[^moore]와, 회로 배선 문제에 같은 아이디어를
독립적으로 적용한 Lee 알고리즘[^lee]으로 거슬러 올라간다. Dijkstra·A* 를 이해하기 위한 기준선이자,
**간선 비용이 균일한 격자에서는 지금도 유효한 최단 경로 알고리즘**이다.

## 동작 원리

`maze01` (20×20, 좁은 통로 미로) 에서의 탐색 진행. 파면이 비용과 무관하게 균일하게 퍼지는 것이 보인다.

![BFS on maze01](../../assets/bfs/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/bfs/maze01_snap_02.png) | ![mid](../../assets/bfs/maze01_snap_05.png) | ![final](../../assets/bfs/maze01_final.png) |

`open01` (열린 필드) 최종 결과:

![BFS on open01](../../assets/bfs/open01_final.png)

FIFO 큐 하나로 동작한다. 큐에서 꺼낸 노드를 확정(expanded)하고, 아직 방문하지 않은 이웃을 모두 큐에 넣는다.
간선 수 k 인 모든 노드는 간선 수 k+1 인 노드보다 먼저 확정되므로, goal 을 처음 만나는 순간의 경로가
간선 수 최소 경로다.

```
BFS(start, goal):
    queue ← [start]; visited ← {start}; parent ← {}
    while queue is not empty:
        v ← queue.pop_front()                # FIFO — 이 한 줄이 BFS 를 정의한다
        if v == goal: return reconstruct(parent, goal)
        for (u, _cost) in neighbors(v):      # cost 는 무시된다
            if u ∉ visited:
                visited.add(u); parent[u] = v
                queue.push_back(u)
    return failure
```

{: .warning }
> 이 저장소의 grid 는 8-connected 로 대각 이동 비용이 √2 다. 즉 **비용이 균일하지 않으므로**
> BFS 경로는 간선 수만 최소일 뿐 world 거리 기준 최적이 아닐 수 있다. 데모에서 BFS 와 Dijkstra 의
> path cost 가 같게 나오는 것은 이 맵 구조에서 우연히 일치한 것이지 보장이 아니다.

측정치 (Python, trace on):

| map | path cost | expanded nodes | path len |
|---|---|---|---|
| maze01 | 28.728 | 221 | 26 |
| open01 | 25.213 | 267 | 20 |

같은 맵에서 [A*](astar.md) 는 heuristic 덕분에 각각 108 / 71 개만 확장한다.

재현:

```bash
python python/demos/demo_bfs.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/bfs.yaml --trace out/bfs.jsonl
python tools/viz/replay.py out/bfs.jsonl --gif out/bfs.gif --snapshots out/bfs_snaps/
```

## 성질

- **완전성**: 유한 그래프에서 해가 존재하면 반드시 찾는다.
- **최적성**: 간선 수(hop count) 기준 최단. 간선 비용이 균일할 때만 비용 최적.
- **복잡도**: 시간 O(V + E), 공간 O(V) — 파면 전체를 메모리에 유지한다.

## 최단성 근거와 증명

기호: $\delta(s,v)$ 를 $s\to v$ 경로의 **최소 간선 수**(hop 수), $d[v]$ 를 BFS 가 $v$ 를 큐에서
꺼낼 때 기록한 깊이라 하자.

**보조정리 (큐의 2-깊이 불변식).** BFS 실행 중 임의 시점에 큐에 담긴 노드의 깊이는 많아야
**연속한 두 값** $k,\,k{+}1$ 로만 이뤄지고, 앞쪽이 $k$·뒤쪽이 $k{+}1$ 순서다. 따라서 dequeue 되는
$d$ 값은 비감소한다.

*증명.* 큐 연산 수에 대한 귀납. 초기 큐 $=[s]$, 깊이 집합 $\{0\}$ — 성립. 한 스텝에서 앞의 깊이
$k$ 노드 $v$ 를 꺼내고, 미방문 이웃(깊이 $k{+}1$)을 뒤에 넣는다. 큐 앞에 아직 깊이 $k$ 가 남아
있으면 상태는 여전히 $\{k,k{+}1\}$ 이고, 마지막 $k$ 를 꺼내면 앞이 $k{+}1$·뒤가 $k{+}2$ 인
$\{k{+}1,k{+}2\}$ 가 되어 불변식이 유지된다. FIFO 이므로 삽입 순서 = 인출 순서라 앞/뒤 배치도
보존된다. ∎

**정리 (hop 최적).** $v$ 를 꺼내는 시점에 $d[v]=\delta(s,v)$ 이다.

*증명.* $\delta$ 값에 대한 귀납. $d[s]=0=\delta(s,s)$. $\delta(s,u)=k{+}1$ 인 $u$ 를 보자. 어떤
최소 hop 경로의 $u$ 직전 노드 $v$ 는 $\delta(s,v)=k$ 이고, 귀납 가정으로 $v$ 는 $d[v]=k$ 에 꺼내진다.
그 시점에 $u$ 가 미방문이면 $d[u]:=k{+}1$ 로 확정되고, 이미 방문됐다면 보조정리로 그 방문은 깊이
$\le k{+}1$ 에서 일어났다. 어느 쪽이든 $d[u]=k{+}1=\delta(s,u)$ (보조정리가 $d[u]<\delta(s,u)$
를 배제한다 — 더 얕은 깊이로는 $u$ 에 닿는 경로가 없다). 따라서 goal 을 처음 만나는 순간의 경로가
hop 수 최소다. ∎

**BFS 는 unit-cost Dijkstra 다.** FIFO 큐는 사실 정수 키(깊이)에 대한 **버킷 우선순위 큐**다.
모든 간선 비용이 $1$ 이면 노드의 최소 hop 수 = 최소 누적 비용이라, BFS 의 dequeue 순서는
[Dijkstra](dijkstra.md) 의 extract-min 순서와 정확히 일치한다. 즉 위 hop 최적성은
Dijkstra 최적성 증명의 **unit-cost 특수화**이며, BFS 는 $\log V$ 힙 대신 $O(1)$ 큐로 그것을 얻는다.

**비용 최적성의 한계.** 가중치 $w_e$ 를 갖는 경로 비용은 $\sum_e w_e$ 다. hop 최소가 곧 비용 최소가
되는 것은 **모든 $w_e$ 가 같을 때뿐**이다. 이 저장소의 8-connected grid 는 $w\in\{1,\sqrt2\}$ 로
균일하지 않다: 예컨대 대각 1 스텝(비용 $\sqrt2\approx1.414$)이 직교 1 스텝(비용 $1$)과 같은
1 hop 으로 세어져, BFS 는 hop 은 적지만 world 거리로는 더 긴 대각 우세 경로를 고를 수 있다
(데모에서 Dijkstra 와 값이 같은 건 이 맵 형상에서의 우연).

**복잡도.** 각 정점은 enqueue/dequeue 정확히 1 회 $O(V)$, 인접 리스트는 정점당 1 회 스캔되어
$\sum_v\deg(v)=2E$ — 총 $O(V+E)$. 힙이 없으므로 Dijkstra 의 $\log V$ 인자가 사라진다.

## 반례: hop 최소 ≠ cost 최적

BFS 는 **간선 수(hop)** 만 최소화하므로, 대각 이동이 $\sqrt2$ 인 8-connected 격자에서는 hop 이
같아도 **world 비용이 더 큰** 경로를 고를 수 있다. `bfs_hopcost01` 은 S 바로 아래 장애물이 BFS 의
탐색(직교 우선 → 좌측 셀 선점)을 **왼쪽 대각 우회**로 몰아, BFS 와 Dijkstra 가 **똑같이 12 hop**
이지만 비용이 갈린다:

| | BFS (hop 최소) | Dijkstra (cost 최소) |
|---|---|---|
| path cost | **14.899** | **13.243** |
| hop 수 | 12 | 12 |
| 경로 | 좌측으로 부풀었다 대각으로 복귀 | goal 쪽으로 더 곧게 |

![BFS 반례](../../assets/bfs/counter.gif)

| BFS 경로 — cost 14.90 | Dijkstra 최적 — cost 13.24 |
|:---:|:---:|
| ![BFS](../../assets/bfs/counter_final.png) | ![Dijkstra](../../assets/bfs/counter_opt.png) |

두 경로는 같은 장애물을 **반대쪽으로** 돈다. BFS 쪽이 값싼 직교 이동을 $\sqrt2$ 대각으로 바꿔
$14.899-13.243\approx1.66$ 만큼 더 든다 — hop 은 같지만 cost 는 아니다.

```bash
python python/demos/demo_bfs.py --map maps/grid/bfs_hopcost01.yaml \
  --scenario maps/scenarios/bfs_hopcost01_s1.yaml --params configs/global_planning/bfs.yaml \
  --trace out/bfs_ce.jsonl   # Dijkstra 로 바꿔 실행하면 13.243 (더 싸다)
```

## 방출 trace 이벤트

`planning_started` → (`node_expanded`, `edge_added`)* → `path_found` → `planning_finished`

## References

[^moore]: Moore, E. F. (1959). "The shortest path through a maze." *Proceedings of the International Symposium on the Theory of Switching*, Harvard University Press, 285–292.
[^lee]: Lee, C. Y. (1961). "An algorithm for path connections and its applications." *IRE Transactions on Electronic Computers*, EC-10(3), 346–365. [doi:10.1109/TEC.1961.5219222](https://doi.org/10.1109/TEC.1961.5219222)
