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

## 성질

- **완전성**: 유한 그래프에서 해가 존재하면 반드시 찾는다.
- **최적성**: 간선 수(hop count) 기준 최단. 간선 비용이 균일할 때만 비용 최적.
- **복잡도**: 시간 O(V + E), 공간 O(V) — 파면 전체를 메모리에 유지한다.

## 최단성 근거와 증명

기호: $\delta(s,v)$ 를 $s\to v$ 경로의 **최소 간선 수**(hop 수), $d[v]$ 를 BFS 가 $v$ 를 큐에서
꺼낼 때 기록한 깊이라 하자.

**보조정리 (큐 단조성).** 실행 내내 큐는 깊이 $k$ 노드가 깊이 $k{+}1$ 노드 앞에 오도록 유지되고,
꺼내는 $d$ 값은 비감소한다.

**정리.** $v$ 를 꺼내는 시점에 $d[v]=\delta(s,v)$ 이다.

*증명.* $d$ 에 대한 귀납. $d[s]=0=\delta(s,s)$. $v$ 를 꺼내 이웃 $u$ 가 미방문이면 $d[u]:=d[v]+1$
로 두고 enqueue 한다. FIFO 와 보조정리에 의해 깊이 $k$ 노드가 깊이 $k{+}1$ 노드보다 먼저 처리되므로
$u$ 가 **처음** 도달되는 경로가 곧 최소 hop 경로다: $d[u]=d[v]+1=\delta(s,v)+1=\delta(s,u)$.
따라서 goal 을 처음 만나는 순간의 경로는 hop 수 최소다. ∎

**비용 최적성의 한계.** 가중치 $w_e$ 를 갖는 경로 비용은 $\sum_e w_e$ 다. hop 최소가 곧 비용 최소가
되는 것은 **모든 $w_e$ 가 같을 때뿐**이다. 이 저장소의 8-connected grid 는 $w\in\{1,\sqrt2\}$ 로
균일하지 않아 world 거리 최적성은 보장되지 않는다 (데모에서 Dijkstra 와 값이 같은 건 이 맵에서의 우연).

**복잡도.** 각 정점은 enqueue/dequeue 정확히 1 회 $O(V)$, 인접 리스트는 정점당 1 회 스캔되어
$\sum_v\deg(v)=2E$ — 총 $O(V+E)$.

## 구현 노트

- C++: `cpp/src/global_planning/bfs.cpp`, Python: `python/navigation/global_planning/bfs.py`
- 두 구현 모두 `DiscreteSpace` capability(`neighbors`, `heuristic`)에만 의존한다. 구체 맵 타입을 알지 못한다.
- BFS 는 튜닝 파라미터가 없다 (`configs/global_planning/bfs.yaml` 은 빈 선언). 결정적이므로
  C++/Python 결과가 완전히 일치한다.

## 방출 trace 이벤트

`planning_started` → (`node_expanded`, `edge_added`)* → `path_found` → `planning_finished`

## Demo

`maze01` (20×20, 좁은 통로 미로) 에서의 탐색 진행. 파면이 비용과 무관하게 균일하게 퍼지는 것이 보인다.

![BFS on maze01](../../assets/bfs/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/bfs/maze01_snap_02.png) | ![mid](../../assets/bfs/maze01_snap_05.png) | ![final](../../assets/bfs/maze01_final.png) |

`open01` (열린 필드) 최종 결과:

![BFS on open01](../../assets/bfs/open01_final.png)

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

## References

[^moore]: Moore, E. F. (1959). "The shortest path through a maze." *Proceedings of the International Symposium on the Theory of Switching*, Harvard University Press, 285–292.
[^lee]: Lee, C. Y. (1961). "An algorithm for path connections and its applications." *IRE Transactions on Electronic Computers*, EC-10(3), 346–365. [doi:10.1109/TEC.1961.5219222](https://doi.org/10.1109/TEC.1961.5219222)
