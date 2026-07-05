---
title: Dijkstra
layout: default
parent: 알고리즘
grand_parent: 한국어
nav_order: 2
---

[🇰🇷 한국어](dijkstra.md) | [🇬🇧 English](../../en/algorithms/dijkstra.md)

# Dijkstra
{: .no_toc }

| 항목 | 내용 |
|---|---|
| 분류 | uninformed graph search (uniform-cost) |
| 요구 capability | `DiscreteSpace` |
| 완전성 | complete (유한 그래프, 비음수 비용) |
| 최적성 | **cost 최적** |
| 복잡도 | 시간 O((V+E) log V) (binary heap), 공간 O(V) |
| 원 논문 | Dijkstra (1959) [^dijkstra] |

1. TOC
{:toc}

## 배경

Dijkstra 알고리즘[^dijkstra]은 비음수 간선 비용 그래프에서 단일 출발점 최단 경로를 찾는다.
1959 년 3 쪽짜리 논문으로 발표된 이후 지금까지 모든 cost 기반 경로 탐색의 뿌리다.
BFS 가 "간선 수" 순서로 파면을 넓힌다면, Dijkstra 는 "누적 비용 g" 순서로 넓힌다 —
goal 을 향한 정보(heuristic)가 전혀 없다는 점에서 uninformed search 이고,
[A*](astar.md) 는 여기에 heuristic 을 더한 것이다 (h ≡ 0 인 A* 가 곧 Dijkstra 다).

## 동작 원리

우선순위 큐에서 g 값이 가장 작은 노드를 꺼내 확정(settle)한다. 비음수 비용에서는 확정 시점의
g 값이 그 노드까지의 최단 비용임이 보장된다 (greedy 선택의 정당성 — 논문의 핵심 논증).
이웃으로의 완화(relaxation)로 더 짧은 경로가 발견되면 g 를 갱신한다.

```
DIJKSTRA(start, goal):
    g[start] ← 0; open ← priority queue keyed by g
    while open is not empty:
        v ← open.pop_min()                    # 최소 g — 이 시점에 v 의 최단 비용 확정
        if v == goal: return reconstruct(v)
        if v already settled: continue        # lazy deletion (아래 구현 노트)
        for (u, c) in neighbors(v):
            if g[v] + c < g[u]:               # relaxation
                g[u] ← g[v] + c; parent[u] ← v
                open.push(u, g[u])
    return failure
```

## 성질

- **완전성**: 유한 그래프 + 비음수 비용에서 해가 존재하면 찾는다.
- **최적성**: 반환 경로는 비용 최적이다. (음수 간선이 있으면 성립하지 않는다 — Bellman-Ford 영역.)
- **복잡도**: binary heap 기준 O((V+E) log V). Fibonacci heap 이론치는 O(E + V log V) 이나
  실무에서는 binary heap + lazy deletion 이 일반적이다.

## 최적성 증명 (settle 시점의 정확성)

비음수 가중치 $w(u,v)\ge 0$ 를 가정한다. $\delta(s,u)$ 를 $s\to u$ 최단 **비용**이라 하자.

**정리.** 우선순위 큐가 $u$ 를 (최소 $g$ 로) 꺼내 확정하는 시점에 $g[u]=\delta(s,u)$ 이다.

*증명 (귀류법).* $g[u]>\delta(s,u)$ 인 채로 확정되는 **첫** 노드를 $u$ 라 하자
($g[s]=0=\delta(s,s)$ 이므로 $u\ne s$). 최단 경로 $P:s\rightsquigarrow u$ 위에서 아직 확정되지 않은
첫 노드를 $y$, 그 직전(확정됨, $g[x]=\delta(s,x)$)을 $x$ 라 한다. $x$ 확정 시 간선 $(x,y)$ 가
완화되었으므로

$$
g[y]\;\le\;g[x]+w(x,y)\;=\;\delta(s,x)+w(x,y)\;=\;\delta(s,y).
$$

또한 $y$ 가 $P$ 상에서 $u$ 를 선행하고 남은 구간 비용이 비음수이므로 $\delta(s,y)\le\delta(s,u)$.
종합하면

$$
g[y]\;\le\;\delta(s,y)\;\le\;\delta(s,u)\;<\;g[u].
$$

그러면 큐는 $u$ 가 아니라 $y$ 를 먼저 꺼내야 하므로 모순이다. 따라서 $g[u]=\delta(s,u)$.
비음수 가정은 $\delta(s,y)\le\delta(s,u)$ 단계에서 필수적이다 (음수 간선이면 성립하지 않음 →
Bellman–Ford 영역). ∎

**복잡도.** binary heap 기준 extract-min $V$ 회 $O(V\log V)$, 완화마다 push $E$ 회
$O(E\log V)$ → $O((V+E)\log V)$.

## 구현 노트

- C++: `cpp/src/global_planning/dijkstra.cpp`, Python: `python/navigation/global_planning/dijkstra.py`
- Dijkstra 와 A* 는 **우선순위 키만 다르다** (f = g vs f = g + w·h). 두 언어 모두 공통
  best-first 골격(`discrete_search` / `_bestfirst`)을 공유해 이 관계를 코드로 드러낸다.
- decrease-key 자료구조 대신 **lazy queue** 를 쓴다: 같은 노드가 큐에 중복 삽입될 수 있고,
  pop 시점에 이미 확정된 노드면 건너뛴다. heap 크기가 다소 커지는 대신 구현이 단순하고 빠르다.
- 파라미터 없음 (`configs/global_planning/dijkstra.yaml` 은 빈 선언). 결정적이므로 C++/Python
  결과가 완전히 일치한다.

## 방출 trace 이벤트

`planning_started` → (`node_expanded`, `edge_added`)* → `path_found` → `planning_finished`

## Demo

`maze01` 에서의 탐색. BFS 와 달리 파면이 **누적 비용 등고선**을 따라 퍼진다 (대각 이동 √2 반영).

![Dijkstra on maze01](../../assets/dijkstra/maze01.gif)

탐색 중간 과정 (좌 → 우: 초반 / 중반 / 최종 경로):

| | | |
|:---:|:---:|:---:|
| ![early](../../assets/dijkstra/maze01_snap_02.png) | ![mid](../../assets/dijkstra/maze01_snap_05.png) | ![final](../../assets/dijkstra/maze01_final.png) |

`open01` 최종 결과:

![Dijkstra on open01](../../assets/dijkstra/open01_final.png)

측정치 (Python, trace on):

| map | path cost | expanded nodes | path len |
|---|---|---|---|
| maze01 | 28.728 | 211 | 26 |
| open01 | 25.213 | 267 | 20 |

경로 비용은 [A*](astar.md) 와 동일하고 (둘 다 최적), 확장 노드 수는 A* 의 약 2–4 배다 —
heuristic 이 없어서 goal 반대 방향까지 균등하게 탐색하기 때문이다.

재현:

```bash
python python/demos/demo_dijkstra.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_s1.yaml \
  --params configs/global_planning/dijkstra.yaml --trace out/dijkstra.jsonl
python tools/viz/replay.py out/dijkstra.jsonl --gif out/dijkstra.gif
```

## References

[^dijkstra]: Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs." *Numerische Mathematik*, 1, 269–271. [doi:10.1007/BF01386390](https://doi.org/10.1007/BF01386390)
