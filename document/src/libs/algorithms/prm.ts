import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {Point, SamplingGrid} from "./sampling_space";

// 브라우저 라이브 데모용 PRM / PRM* (Kavraki et al. 1996; Karaman & Frazzoli
// 2011). 저장소 구현을 그대로 미러한다: free 표본 → 반경 연결 → roadmap 위
// Dijkstra. 두 알고리즘은 연결 반경 정책만 다르다 (고정 r vs r_n = γ·√(log n/n)).
// 표본은 map RNG(numpy PCG64 미러)에서 나오므로 같은 seed 면 python demo와
// 표본·간선·확장 수까지 일치한다.
export interface PRMOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    numSamples: number;
    connectionRadius: number;
    seed: number;
}

export interface PRMStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    numSamples: number;
    gamma: number;
    seed: number;
}

// PRM*의 RGG 연결 반경: r_n = γ·(log n / n)^(1/d), d = 2.
export const rggRadius = (gamma: number, n: number): number =>
    n <= 1 ? Infinity : gamma * Math.sqrt(Math.log(n) / n)

export const runPRM = (opts: PRMOptions): TraceEvent[] =>
    runRoadmap(opts.map, opts.start, opts.goal, opts.numSamples, opts.seed, {
        algorithm: "prm",
        params: {num_samples: opts.numSamples, connection_radius: opts.connectionRadius,
                 seed: opts.seed},
        radiusOf: () => opts.connectionRadius,
    })

export const runPRMStar = (opts: PRMStarOptions): TraceEvent[] =>
    runRoadmap(opts.map, opts.start, opts.goal, opts.numSamples, opts.seed, {
        algorithm: "prm_star",
        params: {num_samples: opts.numSamples, gamma: opts.gamma, seed: opts.seed},
        radiusOf: (n) => rggRadius(opts.gamma, n),
    })

function runRoadmap(
    map: GridMap, start: Point, goal: Point, numSamples: number, seed: number,
    run: {
        algorithm: string;
        params: Record<string, unknown>;
        // 최종 노드 수(시작/목표 포함)에서 정한 연결 반경 — PRM은 상수를 돌려준다.
        radiusOf: (n: number) => number;
    },
): TraceEvent[] {
    const space = new SamplingGrid(map, seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: run.algorithm, params: run.params})

    const nodes: Point[] = [start, goal]
    const adj: Array<Array<[number, number]>> = [[], []]
    const addNode = (p: Point): number => {
        nodes.push(p)
        adj.push([])
        return nodes.length - 1
    }

    // free 표본 수집 (거의 가득 찬 맵에서 무한 루프를 막는 시도 상한 포함).
    let drawn = 0
    for (let attempt = 0; attempt < numSamples * 20; attempt++) {
        if (drawn >= numSamples) break
        const q = space.sample()
        if (!space.isStateValid(q)) continue
        addNode(q)
        drawn++
        emit({event: "sample_drawn", state: [q[0], q[1]]})
    }

    // 반경 연결 — 각 노드를 앞선 노드들과만 이어 무향 간선을 한 번씩 만든다.
    const radius = run.radiusOf(nodes.length)
    for (let idx = 1; idx < nodes.length; idx++) {
        const node = nodes[idx]
        for (let j = 0; j < idx; j++) {
            if (space.distance(nodes[j], node) > radius) continue
            if (!space.isMotionValid(nodes[j], node)) continue
            const cost = space.distance(nodes[j], node)
            adj[idx].push([j, cost])
            adj[j].push([idx, cost])
            emit({event: "edge_added", state: [node[0], node[1]],
                  parent: [nodes[j][0], nodes[j][1]], cost})
        }
    }

    // roadmap 위 Dijkstra. (dist, index) 사전순 pop이 python heapq 순서와 같다.
    const dist = new Array<number>(nodes.length).fill(Infinity)
    const parent = new Array<number>(nodes.length).fill(-1)
    dist[0] = 0
    const heap: Array<[number, number]> = [[0, 0]]
    const popMin = (): [number, number] | null => {
        let bestAt = -1
        for (let k = 0; k < heap.length; k++) {
            if (bestAt < 0 || heap[k][0] < heap[bestAt][0]
                || (heap[k][0] === heap[bestAt][0] && heap[k][1] < heap[bestAt][1])) bestAt = k
        }
        if (bestAt < 0) return null
        return heap.splice(bestAt, 1)[0]
    }
    let expanded = 0
    while (heap.length > 0) {
        const top = popMin()
        if (!top) break
        const [d, u] = top
        if (d > dist[u]) continue
        expanded++
        emit({event: "node_expanded", state: [nodes[u][0], nodes[u][1]], cost: d})
        if (u === 1) break
        for (const [v, w] of adj[u]) {
            const nd = d + w
            if (nd < dist[v]) {
                dist[v] = nd
                parent[v] = u
                heap.push([nd, v])
            }
        }
    }

    if (dist[1] === Infinity) {
        emit({event: "planning_finished", success: false,
              metrics: {expanded_nodes: expanded, samples: nodes.length, tree_size: nodes.length}})
        return events
    }
    const path: number[][] = []
    let node = 1
    while (node !== -1) {
        path.push([nodes[node][0], nodes[node][1]])
        node = parent[node]
    }
    path.reverse()
    emit({event: "path_found", path, cost: dist[1]})
    emit({
        event: "planning_finished",
        success: true,
        metrics: {path_cost: dist[1], expanded_nodes: expanded,
                  samples: nodes.length, tree_size: nodes.length},
    })
    return events
}
