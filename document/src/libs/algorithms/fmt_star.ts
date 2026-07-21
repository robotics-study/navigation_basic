import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {Point, SamplingGrid} from "./sampling_space";
import {rggRadius} from "./prm";

// 브라우저 라이브 데모용 FMT* — Fast Marching Tree (Janson, Schmerling, Clark &
// Pavone 2015). 저장소 python 구현을 연산 순서까지 그대로 미러한다: 한 배치의
// free 표본 위에서 cost-to-come 순으로 트리를 한 번 행진시키고, 후보마다 근방
// 최소비용 open 이웃 하나로 연결하되 그 한 간선만 lazy 충돌 검사한다. rewire도,
// 재확장도 없다. 표본은 map RNG(numpy PCG64 미러)에서 나오므로 같은 seed면
// python demo와 표본·간선·확장 수까지 일치한다.
export interface FMTStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    numSamples: number;
    gamma: number;
    seed: number;
}

// (cost, idx) 사전순 최소를 뽑는 이진 힙. idx가 유일하므로 python heapq의 pop
// 순서(같은 cost는 idx 오름차순)와 정확히 일치한다.
class MinHeap {
    private readonly cost: number[] = []
    private readonly idx: number[] = []

    get size(): number {
        return this.idx.length
    }

    private less(a: number, b: number): boolean {
        return this.cost[a] < this.cost[b]
            || (this.cost[a] === this.cost[b] && this.idx[a] < this.idx[b])
    }

    private swap(a: number, b: number): void {
        [this.cost[a], this.cost[b]] = [this.cost[b], this.cost[a]];
        [this.idx[a], this.idx[b]] = [this.idx[b], this.idx[a]]
    }

    push(cost: number, idx: number): void {
        this.cost.push(cost)
        this.idx.push(idx)
        let c = this.size - 1
        while (c > 0) {
            const p = (c - 1) >> 1
            if (!this.less(c, p)) break
            this.swap(c, p)
            c = p
        }
    }

    pop(): number {
        const top = this.idx[0]
        const last = this.size - 1
        this.swap(0, last)
        this.cost.pop()
        this.idx.pop()
        let p = 0
        const n = this.size
        for (;;) {
            const l = 2 * p + 1
            const r = l + 1
            let m = p
            if (l < n && this.less(l, m)) m = l
            if (r < n && this.less(r, m)) m = r
            if (m === p) break
            this.swap(p, m)
            p = m
        }
        return top
    }
}

export function runFMTStar(opts: FMTStarOptions): TraceEvent[] {
    const {map, start, goal, numSamples, gamma, seed} = opts
    const space = new SamplingGrid(map, seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "fmt_star",
          params: {num_samples: numSamples, gamma, seed}})

    // 표본 집합: index 0 = start, 1 = goal, 그 뒤로 free 표본 (거의 가득 찬 맵에서
    // 무한 루프를 막는 시도 상한 포함) — PRM/FMT*가 공유하는 수집 루프다.
    const points: Point[] = [start, goal]
    for (let attempt = 0; attempt < numSamples * 20; attempt++) {
        if (points.length - 2 >= numSamples) break
        const q = space.sample()
        if (!space.isStateValid(q)) continue
        points.push(q)
        emit({event: "sample_drawn", state: [q[0], q[1]]})
    }
    const n = points.length
    const radius = rggRadius(gamma, n)

    // batch 반경 그래프를 한 번 미리 계산한다 (행진 루프가 near-set을 여러 번
    // 훑으므로). 오름차순 index로 채워져 tie-break가 python과 같다.
    const neighbors: number[][] = Array.from({length: n}, () => [])
    for (let i = 0; i < n; i++) {
        const pi = points[i]
        for (let j = i + 1; j < n; j++) {
            if (space.distance(pi, points[j]) <= radius) {
                neighbors[i].push(j)
                neighbors[j].push(i)
            }
        }
    }

    const cost = new Array<number>(n).fill(Infinity)
    const parent = new Array<number>(n).fill(-1)
    const inOpen = new Array<boolean>(n).fill(false)
    const visited = new Array<boolean>(n).fill(false)
    cost[0] = 0
    inOpen[0] = true
    // visited[i]는 python의 unvisited set 미러 — start만 방문 처리, goal 포함 나머지는
    // unvisited로 남아 행진 중 연결 대상이 된다.
    visited[0] = true
    const heap = new MinHeap()
    let z = 0
    const goalIdx = 1
    let expanded = 0
    let success = false

    for (;;) {
        expanded++
        for (const x of neighbors[z]) {
            if (visited[x]) continue
            let bestY = -1
            let bestC = Infinity
            for (const y of neighbors[x]) {
                if (!inOpen[y]) continue
                const c = cost[y] + space.distance(points[y], points[x])
                if (c < bestC) {
                    bestC = c
                    bestY = y
                }
            }
            // Lazy 충돌 검사: 근방 최소비용 간선 하나만 검사한다. 막히면 x는
            // unvisited로 남아 나중의 z에서 다시 연결될 수 있다 (Janson et al. 2015).
            if (bestY >= 0 && space.isMotionValid(points[bestY], points[x])) {
                parent[x] = bestY
                cost[x] = bestC
                inOpen[x] = true
                visited[x] = true
                heap.push(bestC, x)
                emit({event: "edge_added", state: [points[x][0], points[x][1]],
                      parent: [points[bestY][0], points[bestY][1]],
                      cost: space.distance(points[bestY], points[x])})
            }
        }
        inOpen[z] = false
        z = -1
        while (heap.size > 0) {
            const i = heap.pop()
            if (inOpen[i]) {
                z = i
                break
            }
        }
        if (z < 0) break  // frontier 고갈: 이 표본 집합에서 goal 도달 불가
        emit({event: "node_expanded", state: [points[z][0], points[z][1]], cost: cost[z]})
        if (z === goalIdx) {
            success = true
            break
        }
    }

    if (success) {
        const path: number[][] = []
        let node = goalIdx
        while (node !== -1) {
            path.push([points[node][0], points[node][1]])
            node = parent[node]
        }
        path.reverse()
        emit({event: "path_found", path, cost: cost[goalIdx]})
    }
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: success ? cost[goalIdx] : 0, expanded_nodes: expanded,
                  samples: n, tree_size: n},
    })
    return events
}
