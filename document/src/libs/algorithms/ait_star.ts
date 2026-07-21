import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {informedSample} from "./informed_rrt_star";
import {rggRadius} from "./prm";

// 브라우저 라이브 데모용 AIT* — Adaptively Informed Trees (Strub & Gammell 2020).
// 저장소 python 구현을 연산 순서·RNG·tie-break·이벤트 순서까지 그대로 미러한다.
// BIT*와의 차이는 cost-to-go heuristic이다: BIT*는 직선거리 h_hat(x)=‖x−goal‖를
// 쓰는데, AIT*는 현재 표본 그래프(RGG) 위 goal로부터의 역방향 Dijkstra로 h_hat을
// 만든다. 벽을 우회하는 실제 연결성을 반영한 문제 맞춤 heuristic이다. 게다가
// 전방 탐색이 충돌로 무효라고 밝힌 간선은 invalid_edges에 영구히 쌓여, 다음
// 배치의 역방향 탐색이 도는 그래프에서 빠진다. 곧 heuristic이 발견된 장애물에
// 맞춰 스스로 수리된다 (Strub & Gammell 2020, ICRA; 확장 IJRR 2022).
//
// 표본은 두 RNG 스트림에서 나온다: 아직 해가 없으면 map RNG(space.sample())로
// 전 공간을, 해가 생기면 planner RNG로 informed 타원 안을 뽑는다. informed_sample이
// 두 스트림을 나눠 쓰므로 같은 seed면 python demo와 표본·간선·확장 수까지 일치한다.
export interface AITStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    batchSize: number;
    maxBatches: number;
    gamma: number;
    seed: number;
}

// 알고리즘 한 번 실행의 산출물. 이벤트열은 재생·parity용이고, 나머지 셋은 sandbox가
// heuristic 품질을 읽는 값이다: 마지막 배치의 역방향 heuristic이 시작점에서 보는
// cost-to-go(장애물 인지) vs 직선거리(BIT* 방식) vs 실제 현직 해 비용.
interface AITStarResult {
    events: TraceEvent[];
    reverseHStart: number;
    straightHStart: number;
    optimal: number;
    success: boolean;
}

const INF = Infinity

// 무향 간선 키: 충돌이 어느 방향으로 검출됐든 같은 간선은 같은 키를 갖는다.
const edgeKey = (i: number, j: number): string => (i < j ? `${i}-${j}` : `${j}-${i}`)

// (key, idx) 사전순 최소를 뽑는 이진 힙. idx가 유일하므로 python heapq의 pop 순서
// (같은 key는 idx 오름차순)와 정확히 일치한다. 전방 탐색은 pop한 key까지 필요하므로
// (lazy 삭제 판정) pop은 [key, idx]를 함께 돌려준다.
class MinHeap {
    private readonly key: number[] = []
    private readonly idx: number[] = []

    get size(): number {
        return this.idx.length
    }

    private less(a: number, b: number): boolean {
        return this.key[a] < this.key[b]
            || (this.key[a] === this.key[b] && this.idx[a] < this.idx[b])
    }

    private swap(a: number, b: number): void {
        [this.key[a], this.key[b]] = [this.key[b], this.key[a]];
        [this.idx[a], this.idx[b]] = [this.idx[b], this.idx[a]]
    }

    push(key: number, idx: number): void {
        this.key.push(key)
        this.idx.push(idx)
        let c = this.size - 1
        while (c > 0) {
            const p = (c - 1) >> 1
            if (!this.less(c, p)) break
            this.swap(c, p)
            c = p
        }
    }

    pop(): [number, number] {
        const topKey = this.key[0]
        const topIdx = this.idx[0]
        const last = this.size - 1
        this.swap(0, last)
        this.key.pop()
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
        return [topKey, topIdx]
    }
}

// 배치마다 한 번, 각 점의 반경 내 이웃 index를 미리 계산한다 (역방향·전방 탐색이
// near-set을 여러 번 훑으므로). 오름차순 index로 채워져 tie-break가 python과 같다.
function radiusNeighbors(space: SamplingGrid, points: Point[], radius: number): number[][] {
    const n = points.length
    const nbr: number[][] = Array.from({length: n}, () => [])
    for (let i = 0; i < n; i++) {
        const pi = points[i]
        for (let j = i + 1; j < n; j++) {
            if (space.distance(pi, points[j]) <= radius) {
                nbr[i].push(j)
                nbr[j].push(i)
            }
        }
    }
    return nbr
}

// 적응형 cost-to-go heuristic: goal에서 RGG(무효 간선 제외) 위로 도는 Dijkstra.
// 낙관적이다 — 남은 간선이 충돌 없다고 가정하며 충돌 검사는 하지 않는다. 검증은
// 전방 탐색의 몫이고, 그 발견이 다음 배치의 이 그래프를 좁힌다 (Strub & Gammell 2020).
function reverseSearch(
    space: SamplingGrid, points: Point[], nbr: number[][],
    invalidEdges: Set<string>, goalIdx: number,
): number[] {
    const n = points.length
    const h = new Array<number>(n).fill(INF)
    const settled = new Array<boolean>(n).fill(false)
    h[goalIdx] = 0
    const heap = new MinHeap()
    heap.push(0, goalIdx)
    while (heap.size > 0) {
        const [, u] = heap.pop()
        if (settled[u]) continue
        settled[u] = true
        for (const w of nbr[u]) {
            if (invalidEdges.has(edgeKey(u, w))) continue
            const nd = h[u] + space.distance(points[u], points[w])
            if (nd < h[w]) {
                h[w] = nd
                heap.push(nd, w)
            }
        }
    }
    return h
}

// AIT* 한 번 실행 — 이벤트열과 함께, 마지막 배치의 시작점 역방향 heuristic을 돌려준다.
// runAITStar / aitStarReadout이 공유하는 코어라 두 진입점의 로직이 갈라지지 않는다.
function aitStar(opts: AITStarOptions): AITStarResult {
    const {map, start, goal, batchSize, maxBatches, gamma, seed} = opts
    const space = new SamplingGrid(map, seed)
    // planner 자체 draw(informed 타원)용 스트림 — map RNG와 분리되지만 같은 seed다.
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "ait_star",
          params: {batch_size: batchSize, max_batches: maxBatches, gamma, seed}})

    const goalIdx = 1
    const points: Point[] = [start, goal]     // 0 = start (root), 1 = goal
    const invalidEdges = new Set<string>()    // 배치를 넘어 자라는 적응형 피드백
    let cBest = INF
    let expanded = 0
    // 마지막 배치의 전방 탐색 결과. 최종 현직 해는 여기서 읽는다.
    let g: number[] = [0, INF]
    let parent: number[] = [-1, -1]
    // 마지막 배치의 역방향 heuristic이 시작점에서 본 cost-to-go (sandbox readout).
    let reverseHStart = space.distance(start, goal)

    const reconstruct = (goalNode: number): number[][] => {
        const path: number[][] = []
        let node = goalNode
        while (node !== -1) {
            path.push([points[node][0], points[node][1]])
            node = parent[node]
        }
        path.reverse()
        return path
    }

    for (let batch = 0; batch < maxBatches; batch++) {
        // --- 1. RGG를 키운다 (해가 생기면 informed) ---------------------------
        let drawn = 0
        for (let attempt = 0; attempt < batchSize * 40; attempt++) {
            if (drawn >= batchSize) break
            const q = informedSample(space, start, goal, cBest, rng)
            if (!space.isStateValid(q)) continue
            points.push(q)
            drawn++
            emit({event: "sample_drawn", state: [q[0], q[1]]})
        }

        const n = points.length
        const radius = rggRadius(gamma, n)
        const nbr = radiusNeighbors(space, points, radius)

        // --- 2+3. 무효 간선을 뺀 그래프 위 역방향 탐색 = 적응형 heuristic -------
        const hHat = reverseSearch(space, points, nbr, invalidEdges, goalIdx)
        if (hHat[0] < INF) reverseHStart = hHat[0]

        // --- 4. g + h_hat을 키로 하는 전방 A*, 각 간선을 lazy 검증 ------------
        g = new Array<number>(n).fill(INF)
        g[0] = 0
        parent = new Array<number>(n).fill(-1)
        const closed = new Array<boolean>(n).fill(false)
        const openHeap = new MinHeap()
        openHeap.push(hHat[0], 0)
        while (openHeap.size > 0) {
            const [key, v] = openHeap.pop()
            // Lazy 삭제: 더 싼 g[v]로 대체된 낡은 항목을 건너뛴다 (BIT*가 vertex
            // queue에 쓰는 것과 같은 수법).
            if (closed[v] || key > g[v] + hHat[v] + 1e-9) continue
            closed[v] = true
            expanded++
            for (const x of nbr[v]) {
                if (invalidEdges.has(edgeKey(v, x))) continue
                const d = space.distance(points[v], points[x])
                if (!space.isMotionValid(points[v], points[x])) {
                    // 무효 발견: 이후 모든 배치의 역방향 탐색에서 제외한다 — AIT*의
                    // 적응형 피드백 루프다.
                    invalidEdges.add(edgeKey(v, x))
                    continue
                }
                const newG = g[v] + d
                if (newG < g[x]) {
                    const first = g[x] === INF
                    g[x] = newG
                    parent[x] = v
                    // candidate_evaluated는 실행 가능하고 개선되는 간선에만 방출한다
                    // (이완한 이웃 전부가 아니라). 배치마다 그래프를 다시 세우는 이
                    // 구현에서 trace가 replay.py로 재생 가능한 규모를 유지하게 한다.
                    emit({event: "candidate_evaluated", state: [points[x][0], points[x][1]],
                          cost: newG})
                    if (first) {
                        emit({event: "edge_added", state: [points[x][0], points[x][1]],
                              parent: [points[v][0], points[v][1]], cost: d})
                    } else {
                        emit({event: "rewire", state: [points[x][0], points[x][1]],
                              parent: [points[v][0], points[v][1]]})
                    }
                    openHeap.push(newG + hHat[x], x)
                    if (x === goalIdx && newG < cBest) {
                        cBest = newG
                        emit({event: "path_found", path: reconstruct(goalIdx), cost: newG})
                    }
                }
            }
        }
    }

    // --- 최종 현직 해 추출 ----------------------------------------------------
    const n = points.length
    const success = g[goalIdx] < INF
    let cost = 0
    if (success) {
        const path = reconstruct(goalIdx)
        for (let i = 1; i < path.length; i++) {
            cost += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        }
    }
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: success ? cost : 0, expanded_nodes: expanded,
                  samples: n, tree_size: n},
    })
    return {
        events,
        reverseHStart,
        straightHStart: space.distance(start, goal),
        optimal: success ? cost : 0,
        success,
    }
}

export function runAITStar(opts: AITStarOptions): TraceEvent[] {
    return aitStar(opts).events
}

// sandbox 전용: heuristic 품질 readout. 역방향(장애물 인지) heuristic이 시작점에서
// 보는 cost-to-go를 직선거리·실제 최적 비용과 나란히 재려고 스칼라만 뽑는다.
export function aitStarReadout(
    opts: AITStarOptions,
): {straightH: number; reverseH: number; optimal: number; success: boolean} {
    const r = aitStar(opts)
    return {straightH: r.straightHStart, reverseH: r.reverseHStart,
            optimal: r.optimal, success: r.success}
}
