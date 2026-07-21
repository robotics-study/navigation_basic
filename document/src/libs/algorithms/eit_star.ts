import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {informedSample} from "./informed_rrt_star";
import {pathLength} from "./rrt";
import {rggRadius} from "./prm";

// 브라우저 라이브 데모용 EIT* — Effort Informed Trees (Strub & Gammell 2022, IJRR).
// 저장소 python 구현을 연산 순서·RNG draw 순서·tie-break·이벤트 순서까지 그대로
// 미러한다. EIT*는 AIT*를 확장한다: AIT*처럼 표본 그래프(RGG) 위 goal로부터의 역방향
// 탐색으로 cost-to-go heuristic ĥ을 만들되, 여기에 두 번째 역방향 탐색을 더해 남은
// 경로의 **검증 노력**(effort, 충돌 검사 sub-segment 수) ê까지 추정한다. 전방
// best-first는 사전순 (g+ĥ, effort_g+ê)로 정렬해 비용이 같은 후보들 사이에서 더 싸게
// 충돌 검사되는 쪽을 먼저 본다. 고차원·검사 비용이 큰 공간에서는 그래프 탐색이 아니라
// 충돌 검사가 런타임을 지배한다는 관찰이 동기다 (Strub & Gammell 2022).
//
// 저장소 단순화: 역방향 탐색은 배치마다 누적 표본 위에서 처음부터 다시 계산한다 (원
// 논문의 LPA* 증분 수리가 아니다). ĥ과 ê는 각각 독립적인 단일 기준 Dijkstra 두 번에서
// 나온다 (논문의 통합 처리가 아니다). effort는 dist/step_size 이산화 proxy다. 배치를
// 넘어 남는 것은 c_best와 영구 무효 간선 집합뿐 — AIT*와 같은 적응형 피드백이다.
//
// 표본은 두 numpy PCG64 스트림에서 나온다: 아직 해가 없으면 map RNG(space.sample()),
// 해가 생기면 planner RNG로 informed 타원 안. 같은 seed면 python demo와 표본·간선·확장
// 수까지 일치한다.
export interface EITStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    batchSize: number;
    maxBatches: number;
    gamma: number;
    stepSize: number;
    seed: number;
}

// 알고리즘 한 번 실행의 산출물. 이벤트열은 재생·parity용이고, readout 스칼라는 sandbox가
// EIT*의 정체(두 역방향 heuristic)를 읽는 값이다: 시작점에서 본 cost-to-go ĥ(meters)과
// effort-to-go ê(충돌 검사 segment 수), 장애물을 못 보는 직선거리 effort 추정, 그리고
// 실제 현직 해의 비용·effort.
interface EITStarResult {
    events: TraceEvent[];
    costToGoStart: number;
    effortToGoStart: number;
    straightEffort: number;
    pathCost: number;
    pathEffort: number;
    success: boolean;
}

const INF = Infinity

// 무향 간선 키: 충돌이 어느 방향으로 검출됐든 같은 간선은 같은 키를 갖는다.
const edgeKey = (i: number, j: number): string => (i < j ? `${i}-${j}` : `${j}-${i}`)

// python round()의 round-half-to-even을 재현한다 — effort가 ê·effort_g의 tie-break에
// 들어가므로, .5 경계에서 JS Math.round(항상 올림)와 갈리면 python과 결과가 어긋날 수
// 있다. 연속 표본에서 .5 경계는 사실상 없지만 readout 값을 python 보고치와 맞추려면
// 은행가 반올림이 정확하다.
const pyRound = (x: number): number => {
    const f = Math.floor(x)
    const diff = x - f
    if (diff < 0.5) return f
    if (diff > 0.5) return f + 1
    return f % 2 === 0 ? f : f + 1
}

// 간선 (a,b)의 effort: step_size 크기 sub-segment 몇 개를 이산화 검증기가 검사하는가.
// SamplingSpace의 distance만 읽는 capability-free proxy다 (Strub & Gammell 2022).
const edgeEffort = (space: SamplingGrid, a: Point, b: Point, step: number): number =>
    Math.max(1, pyRound(space.distance(a, b) / step))

// (k1, k2, idx) 사전순 최소 힙 — python heapq의 튜플 pop 순서를 그대로 낸다. 역방향
// Dijkstra는 (dist, idx) 2튜플이라 b=0으로 밀어 넣어 (dist, idx) 비교로 환원되고, 전방
// 탐색은 (g+ĥ, effort_g+ê, idx) 3튜플이라 비용→effort→idx로 tie-break 한다. idx가
// 유일하므로 pop 순서가 결정적이다.
class Heap {
    private readonly k1: number[] = []
    private readonly k2: number[] = []
    private readonly id: number[] = []

    get size(): number {
        return this.id.length
    }

    private less(a: number, b: number): boolean {
        if (this.k1[a] !== this.k1[b]) return this.k1[a] < this.k1[b]
        if (this.k2[a] !== this.k2[b]) return this.k2[a] < this.k2[b]
        return this.id[a] < this.id[b]
    }

    private swap(a: number, b: number): void {
        [this.k1[a], this.k1[b]] = [this.k1[b], this.k1[a]];
        [this.k2[a], this.k2[b]] = [this.k2[b], this.k2[a]];
        [this.id[a], this.id[b]] = [this.id[b], this.id[a]]
    }

    push(k1: number, k2: number, id: number): void {
        this.k1.push(k1)
        this.k2.push(k2)
        this.id.push(id)
        let c = this.size - 1
        while (c > 0) {
            const p = (c - 1) >> 1
            if (!this.less(c, p)) break
            this.swap(c, p)
            c = p
        }
    }

    pop(): [number, number, number] {
        const top: [number, number, number] = [this.k1[0], this.k2[0], this.id[0]]
        const last = this.size - 1
        this.swap(0, last)
        this.k1.pop()
        this.k2.pop()
        this.id.pop()
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

// 배치마다 한 번, 각 점의 반경 내 이웃 index를 미리 계산한다. 오름차순 index로 채워져
// tie-break가 python과 같다.
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

// goal로부터 고정 무향 그래프 위 단일 기준 Dijkstra. EIT*는 이것을 goal에서 두 번 부른다
// (거리 가중치로 ĥ, effort 가중치로 ê) — 두 역방향 heuristic을 독립적으로 세운다.
function dijkstraFrom(adjacency: number[][], weight: number[][], source: number): number[] {
    const n = adjacency.length
    const dist = new Array<number>(n).fill(INF)
    dist[source] = 0
    const heap = new Heap()
    heap.push(0, source, 0)
    while (heap.size > 0) {
        const [d, u] = heap.pop()
        if (d > dist[u]) continue
        const adjU = adjacency[u]
        const wU = weight[u]
        for (let k = 0; k < adjU.length; k++) {
            const v = adjU[k]
            const nd = d + wU[k]
            if (nd < dist[v]) {
                dist[v] = nd
                heap.push(nd, v, 0)
            }
        }
    }
    return dist
}

// EIT* 한 번 실행 — 이벤트열과 함께 sandbox readout 스칼라를 돌려준다.
// runEITStar / eitStarReadout이 공유하는 코어라 두 진입점의 로직이 갈라지지 않는다.
function eitStar(opts: EITStarOptions): EITStarResult {
    const {map, start, goal, batchSize, maxBatches, gamma, stepSize, seed} = opts
    const space = new SamplingGrid(map, seed)
    // planner 자체 draw(informed 타원)용 스트림 — map RNG와 분리되지만 같은 seed다.
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "eit_star",
          params: {batch_size: batchSize, max_batches: maxBatches, gamma,
                   step_size: stepSize, seed}})

    const startIdx = 0
    const goalIdx = 1
    const points: Point[] = [start, goal]     // 0 = start (root), 1 = goal
    const invalidEdges = new Set<string>()    // 배치를 넘어 자라는 적응형 피드백
    let cBest = INF
    let expanded = 0
    // 마지막 배치의 전방 탐색 결과. 최종 현직 해는 여기서 읽는다.
    let g: number[] = [0, INF]
    let parent: number[] = [-1, -1]
    // 마지막 배치의 역방향 heuristic이 시작점에서 본 값 (sandbox readout).
    let costToGoStart = space.distance(start, goal)
    let effortToGoStart = edgeEffort(space, start, goal, stepSize)

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

        // --- 2. 무효 간선을 뺀 필터 그래프 + 거리/effort 병렬 가중치 ------------
        const adjacency: number[][] = Array.from({length: n}, () => [])
        const distW: number[][] = Array.from({length: n}, () => [])
        const effW: number[][] = Array.from({length: n}, () => [])
        for (let u = 0; u < n; u++) {
            for (const v of nbr[u]) {
                if (invalidEdges.has(edgeKey(u, v))) continue
                adjacency[u].push(v)
                distW[u].push(space.distance(points[u], points[v]))
                effW[u].push(edgeEffort(space, points[u], points[v], stepSize))
            }
        }

        // --- 3. 역방향 탐색: goal로부터 독립 Dijkstra 두 번 (ĥ, ê) -------------
        const hHat = dijkstraFrom(adjacency, distW, goalIdx)
        const eHat = dijkstraFrom(adjacency, effW, goalIdx)
        if (hHat[startIdx] < INF) costToGoStart = hHat[startIdx]
        if (eHat[startIdx] < INF) effortToGoStart = eHat[startIdx]

        // --- 4. 전방 탐색: (비용, effort) 사전순 lazy-deletion best-first ------
        g = new Array<number>(n).fill(INF)
        const effortG = new Array<number>(n).fill(INF)
        parent = new Array<number>(n).fill(-1)
        const closed = new Array<boolean>(n).fill(false)
        g[startIdx] = 0
        effortG[startIdx] = 0
        // 힙 key: (g+ĥ, effort_g+ê, idx). 비용이 1차, effort가 tie-break — 논문의
        // "비용이 같은 해를 effort로 가른다" 정렬이다.
        const heap = new Heap()
        heap.push(hHat[startIdx], eHat[startIdx], startIdx)

        while (heap.size > 0) {
            const [, , v] = heap.pop()
            if (closed[v]) continue
            closed[v] = true
            expanded++
            for (const x of adjacency[v]) {
                const d = space.distance(points[v], points[x])
                if (!space.isMotionValid(points[v], points[x])) {
                    // 무효 발견: 이후 모든 배치의 역방향 탐색에서 제외한다 — 적응형 피드백.
                    invalidEdges.add(edgeKey(v, x))
                    continue
                }
                const newG = g[v] + d
                const newEffort = effortG[v] + edgeEffort(space, points[v], points[x], stepSize)
                // 사전순 채택: 비용이 1차, 누적 effort가 tie-break. 힙 정렬과 일관된다.
                if (newG < g[x] || (newG === g[x] && newEffort < effortG[x])) {
                    const first = parent[x] === -1
                    g[x] = newG
                    effortG[x] = newEffort
                    parent[x] = v
                    closed[x] = false  // 개선됨: 이번 배치에서 재확장 허용
                    heap.push(newG + hHat[x], newEffort + eHat[x], x)
                    // candidate_evaluated는 실행 가능하고 개선되는 간선에만 방출한다
                    // (이완한 이웃 전부가 아니라). 배치마다 그래프를 다시 세우는 이
                    // 구현에서 trace가 replay 가능한 규모를 유지하게 한다.
                    emit({event: "candidate_evaluated", state: [points[x][0], points[x][1]],
                          cost: newG})
                    if (first) {
                        emit({event: "edge_added", state: [points[x][0], points[x][1]],
                              parent: [points[v][0], points[v][1]], cost: d})
                    } else {
                        emit({event: "rewire", state: [points[x][0], points[x][1]],
                              parent: [points[v][0], points[v][1]]})
                    }
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
    let pathCost = 0
    let pathEffort = 0
    if (success) {
        const poly = reconstruct(goalIdx)
        pathCost = pathLength(space, poly.map((p) => [p[0], p[1]] as Point))
        for (let i = 1; i < poly.length; i++) {
            pathEffort += edgeEffort(space, poly[i - 1] as Point, poly[i] as Point, stepSize)
        }
        emit({event: "path_found", path: poly, cost: pathCost})
    }
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: success ? pathCost : 0, expanded_nodes: expanded,
                  samples: n, tree_size: n},
    })
    return {
        events,
        costToGoStart,
        effortToGoStart,
        straightEffort: edgeEffort(space, start, goal, stepSize),
        pathCost: success ? pathCost : 0,
        pathEffort: success ? pathEffort : 0,
        success,
    }
}

export function runEITStar(opts: EITStarOptions): TraceEvent[] {
    return eitStar(opts).events
}

// sandbox 전용: EIT*의 두 역방향 heuristic readout. 시작점에서 본 cost-to-go ĥ과
// effort-to-go ê(장애물을 우회한 충돌 검사 segment 수)를, 장애물을 못 보는 직선거리
// effort 추정 및 실제 현직 해의 비용·effort와 나란히 재려고 스칼라만 뽑는다.
export function eitStarReadout(opts: EITStarOptions): {
    costToGo: number; effortToGo: number; straightEffort: number;
    pathCost: number; pathEffort: number; success: boolean;
} {
    const r = eitStar(opts)
    return {costToGo: r.costToGoStart, effortToGo: r.effortToGoStart,
            straightEffort: r.straightEffort, pathCost: r.pathCost,
            pathEffort: r.pathEffort, success: r.success}
}
