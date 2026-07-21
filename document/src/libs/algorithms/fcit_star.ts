import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {informedSample} from "./informed_rrt_star";

// 브라우저 라이브 데모용 FCIT* — Fully Connected Informed Trees (Wilson, Thomason,
// Kingston, Kavraki & Gammell 2025, ICRA). 저장소 python 구현을 연산 순서·RNG·tie-break·이벤트 순서까지
// 그대로 미러한다. AIT*와의 차이는 후보 그래프다: AIT*는 표본 위 반경 RGG(줄어드는
// r_n)로 간선 수를 묶는데, FCIT*는 반경을 버리고 누적된 모든 표본을 서로 잇는 완전
// 연결 그래프를 탐색한다. 반경이 막던 먼 두 표본 사이 직행 간선을 허용해, 반경
// 그래프가 놓치는 지름길을 A* 정렬로 잡는다. 대신 간선 수가 O(n^2)이라 배치 크기를
// 작게 유지한다. AIT*처럼 goal로부터의 역방향 Dijkstra가 장애물 인지 heuristic을
// 만들고, 전방 A*가 간선을 lazy 검증하며 무효 간선을 invalid_edges에 영구 누적한다.
//
// 표본은 두 RNG 스트림에서 나온다: 아직 해가 없으면 map RNG(space.sample())로 전
// 공간을, 해가 생기면 planner RNG로 informed 타원 안을 뽑는다. informed_sample이 두
// 스트림을 나눠 쓰므로 같은 seed면 python demo와 표본·간선·확장 수까지 일치한다.
export interface FCITStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    batchSize: number;
    maxBatches: number;
    seed: number;
}

const INF = Infinity
const START = 0  // 전방 탐색의 뿌리(시작점)의 영구 index
const GOAL = 1   // 역방향 탐색의 원천(goal 표본)의 영구 index

// 무향 간선 키: 충돌이 어느 방향으로 검출됐든 같은 간선은 같은 키를 갖는다.
const edgeKey = (i: number, j: number): string => (i < j ? `${i}-${j}` : `${j}-${i}`)

// (key, idx) 사전순 최소를 뽑는 이진 힙. idx가 유일하므로 python heapq의 pop 순서
// (같은 key는 idx 오름차순)와 정확히 일치한다. 역방향은 (dist, idx), 전방은 (f, idx)
// 2튜플 pop이라 두 탐색 모두 이 힙 하나로 환원된다. 전방 탐색은 pop한 key까지 필요하므로
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

// 역방향 heuristic: goal에서 완전 연결 그래프(무효 간선 제외) 위로 도는 Dijkstra.
// 낙관적이다 — 남은 간선이 충돌 없다고 가정하며 충돌 검사는 하지 않는다. 검증은
// 전방 탐색의 몫이고, 그 발견이 다음 배치의 이 그래프를 좁힌다 (Strub & Gammell 2020).
// nbr가 이미 invalid_edges를 걸러 두었으므로 여기서는 다시 검사하지 않는다.
function reverseSearch(space: SamplingGrid, points: Point[], nbr: number[][]): number[] {
    const n = points.length
    const dist = new Array<number>(n).fill(INF)
    const settled = new Array<boolean>(n).fill(false)
    dist[GOAL] = 0
    const pq = new MinHeap()
    pq.push(0, GOAL)
    while (pq.size > 0) {
        const [d, u] = pq.pop()
        if (settled[u]) continue
        settled[u] = true
        const pu = points[u]
        for (const v of nbr[u]) {
            if (settled[v]) continue
            const nd = d + space.distance(pu, points[v])
            if (nd < dist[v]) {
                dist[v] = nd
                pq.push(nd, v)
            }
        }
    }
    return dist
}

export function runFCITStar(opts: FCITStarOptions): TraceEvent[] {
    const {map, start, goal, batchSize, maxBatches, seed} = opts
    const space = new SamplingGrid(map, seed)
    // planner 자체 draw(informed 타원)용 스트림 — map RNG와 분리되지만 같은 seed다.
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "fcit_star",
          params: {batch_size: batchSize, max_batches: maxBatches, seed}})

    // 배치를 넘어 남는 상태는 셋뿐이다: 자라는 표본 배열(0=start, 1=goal), 적응형
    // 무효 간선 집합, 현직 해 비용/경로. 나머지(g/parent/open-heap/역방향 heuristic)는
    // 배치마다 처음부터 다시 세운다.
    const points: Point[] = [start, goal]
    const invalidEdges = new Set<string>()
    let cBest = INF
    let bestPath: Point[] = []
    let expanded = 0

    for (let batch = 0; batch < maxBatches; batch++) {
        // --- 1. 배치를 키운다 (해가 생기면 informed 타원) --------------------
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

        // --- 2. 완전 연결 인접, 알려진 무효 간선만 제외 ----------------------
        // 반경 없음: FCIT*는 모든 표본을 서로 이어, 반경 RGG(AIT*/BIT*)가 놓치는
        // 지름길을 낼 수 있는 더 빽빽한 후보 그래프와 맞바꾼다 (Wilson et al. 2025).
        // 오름차순 index로 채워져 tie-break가 python과 같다.
        const nbr: number[][] = Array.from({length: n}, () => [])
        for (let i = 0; i < n; i++) {
            for (let j = 0; j < n; j++) {
                if (j !== i && !invalidEdges.has(edgeKey(i, j))) nbr[i].push(j)
            }
        }

        // --- 3. 역방향 탐색: goal로부터의 Dijkstra가 h_hat을 준다 -----------
        const hHat = reverseSearch(space, points, nbr)

        // --- 4. 전방 탐색: g + h_hat 위 lazy 검증 best-first ----------------
        const g = new Array<number>(n).fill(INF)
        const parent = new Array<number>(n).fill(-1)
        const closed = new Array<boolean>(n).fill(false)
        g[START] = 0
        const openHeap = new MinHeap()
        openHeap.push(hHat[START], START)

        while (openHeap.size > 0) {
            const [f, v] = openHeap.pop()
            if (closed[v]) continue
            // Lazy 삭제: 더 싼 g[v]로 대체된 낡은 항목을 건너뛴다.
            if (f > g[v] + hHat[v] + 1e-9) continue
            closed[v] = true
            expanded++
            if (v === GOAL) break  // 이번 배치 그래프에서 goal이 최종 비용으로 확정됐다
            const pv = points[v]
            for (const x of nbr[v]) {
                if (closed[x]) continue
                const edgeCost = space.distance(pv, points[x])
                const tentative = g[v] + edgeCost
                if (tentative >= g[x]) continue  // v를 거쳐도 x를 개선 못 한다
                emit({event: "candidate_evaluated", state: [points[x][0], points[x][1]],
                      cost: tentative})
                const ek = edgeKey(v, x)
                if (invalidEdges.has(ek)) continue
                if (!space.isMotionValid(pv, points[x])) {
                    invalidEdges.add(ek)  // 적응형 피드백: 다시는 고려하지 않는다
                    continue
                }
                const wasConnected = g[x] < INF
                g[x] = tentative
                parent[x] = v
                openHeap.push(tentative + hHat[x], x)
                if (wasConnected) {
                    emit({event: "rewire", state: [points[x][0], points[x][1]],
                          parent: [pv[0], pv[1]]})
                } else {
                    emit({event: "edge_added", state: [points[x][0], points[x][1]],
                          parent: [pv[0], pv[1]], cost: edgeCost})
                }
                if (x === GOAL && g[x] < cBest) {
                    cBest = g[x]
                    bestPath = extract(points, parent)
                    emit({event: "path_found", path: bestPath.map((p) => [p[0], p[1]])})
                }
            }
        }
    }

    const success = cBest < INF
    const path = success ? bestPath : []
    let cost = 0
    for (let i = 1; i < path.length; i++) {
        cost += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    }
    const n = points.length
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: success ? cost : 0, expanded_nodes: expanded,
                  samples: n, tree_size: n},
    })
    return events
}

function extract(points: Point[], parent: number[]): Point[] {
    const path: Point[] = []
    let node = GOAL
    while (node !== -1) {
        path.push(points[node])
        node = parent[node]
    }
    path.reverse()
    return path
}
