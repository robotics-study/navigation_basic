import {GridMap, isOccupied} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 ARA*. 저장소 구현과 같은 ε-스케줄 + INCONS 재사용 구조로
// 같은 trace 이벤트를 방출한다 (Likhachev, Gordon & Thrun 2003).
export interface ARAStarOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
    epsStart: number;
    epsFinal: number;
    epsStep: number;
}

// 반복별 통계 — sandbox가 "매번 weighted A* 재실행"과 비교할 때 쓴다.
export interface IterationStat {
    eps: number;
    expanded: number;   // 이 반복에서의 확장 수
    cost: number;       // 이 반복이 발표한 해의 비용
}

export interface ARAStarRun {
    events: TraceEvent[];
    iterations: IterationStat[];
}

const SQRT2 = Math.SQRT2;

const octile = (a: Cell, b: Cell): number => {
    const dr = Math.abs(a[0] - b[0])
    const dc = Math.abs(a[1] - b[1])
    return Math.max(dr, dc) + (SQRT2 - 1) * Math.min(dr, dc)
}

export function runARAStar({map, start, goal, epsStart, epsFinal, epsStep}: ARAStarOptions): ARAStarRun {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "ara_star",
        params: {eps_start: epsStart, eps_final: epsFinal, eps_step: epsStep},
    })

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const size = map.width * map.height
    const g = new Array<number>(size).fill(Infinity)
    const parent = new Array<number>(size).fill(-1)
    const closed = new Array<boolean>(size).fill(false)
    const inOpen = new Array<boolean>(size).fill(false)
    const incons = new Set<number>()
    let heap: Array<{key: number; id: number; i: number}> = []
    let counter = 0

    const DELTAS: Array<[number, number, number]> = [
        [-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
        [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2],
    ]
    const neighbors = (i: number): Array<[number, number]> => {
        const r = Math.floor(i / W)
        const c = i % W
        const out: Array<[number, number]> = []
        for (const [dr, dc, cost] of DELTAS) {
            const nr = r + dr
            const nc = c + dc
            if (isOccupied(map, nr, nc)) continue
            if (dr !== 0 && dc !== 0
                && (isOccupied(map, r + dr, c) || isOccupied(map, r, c + dc))) continue
            out.push([nr * W + nc, cost])
        }
        return out
    }
    const cellOf = (i: number): Cell => [Math.floor(i / W), i % W]
    const h = (i: number) => octile(cellOf(i), goal)

    const push = (i: number, key: number) => {
        heap.push({key, id: counter++, i})
        inOpen[i] = true
    }
    const popMin = (): {key: number; i: number} | null => {
        // 낡은 항목(open에서 빠진 것)은 건너뛴다. 교육용 규모라 선형 최소 탐색으로 충분.
        let bestAt = -1
        for (let k = 0; k < heap.length; k++) {
            const e = heap[k]
            if (!inOpen[e.i]) continue
            if (bestAt < 0 || e.key < heap[bestAt].key
                || (e.key === heap[bestAt].key && e.id < heap[bestAt].id)) bestAt = k
        }
        if (bestAt < 0) return null
        const e = heap[bestAt]
        heap.splice(bestAt, 1)
        return {key: e.key, i: e.i}
    }
    const minKey = (): number => {
        let best = Infinity
        for (const e of heap) if (inOpen[e.i] && e.key < best) best = e.key
        return best
    }

    const startI = idx(start)
    const goalI = idx(goal)
    let expanded = 0
    const iterations: IterationStat[] = []

    const improvePath = (eps: number): number => {
        const before = expanded
        for (;;) {
            const top = minKey()
            if (top === Infinity) break
            if (g[goalI] <= top) break
            const popped = popMin()
            if (!popped) break
            const s = popped.i
            inOpen[s] = false
            closed[s] = true
            expanded++
            emit({event: "node_expanded", state: cellOf(s), cost: g[s]})
            for (const [nb, cost] of neighbors(s)) {
                const tentative = g[s] + cost
                if (tentative < g[nb]) {
                    g[nb] = tentative
                    parent[nb] = s
                    emit({event: "candidate_evaluated", state: cellOf(nb), cost: tentative})
                    emit({event: "edge_added", state: cellOf(nb), parent: cellOf(s), cost})
                    if (!closed[nb]) {
                        push(nb, tentative + eps * h(nb))
                    } else {
                        // 이미 확장됐지만 개선됨: 지금 재확장하지 않고 다음 ε 반복으로 미룬다.
                        incons.add(nb)
                    }
                }
            }
        }
        return expanded - before
    }

    const reconstruct = (): Cell[] => {
        const path: Cell[] = []
        let p = goalI
        while (p >= 0) {
            path.unshift(cellOf(p))
            p = parent[p]
        }
        return path
    }

    let eps = Math.max(epsStart, epsFinal)
    g[startI] = 0
    push(startI, eps * h(startI))

    let success = false
    let guard = 0
    for (;;) {
        if (guard++ > 100) break
        const iterExpanded = improvePath(eps)
        if (g[goalI] < Infinity) {
            success = true
            const path = reconstruct()
            emit({event: "path_found", path, cost: g[goalI]})
            iterations.push({eps, expanded: iterExpanded, cost: g[goalI]})
        } else {
            break
        }
        if (eps <= epsFinal) break
        eps = Math.max(epsFinal, eps - epsStep)
        // INCONS∪OPEN을 조여진 ε 키로 다시 열고 CLOSED를 비운다.
        const reopen = new Set<number>(incons)
        for (const e of heap) if (inOpen[e.i]) reopen.add(e.i)
        incons.clear()
        closed.fill(false)
        heap = []
        reopen.forEach((i) => {
            inOpen[i] = false
            push(i, g[i] + eps * h(i))
        })
    }

    emit({
        event: "planning_finished",
        success,
        metrics: {
            path_cost: success ? g[goalI] : 0,
            expanded_nodes: expanded,
            iterations: iterations.length,
        },
    })
    return {events, iterations}
}
