import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";
import {lineOfSight} from "./theta_star";

// 브라우저 라이브 데모용 Visibility A*. 저장소 구현을 그대로 미러한다: cell-centre
// visibility graph(정점 = 시작의 연결 성분, 간선 = supercover LOS로 서로 보이는
// 쌍, 가중치 = 유클리드 직선거리) 위의 plain A*. 같은 연산(sqrt, LOS, FIFO
// tie-break, 행/열 오름차순 relaxation)을 쓰므로 확장 수까지 python demo와 일치한다.
export interface VisibilityAStarOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
    heuristicWeight: number;
}

const euclid = (a: Cell, b: Cell): number => {
    const dr = a[0] - b[0]
    const dc = a[1] - b[1]
    return Math.sqrt(dr * dr + dc * dc)
}

const DELTAS: Array<[number, number]> = [
    [-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [-1, 1], [1, -1], [1, 1],
]

const freeCell = (map: GridMap, r: number, c: number) =>
    inBounds(map, r, c) && !map.occupied[r * map.width + c]

// 8-connected 이웃 (corner cutting 금지) — 연결 성분 탐색에만 쓰므로 비용은 불필요.
const neighborCells = (map: GridMap, s: Cell): Cell[] => {
    const out: Cell[] = []
    for (const [dr, dc] of DELTAS) {
        const nr = s[0] + dr
        const nc = s[1] + dc
        if (!freeCell(map, nr, nc)) continue
        if (dr !== 0 && dc !== 0
            && !(freeCell(map, s[0] + dr, s[1]) && freeCell(map, s[0], s[1] + dc))) continue
        out.push([nr, nc])
    }
    return out
}

export function runVisibilityAStar(opts: VisibilityAStarOptions): TraceEvent[] {
    const {map, start, goal, heuristicWeight: w} = opts
    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const cellOf = (i: number): Cell => [Math.floor(i / W), i % W]
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "visibility_astar",
        params: {heuristic_weight: w},
    })

    const h = (c: Cell) => w * euclid(c, goal)

    // 후보 정점 = 시작의 연결 자유 성분. 도달 가능한 어떤 경로도 이 성분을 벗어나지
    // 않으므로 정점을 여기로 제한해도 visibility-graph 최적은 보존된다.
    const seen = new Set<number>([idx(start)])
    const stack: Cell[] = [start]
    while (stack.length > 0) {
        const cell = stack.pop()!
        for (const nb of neighborCells(map, cell)) {
            const i = idx(nb)
            if (!seen.has(i)) {
                seen.add(i)
                stack.push(nb)
            }
        }
    }
    const byRow = new Map<number, number[]>()
    seen.forEach((i) => {
        const r = Math.floor(i / W)
        const cols = byRow.get(r)
        if (cols) cols.push(i % W)
        else byRow.set(r, [i % W])
    })
    const rows = Array.from(byRow.keys()).sort((a, b) => a - b)
    for (const r of rows) byRow.get(r)!.sort((a, b) => a - b)

    const g = new Map<number, number>([[idx(start), 0]])
    const parent = new Map<number, number>([[idx(start), idx(start)]])
    const closed = new Set<number>()
    // 저장소와 같은 lazy heap + FIFO counter tie-break (선형 최소 탐색, 교육용 규모).
    const heap: Array<{key: number; id: number; i: number}> = []
    let counter = 0
    const push = (i: number, key: number) => heap.push({key, id: counter++, i})
    const popMin = (): number | null => {
        let bestAt = -1
        for (let k = 0; k < heap.length; k++) {
            const e = heap[k]
            if (bestAt < 0 || e.key < heap[bestAt].key
                || (e.key === heap[bestAt].key && e.id < heap[bestAt].id)) bestAt = k
        }
        if (bestAt < 0) return null
        return heap.splice(bestAt, 1)[0].i
    }

    // 웹 패널 전용 지표: LOS 검사 횟수 — 확장당 비용이 어디서 오는지 보여 준다.
    let losChecks = 0
    const los = (a: Cell, b: Cell): boolean => {
        losChecks++
        return lineOfSight(map, a, b)
    }

    const startI = idx(start)
    const goalI = idx(goal)
    push(startI, h(start))
    let expanded = 0
    let found = false
    const goalReachable = seen.has(goalI)

    while (heap.length > 0 && goalReachable) {
        const rootI = popMin()
        if (rootI === null) break
        if (closed.has(rootI)) continue
        closed.add(rootI)
        const root = cellOf(rootI)
        expanded++
        emit({event: "node_expanded", state: root, cost: g.get(rootI)!})
        if (rootI === goalI) {
            found = true
            break
        }
        // root의 가시 영역을 행별 interval로 투영해 그 안의 모든 셀을 relax 한다.
        // interval = 행에서 연속이고 전부 root에서 보이는 free 셀의 최대 run.
        const gRoot = g.get(rootI)!
        for (const row of rows) {
            const cols = byRow.get(row)!
            const n = cols.length
            let i = 0
            while (i < n) {
                if (!los(root, [row, cols[i]])) {
                    i++
                    continue
                }
                let j = i
                while (j + 1 < n && cols[j + 1] === cols[j] + 1
                    && los(root, [row, cols[j + 1]])) {
                    j++
                }
                const lo = cols[i]
                const hi = cols[j]
                for (let col = lo; col <= hi; col++) {
                    const ci = row * W + col
                    if (ci === rootI || closed.has(ci)) continue
                    const cell: Cell = [row, col]
                    const ecost = euclid(root, cell)
                    const cand = gRoot + ecost
                    const old = g.get(ci)
                    if (old === undefined || cand < old) {
                        g.set(ci, cand)
                        parent.set(ci, rootI)
                        emit({event: "candidate_evaluated", state: cell, cost: cand})
                        emit({event: "edge_added", state: cell, parent: root, cost: ecost,
                              data: {row, col_lo: lo, col_hi: hi}})
                        push(ci, cand + h(cell))
                    }
                }
                i = j + 1
            }
        }
    }

    if (found) {
        const path: Cell[] = []
        let p = goalI
        for (;;) {
            path.unshift(cellOf(p))
            const pp = parent.get(p)!
            if (pp === p) break
            p = pp
        }
        emit({event: "path_found", path, cost: g.get(goalI)!})
        emit({
            event: "planning_finished",
            success: true,
            metrics: {path_cost: g.get(goalI)!, expanded_nodes: expanded, los_checks: losChecks},
        })
    } else {
        emit({event: "planning_finished", success: false,
              metrics: {expanded_nodes: expanded, los_checks: losChecks}})
    }
    return events
}
