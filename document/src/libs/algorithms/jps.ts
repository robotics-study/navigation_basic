import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 JPS. 저장소 구현(no-corner-cutting 변형, Harabor & Grastien
// 2011)을 연산·tie-break·방향 순서까지 미러해 python demo와 확장 수가 일치한다.
export interface JPSOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
}

const SQRT2 = Math.SQRT2;
const DIRS_8: Array<[number, number]> = [
    [-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [-1, 1], [1, -1], [1, 1],
]

const octile = (a: Cell, b: Cell): number => {
    const dr = Math.abs(a[0] - b[0])
    const dc = Math.abs(a[1] - b[1])
    const lo = Math.min(dr, dc)
    const hi = Math.max(dr, dc)
    return (hi - lo) + SQRT2 * lo
}

const sign = (x: number): number => (x > 0 ? 1 : 0) - (x < 0 ? 1 : 0)

export function runJPS({map, start, goal}: JPSOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "jps", params: {}})

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const blocked = (r: number, c: number) => !inBounds(map, r, c) || map.occupied[r * W + c]
    const free = (r: number, c: number) => !blocked(r, c)

    // (r,c)에서 (dr,dc) 방향으로 합법적인 한 칸씩 전진하며 첫 jump point를 찾는다.
    const scan = (r: number, c: number, dr: number, dc: number): Cell | null => {
        const diagonal = dr !== 0 && dc !== 0
        for (;;) {
            if (blocked(r + dr, c + dc)) return null
            if (diagonal && (blocked(r + dr, c) || blocked(r, c + dc))) return null
            r += dr
            c += dc
            if (r === goal[0] && c === goal[1]) return [r, c]
            if (!diagonal) {
                if (dc !== 0) {   // 수평: 진행 방향 대각 뒤의 장애물이 옆 칸을 연다
                    if ((free(r - 1, c) && blocked(r - 1, c - dc))
                        || (free(r + 1, c) && blocked(r + 1, c - dc))) return [r, c]
                } else {          // 수직
                    if ((free(r, c - 1) && blocked(r - dr, c - 1))
                        || (free(r, c + 1) && blocked(r - dr, c + 1))) return [r, c]
                }
            } else if (scan(r, c, dr, 0) !== null || scan(r, c, 0, dc) !== null) {
                // 직교 스캔이 jump point를 찾는 대각 칸은 그 자체가 jump point다.
                return [r, c]
            }
        }
    }

    // u에 도달한 방향에 따라 뛰어 볼 방향들 (시작은 전방향).
    const successorDirs = (u: Cell, par: Cell | undefined): Array<[number, number]> => {
        if (!par) return DIRS_8
        const [r, c] = u
        const pdr = sign(r - par[0])
        const pdc = sign(c - par[1])
        if (pdr !== 0 && pdc !== 0) return [[pdr, 0], [0, pdc], [pdr, pdc]]
        const dirs: Array<[number, number]> = [[pdr, pdc]]
        if (pdc !== 0) {
            if (free(r - 1, c) && blocked(r - 1, c - pdc)) dirs.push([-1, pdc], [-1, 0])
            if (free(r + 1, c) && blocked(r + 1, c - pdc)) dirs.push([1, pdc], [1, 0])
        } else {
            if (free(r, c - 1) && blocked(r - pdr, c - 1)) dirs.push([pdr, -1], [0, -1])
            if (free(r, c + 1) && blocked(r - pdr, c + 1)) dirs.push([pdr, 1], [0, 1])
        }
        return dirs
    }

    const g = new Map<number, number>()
    const parent = new Map<number, Cell>()
    const closed = new Set<number>()
    const heap: Array<{key: number; id: number; cell: Cell}> = []
    let counter = 0
    const push = (cell: Cell, key: number) => heap.push({key, id: counter++, cell})
    const popMin = (): Cell | null => {
        let bestAt = -1
        for (let k = 0; k < heap.length; k++) {
            const e = heap[k]
            if (bestAt < 0 || e.key < heap[bestAt].key
                || (e.key === heap[bestAt].key && e.id < heap[bestAt].id)) bestAt = k
        }
        if (bestAt < 0) return null
        return heap.splice(bestAt, 1)[0].cell
    }

    g.set(idx(start), 0)
    push(start, octile(start, goal))
    let expanded = 0
    let found = false

    while (heap.length > 0) {
        const u = popMin()
        if (!u) break
        const ui = idx(u)
        if (closed.has(ui)) continue
        closed.add(ui)
        expanded++
        emit({event: "node_expanded", state: u, cost: g.get(ui)!})
        if (u[0] === goal[0] && u[1] === goal[1]) {
            found = true
            break
        }
        for (const [dr, dc] of successorDirs(u, parent.get(ui))) {
            const jp = scan(u[0], u[1], dr, dc)
            if (!jp || closed.has(idx(jp))) continue
            const tentative = g.get(ui)! + octile(u, jp)
            const old = g.get(idx(jp))
            if (old === undefined || tentative < old) {
                g.set(idx(jp), tentative)
                parent.set(idx(jp), u)
                emit({event: "candidate_evaluated", state: jp, cost: tentative})
                emit({event: "edge_added", state: jp, parent: u, cost: octile(u, jp)})
                push(jp, tentative + octile(jp, goal))
            }
        }
    }

    if (found) {
        // jump point 사이의 직선/대각 중간 셀을 채워 전체 경로로 만든다 (A*와 동일 형식).
        const jumps: Cell[] = [goal]
        let node = goal
        while (node[0] !== start[0] || node[1] !== start[1]) {
            node = parent.get(idx(node))!
            jumps.push(node)
        }
        jumps.reverse()
        const path: Cell[] = [start]
        for (let i = 1; i < jumps.length; i++) {
            const a = jumps[i - 1]
            const b = jumps[i]
            const dr = sign(b[0] - a[0])
            const dc = sign(b[1] - a[1])
            let cur = a
            while (cur[0] !== b[0] || cur[1] !== b[1]) {
                cur = [cur[0] + dr, cur[1] + dc]
                path.push(cur)
            }
        }
        emit({event: "path_found", path, cost: g.get(idx(goal))!})
        emit({
            event: "planning_finished",
            success: true,
            metrics: {path_cost: g.get(idx(goal))!, expanded_nodes: expanded},
        })
    } else {
        emit({event: "planning_finished", success: false, metrics: {expanded_nodes: expanded}})
    }
    return events
}
