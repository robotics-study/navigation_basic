import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 Anya (Harabor, Grastien, Öz & Aksakalli 2016). 저장소
// 구현을 그대로 미러한다: 턴 지점을 셀 중심이 아니라 grid corner(격자 꼭짓점)에
// 두고, corner root의 가시 영역을 행별 interval로 sweep 해 successor를 만든다.
// 평면 최단 경로는 볼록 장애물 모서리에서만 꺾이는 taut string 이므로 결과는 참
// 유클리드 최적이다. 같은 연산(sqrt, 동일 split/merge 순서, FIFO tie-break)을
// 쓰므로 확장 수까지 python demo와 일치한다.
export interface AnyaOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
    vertexEpsilon: number;
}

// 기하 좌표계 (x = col + 0.5, y = row + 0.5). 셀 (r, c)는 x∈[c, c+1], y∈[r, r+1]의
// 단위 정사각형이고, grid corner는 정수 (x, y) 격자점이다.
type Point = [number, number];

const euclid = (ax: number, ay: number, bx: number, by: number): number => {
    const dx = ax - bx
    const dy = ay - by
    return Math.sqrt(dx * dx + dy * dy)
}

const DELTAS: Array<[number, number]> = [
    [-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [-1, 1], [1, -1], [1, 1],
]

export interface AnyaResult {
    events: TraceEvent[];
    // 최종 경로의 실제 corner 기하, 셀 인덱스 프레임의 연속 좌표 (row, col).
    // trace의 path는 셀로 스냅되지만, sandbox는 corner를 그대로 그린다.
    geometry: Array<[number, number]>;
}

export const runAnya = (opts: AnyaOptions): TraceEvent[] => runAnyaFull(opts).events

export function runAnyaFull(opts: AnyaOptions): AnyaResult {
    const {map, start, goal, vertexEpsilon: eps} = opts
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "anya", params: {vertex_epsilon: eps}})

    // 도달 가능한 free 성분 (occupancy는 이 집합으로만 관찰한다).
    const W = map.width
    const freeCell0 = (r: number, c: number) =>
        inBounds(map, r, c) && !map.occupied[r * W + c]
    const free = new Set<number>()
    const key = (r: number, c: number) => r * (W + 4) + c + 2
    {
        const stack: Cell[] = [start]
        free.add(key(start[0], start[1]))
        while (stack.length > 0) {
            const s = stack.pop()!
            for (const [dr, dc] of DELTAS) {
                const nr = s[0] + dr
                const nc = s[1] + dc
                if (!freeCell0(nr, nc)) continue
                if (dr !== 0 && dc !== 0
                    && !(freeCell0(s[0] + dr, s[1]) && freeCell0(s[0], s[1] + dc))) continue
                if (!free.has(key(nr, nc))) {
                    free.add(key(nr, nc))
                    stack.push([nr, nc])
                }
            }
        }
    }
    // (cx, cy) = min-corner가 (x=cx, y=cy)인 셀 == (row=cy, col=cx).
    const cellFree = (cx: number, cy: number) =>
        cy >= 0 && cy < map.height && cx >= 0 && cx < W && free.has(key(cy, cx))

    let r0 = Infinity
    let r1 = -Infinity
    let c0 = Infinity
    let c1 = -Infinity
    free.forEach((k) => {
        const r = Math.floor(k / (W + 4))
        const c = (k % (W + 4)) - 2
        if (r < r0) r0 = r
        if (r > r1) r1 = r
        if (c < c0) c0 = c
        if (c > c1) c1 = c
    })

    // 격자 꼭짓점이 free/blocked를 섞어 접하면 turning point 후보(모서리)다.
    const isCorner = (x: number, y: number): boolean => {
        const blocked = [
            !cellFree(x - 1, y - 1), !cellFree(x, y - 1),
            !cellFree(x - 1, y), !cellFree(x, y),
        ]
        return blocked.some(Boolean) && !blocked.every(Boolean)
    }

    const projX = (rx: number, ry: number, x0: number, y0: number, yn: number): number =>
        rx + (x0 - rx) * (yn - ry) / (y0 - ry)

    // 열린 선분 p→q가 유효한 any-angle 이동인지: blocked 셀 내부를 지나지 않고,
    // 양쪽이 모두 blocked 인 grid edge 위를 달리지 않으며, blocked pinch corner 를
    // 관통하지 않는다. 정수 grid line 위에 놓인 선분은 경계만 스치므로 한쪽 셀만
    // free 면 유효하다 (모서리를 스치는 taut leg를 허용해야 최적이 유지된다).
    const segClear = (p: Point, q: Point): boolean => {
        const [px, py] = p
        const [qx, qy] = q
        if (px === qx && py === qy) return true
        const dx = qx - px
        const dy = qy - py
        const ts = new Set<number>([0, 1])
        if (dx !== 0) {
            const lo = Math.min(px, qx)
            const hi = Math.max(px, qx)
            for (let xi = Math.ceil(lo); xi <= Math.floor(hi); xi++) ts.add((xi - px) / dx)
        }
        if (dy !== 0) {
            const lo = Math.min(py, qy)
            const hi = Math.max(py, qy)
            for (let yi = Math.ceil(lo); yi <= Math.floor(hi); yi++) ts.add((yi - py) / dy)
        }
        const ordered = Array.from(ts).filter((t) => t >= 0 && t <= 1).sort((a, b) => a - b)
        for (let i = 0; i + 1 < ordered.length; i++) {
            const a = ordered[i]
            const b = ordered[i + 1]
            if (b - a < 1e-12) continue
            const tm = 0.5 * (a + b)
            const mx = px + tm * dx
            const my = py + tm * dy
            if (dx === 0 && Math.abs(mx - Math.round(mx)) < eps) {
                const xi = Math.round(mx)
                const row = Math.floor(my)
                if (!cellFree(xi - 1, row) && !cellFree(xi, row)) return false
            } else if (dy === 0 && Math.abs(my - Math.round(my)) < eps) {
                const yi = Math.round(my)
                const col = Math.floor(mx)
                if (!cellFree(col, yi - 1) && !cellFree(col, yi)) return false
            } else if (!cellFree(Math.floor(mx), Math.floor(my))) {
                return false
            }
        }
        if (dx !== 0 && dy !== 0) {
            for (const tt of ordered) {
                if (tt <= eps || tt >= 1 - eps) continue
                const x = px + tt * dx
                const y = py + tt * dy
                if (Math.abs(x - Math.round(x)) < eps && Math.abs(y - Math.round(y)) < eps) {
                    const ix = Math.round(x)
                    const iy = Math.round(y)
                    if ((dx > 0) === (dy > 0)) {
                        if (!cellFree(ix - 1, iy) && !cellFree(ix, iy - 1)) return false
                    } else if (!cellFree(ix - 1, iy - 1) && !cellFree(ix, iy)) {
                        return false
                    }
                }
            }
        }
        return true
    }

    const merge = (pieces: Array<[number, number]>): Array<[number, number]> => {
        if (pieces.length === 0) return []
        const sorted = [...pieces].sort((u, v) => u[0] - v[0] || u[1] - v[1])
        const out: Array<[number, number]> = [[sorted[0][0], sorted[0][1]]]
        for (let i = 1; i < sorted.length; i++) {
            const [a, b] = sorted[i]
            if (a <= out[out.length - 1][1] + eps) {
                out[out.length - 1][1] = Math.max(out[out.length - 1][1], b)
            } else {
                out.push([a, b])
            }
        }
        return out
    }

    // 행 yn 위 [lo, hi] 중 root에서 온전히 보이는 최대 x-부분구간들. split 후보가
    // 가시성이 바뀔 수 있는 모든 지점을 표시하고, 균질한 조각마다 표본 하나로 검증한다.
    const clearPieces = (
        root: Point, yn: number, lo: number, hi: number, splits: number[],
    ): Array<[number, number]> => {
        const pts = new Set<number>([lo, hi])
        for (const s of splits) {
            if (lo - eps <= s && s <= hi + eps) pts.add(Math.min(Math.max(s, lo), hi))
        }
        const ordered = Array.from(pts).sort((a, b) => a - b)
        const pieces: Array<[number, number]> = []
        for (let i = 0; i + 1 < ordered.length; i++) {
            const a = ordered[i]
            const b = ordered[i + 1]
            if (b - a < 1e-7) continue
            if (segClear(root, [0.5 * (a + b), yn])) pieces.push([a, b])
        }
        return merge(pieces)
    }

    // corner ↔ cell (trace의 list[Cell] 계약용): 시작/goal은 자기 셀, corner 는
    // 고정 순서의 첫 free 인접 셀로 스냅한다. 비용은 corner 기하 그대로다.
    const cellOfPoint = (p: Point): Cell => {
        const row = p[1] - 0.5
        const col = p[0] - 0.5
        if (Math.abs(row - Math.round(row)) < 1e-6 && Math.abs(col - Math.round(col)) < 1e-6) {
            return [Math.round(row), Math.round(col)]
        }
        const x = Math.round(p[0])
        const y = Math.round(p[1])
        for (const [cx, cy] of [[x - 1, y - 1], [x - 1, y], [x, y - 1], [x, y]] as Array<[number, number]>) {
            if (cellFree(cx, cy)) return [cy, cx]
        }
        return [y, x]
    }

    type Found = Map<string, {pt: Point; interval: [number, number, number]}>
    const emitCorners = (root: Point, y: number, beams: Array<[number, number]>, found: Found) => {
        const iy = Math.round(y)
        for (const [a, b] of beams) {
            for (let x = Math.ceil(a - 1e-6); x <= Math.floor(b + 1e-6); x++) {
                if (isCorner(x, iy) && segClear(root, [x, y])) {
                    const k = `${x},${y}`
                    if (!found.has(k)) found.set(k, {pt: [x, y], interval: [y, a, b]})
                }
            }
        }
    }

    // successor 생성: cone(행별 interval 투영) + flat(자기 행 위 corner 걷기).
    const successors = (root: Point): Array<{pt: Point; interval: [number, number, number]}> => {
        const [rx, ry] = root
        const found: Found = new Map()
        const spanLo = c0 - 2
        const spanHi = c1 + 3

        for (const direction of [1, -1]) {
            let y = direction > 0 ? Math.floor(ry) + 1 : Math.ceil(ry) - 1
            const splits: number[] = []
            for (let e = Math.trunc(spanLo) - 1; e <= Math.trunc(spanHi) + 1; e++) splits.push(e)
            let beams = clearPieces(root, y, spanLo, spanHi, splits)
            emitCorners(root, y, beams, found)
            let steps = 0
            while (beams.length > 0 && r0 - 2 <= y && y <= r1 + 2 && steps < 400) {
                const yn = y + direction
                const child: Array<[number, number]> = []
                for (const [a, b] of beams) {
                    const an = projX(rx, ry, a, y, yn)
                    const bn = projX(rx, ry, b, y, yn)
                    const lo = Math.min(an, bn)
                    const hi = Math.max(an, bn)
                    const candSplits: number[] = []
                    for (let e = Math.floor(lo) - 1; e <= Math.floor(hi) + 1; e++) {
                        candSplits.push(e)
                        candSplits.push(projX(rx, ry, e, y, yn))
                    }
                    child.push(...clearPieces(root, yn, lo, hi, candSplits))
                }
                y = yn
                beams = merge(child)
                emitCorners(root, y, beams, found)
                steps++
            }
        }

        if (Math.abs(ry - Math.round(ry)) < eps) {
            const yr = Math.round(ry)
            for (const direction of [1, -1]) {
                let x = Math.round(rx)
                let steps = 0
                while (c0 - 2 <= x && x <= c1 + 2 && steps < 400) {
                    const col = direction > 0 ? x : x - 1
                    if (!cellFree(col, yr - 1) && !cellFree(col, yr)) break
                    x += direction
                    steps++
                    if (isCorner(x, yr) && segClear(root, [x, yr])) {
                        const k = `${x},${yr}`
                        if (!found.has(k)) found.set(k, {pt: [x, yr], interval: [yr, x, x]})
                    }
                }
            }
        }

        found.delete(`${rx},${ry}`)
        // 두 언어가 동일하게 확장하도록 (y, x) 순으로 정렬한 결정적 순서.
        return Array.from(found.values())
            .sort((u, v) => u.pt[1] - v.pt[1] || u.pt[0] - v.pt[0])
    }

    // --- A* over corner roots ---
    const sPt: Point = [start[1] + 0.5, start[0] + 0.5]
    const gPt: Point = [goal[1] + 0.5, goal[0] + 0.5]
    const h = (p: Point) => euclid(p[0], p[1], gPt[0], gPt[1])
    const pKey = (p: Point) => `${p[0]},${p[1]}`

    const g = new Map<string, number>([[pKey(sPt), 0]])
    const parent = new Map<string, Point>([[pKey(sPt), sPt]])
    const settled = new Set<string>()
    const heap: Array<{f: number; id: number; pt: Point}> = []
    let counter = 0
    const push = (pt: Point, f: number) => heap.push({f, id: counter++, pt})
    const popMin = (): {f: number; pt: Point} | null => {
        let bestAt = -1
        for (let k = 0; k < heap.length; k++) {
            const e = heap[k]
            if (bestAt < 0 || e.f < heap[bestAt].f
                || (e.f === heap[bestAt].f && e.id < heap[bestAt].id)) bestAt = k
        }
        if (bestAt < 0) return null
        const e = heap.splice(bestAt, 1)[0]
        return {f: e.f, pt: e.pt}
    }

    push(sPt, h(sPt))
    let goalCost = Infinity
    let goalRoot: Point | null = null
    let expanded = 0

    while (heap.length > 0) {
        const top = popMin()
        if (!top) break
        const root = top.pt
        const rk = pKey(root)
        if (settled.has(rk)) continue
        if (top.f >= goalCost - eps) break
        settled.add(rk)
        expanded++
        emit({event: "node_expanded", state: cellOfPoint(root), cost: g.get(rk)!})
        // 마지막 leg: goal이 직접 보이는 corner는 경로를 닫는다.
        if (segClear(root, gPt)) {
            const cand = g.get(rk)! + euclid(root[0], root[1], gPt[0], gPt[1])
            if (cand < goalCost) {
                goalCost = cand
                goalRoot = root
            }
        }
        for (const {pt: corner, interval} of successors(root)) {
            const nd = g.get(rk)! + euclid(root[0], root[1], corner[0], corner[1])
            const ck = pKey(corner)
            const old = g.get(ck)
            if (old === undefined || nd < old - eps) {
                g.set(ck, nd)
                parent.set(ck, root)
                const cCell = cellOfPoint(corner)
                emit({event: "candidate_evaluated", state: cCell, cost: nd})
                emit({
                    event: "edge_added", state: cCell, parent: cellOfPoint(root),
                    cost: euclid(root[0], root[1], corner[0], corner[1]),
                    data: {row: interval[0] - 0.5, col_lo: interval[1] - 0.5, col_hi: interval[2] - 0.5},
                })
                push(corner, nd + h(corner))
            }
        }
    }

    if (goalRoot === null) {
        emit({event: "planning_finished", success: false, metrics: {expanded_nodes: expanded}})
        return {events, geometry: []}
    }
    const chain: Point[] = [goalRoot]
    {
        let node = goalRoot
        while (pKey(node) !== pKey(sPt)) {
            node = parent.get(pKey(node))!
            chain.push(node)
        }
        chain.reverse()
    }
    const path: Cell[] = [start]
    for (const pt of chain) {
        const cell = cellOfPoint(pt)
        const last = path[path.length - 1]
        if (cell[0] !== last[0] || cell[1] !== last[1]) path.push(cell)
    }
    const last = path[path.length - 1]
    if (last[0] !== goal[0] || last[1] !== goal[1]) path.push(goal)
    emit({event: "path_found", path, cost: goalCost})
    emit({
        event: "planning_finished",
        success: true,
        metrics: {path_cost: goalCost, expanded_nodes: expanded},
    })
    const geometry: Array<[number, number]> = [...chain, gPt].map((p) => [p[1] - 0.5, p[0] - 0.5])
    return {events, geometry}
}
