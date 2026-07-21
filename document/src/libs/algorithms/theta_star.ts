import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 Theta* / Lazy Theta*. 저장소 구현과 같은 연산(유클리드 sqrt,
// supercover LOS, FIFO tie-break, 이웃 순서)을 그대로 미러해 trace 이벤트·확장 수가
// python demo와 일치한다 (Nash, Daniel, Koenig & Felner 2007; Nash & Koenig 2010).
export interface ThetaStarOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
    heuristicWeight: number;
}

const SQRT2 = Math.SQRT2;

const euclid = (a: Cell, b: Cell): number => {
    const dr = a[0] - b[0]
    const dc = a[1] - b[1]
    return Math.sqrt(dr * dr + dc * dc)
}

const DELTAS: Array<[number, number, number]> = [
    [-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
    [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2],
]

const freeCell = (map: GridMap, r: number, c: number) =>
    inBounds(map, r, c) && !map.occupied[r * map.width + c]

const neighbors = (map: GridMap, s: Cell): Array<[Cell, number]> => {
    const out: Array<[Cell, number]> = []
    for (const [dr, dc, cost] of DELTAS) {
        const nr = s[0] + dr
        const nc = s[1] + dc
        if (!freeCell(map, nr, nc)) continue
        if (dr !== 0 && dc !== 0
            && !(freeCell(map, s[0] + dr, s[1]) && freeCell(map, s[0], s[1] + dc))) continue
        out.push([[nr, nc], cost])
    }
    return out
}

// 셀 중심 사이 직선의 supercover 통과 검사 (Amanatides & Woo 1987). 저장소의
// is_motion_valid와 동일한 규칙: 지나는 모든 셀이 free, 정확한 corner 교차는 양쪽
// 직교 셀이 모두 free 여야 한다. (row, col) 공간에서 x=col, y=row로 계산해도 대칭이라
// 결과는 같다.
export function lineOfSight(map: GridMap, a: Cell, b: Cell): boolean {
    const x0 = a[1] + 0.5
    const y0 = a[0] + 0.5
    const x1 = b[1] + 0.5
    const y1 = b[0] + 0.5
    let ix = Math.floor(x0)
    let iy = Math.floor(y0)
    const jx = Math.floor(x1)
    const jy = Math.floor(y1)
    if (!freeCell(map, iy, ix)) return false
    const dx = x1 - x0
    const dy = y1 - y0
    const stepX = dx > 0 ? 1 : -1
    const stepY = dy > 0 ? 1 : -1
    const tDeltaX = dx !== 0 ? Math.abs(1 / dx) : Infinity
    const tDeltaY = dy !== 0 ? Math.abs(1 / dy) : Infinity
    let tMaxX = dx !== 0
        ? (dx > 0 ? (Math.floor(x0) + 1 - x0) : (x0 - Math.floor(x0))) * tDeltaX
        : Infinity
    let tMaxY = dy !== 0
        ? (dy > 0 ? (Math.floor(y0) + 1 - y0) : (y0 - Math.floor(y0))) * tDeltaY
        : Infinity
    while (ix !== jx || iy !== jy) {
        if (iy === jy || tMaxX < tMaxY) {
            ix += stepX
            tMaxX += tDeltaX
        } else if (ix === jx || tMaxY < tMaxX) {
            iy += stepY
            tMaxY += tDeltaY
        } else {
            // 정확한 corner 교차: 양쪽 직교 셀이 모두 free 여야 통과.
            if (!(freeCell(map, iy, ix + stepX) && freeCell(map, iy + stepY, ix))) return false
            ix += stepX
            iy += stepY
            tMaxX += tDeltaX
            tMaxY += tDeltaY
        }
        if (!freeCell(map, iy, ix)) return false
    }
    return true
}

interface Ctx {
    map: GridMap;
    goal: Cell;
    w: number;
    events: TraceEvent[];
    seq: number;
    g: Map<number, number>;
    parent: Map<number, number>;
    closed: Set<number>;
    heap: Array<{key: number; id: number; i: number}>;
    counter: number;
}

const run = (opts: ThetaStarOptions, lazy: boolean): TraceEvent[] => {
    const {map, start, goal, heuristicWeight: w} = opts
    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const cellOf = (i: number): Cell => [Math.floor(i / W), i % W]
    const ctx: Ctx = {
        map, goal, w, events: [], seq: 0,
        g: new Map(), parent: new Map(), closed: new Set(), heap: [], counter: 0,
    }
    const emit = (ev: Omit<TraceEvent, "seq">) => ctx.events.push({seq: ctx.seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: lazy ? "lazy_theta_star" : "theta_star",
        params: {heuristic_weight: w},
    })
    // 웹 패널 전용 지표: LOS 검사 횟수 — lazy가 절약하는 것이 바로 이것이다.
    let losChecks = 0
    const los = (a: Cell, b: Cell): boolean => {
        losChecks++
        return lineOfSight(map, a, b)
    }
    const h = (c: Cell) => w * euclid(c, goal)
    // 저장소와 같은 lazy heap + FIFO counter tie-break (선형 최소 탐색, 교육용 규모).
    const push = (i: number, key: number) => ctx.heap.push({key, id: ctx.counter++, i})
    const popMin = (): number | null => {
        let bestAt = -1
        for (let k = 0; k < ctx.heap.length; k++) {
            const e = ctx.heap[k]
            if (bestAt < 0 || e.key < ctx.heap[bestAt].key
                || (e.key === ctx.heap[bestAt].key && e.id < ctx.heap[bestAt].id)) bestAt = k
        }
        if (bestAt < 0) return null
        return ctx.heap.splice(bestAt, 1)[0].i
    }

    const startI = idx(start)
    const goalI = idx(goal)
    ctx.g.set(startI, 0)
    ctx.parent.set(startI, startI)   // self-parent: 조부모 조회와 재구성 종료 지점
    push(startI, h(start))
    let expanded = 0
    let found = false

    while (ctx.heap.length > 0) {
        const si = popMin()
        if (si === null) break
        if (ctx.closed.has(si)) continue
        const s = cellOf(si)
        let p = cellOf(ctx.parent.get(si)!)

        if (lazy && ctx.parent.get(si) !== si && !los(p, s)) {
            // set_vertex — 낙관적 조부모가 실제로는 안 보였다면, 이미 settle 된 grid
            // 이웃 중 가장 싼 것으로 부모를 수리한다 (생성자는 항상 유효한 fallback).
            let bestG = Infinity
            let bestPar = ctx.parent.get(si)!
            let bestCost = 0
            for (const [s3, cost] of neighbors(map, s)) {
                const i3 = idx(s3)
                if (ctx.closed.has(i3)) {
                    const cand = ctx.g.get(i3)! + cost
                    if (cand < bestG) {
                        bestG = cand
                        bestPar = i3
                        bestCost = cost
                    }
                }
            }
            ctx.g.set(si, bestG)
            ctx.parent.set(si, bestPar)
            p = cellOf(bestPar)
            emit({event: "candidate_evaluated", state: s, cost: bestG})
            emit({event: "edge_added", state: s, parent: p, cost: bestCost})
        }

        ctx.closed.add(si)
        expanded++
        emit({event: "node_expanded", state: s, cost: ctx.g.get(si)!})
        if (si === goalI) {
            found = true
            break
        }
        for (const [s2, edgeCost] of neighbors(map, s)) {
            const i2 = idx(s2)
            if (ctx.closed.has(i2)) continue
            let cand: number
            let parI: number
            let ecost: number
            if (lazy) {
                // Lazy Path 2: LOS 검사 없이 조부모 가시를 낙관 가정 (검사는 pop 시로 연기).
                ecost = euclid(p, s2)
                cand = ctx.g.get(idx(p))! + ecost
                parI = idx(p)
            } else if (los(p, s2)) {
                // Path 2 — 조부모에서 직선 지름길.
                ecost = euclid(p, s2)
                cand = ctx.g.get(idx(p))! + ecost
                parI = idx(p)
            } else {
                // Path 1 — s를 거치는 표준 grid 스텝.
                ecost = edgeCost
                cand = ctx.g.get(si)! + ecost
                parI = si
            }
            const old = ctx.g.get(i2)
            if (old === undefined || cand < old) {
                ctx.g.set(i2, cand)
                ctx.parent.set(i2, parI)
                emit({event: "candidate_evaluated", state: s2, cost: cand})
                emit({event: "edge_added", state: s2, parent: cellOf(parI), cost: ecost})
                push(i2, cand + h(s2))
            }
        }
    }

    if (found) {
        const path: Cell[] = []
        let p2 = goalI
        for (;;) {
            path.unshift(cellOf(p2))
            const pp = ctx.parent.get(p2)!
            if (pp === p2) break
            p2 = pp
        }
        emit({event: "path_found", path, cost: ctx.g.get(goalI)!})
        emit({
            event: "planning_finished",
            success: true,
            metrics: {path_cost: ctx.g.get(goalI)!, expanded_nodes: expanded, los_checks: losChecks},
        })
    } else {
        emit({event: "planning_finished", success: false, metrics: {expanded_nodes: expanded, los_checks: losChecks}})
    }
    return ctx.events
}

export const runThetaStar = (opts: ThetaStarOptions): TraceEvent[] => run(opts, false)
export const runLazyThetaStar = (opts: ThetaStarOptions): TraceEvent[] => run(opts, true)
