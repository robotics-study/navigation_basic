import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 Hybrid A*. 저장소 구현(Dolgov, Thrun, Montemerlo & Diebel
// 2008 단순화판: Euclidean h, analytic expansion 없음)을 미러한다. 상태는 연속
// SE(2) pose (x, y, θ)이고, closed set은 (x, y, θ) bin 위에서 관리된다.
// sin/cos 는 libm 구현별로 1 ULP 차이가 날 수 있어 python demo 와는 허용 오차
// 안에서만 일치한다 (parity 하니스에서 tolerance 로 검사).
export type Pose = [number, number, number];

export interface HybridAStarOptions {
    map: GridMap;
    start: Pose;
    goal: Pose;
    minTurnRadius: number;
    arcStep: number;
    numSteering: number;
    thetaBins: number;
    xyResolution: number;
    footprintRadius: number;
    allowReverse: boolean;
    reversePenalty: number;
    steerPenalty: number;
    goalPosTolerance: number;
    goalHeadingTolerance: number;
}

const TWO_PI = 2 * Math.PI;
const STRAIGHT_EPS = 1e-9;

const wrapAngle = (theta: number): number => theta - TWO_PI * Math.floor(theta / TWO_PI)

const angDiff = (a: number, b: number): number => {
    let d = wrapAngle(a - b)
    if (d > Math.PI) d -= TWO_PI
    return Math.abs(d)
}

// 일정 곡률 arc 를 부호 있는 길이만큼 정확히 적분한다.
export const integrate = (p: Pose, kappa: number, length: number): Pose => {
    const [x, y, theta] = p
    const theta2 = theta + kappa * length
    if (Math.abs(kappa) < STRAIGHT_EPS) {
        return [x + length * Math.cos(theta), y + length * Math.sin(theta), theta2]
    }
    return [
        x + (Math.sin(theta2) - Math.sin(theta)) / kappa,
        y - (Math.cos(theta2) - Math.cos(theta)) / kappa,
        theta2,
    ]
}

// 내접원 footprint 의 disc-셀 겹침 충돌 검사 — 저장소 OccupancyGrid2D.is_collision 미러.
const isCollision = (map: GridMap, radius: number, x: number, y: number): boolean => {
    const res = map.resolution
    const half = res * 0.5
    const worldToCell = (wx: number, wy: number): [number, number] => [
        map.height - 1 - Math.floor((wy - map.originY) / res),
        Math.floor((wx - map.originX) / res),
    ]
    const freeCell = (r: number, c: number) =>
        r >= 0 && r < map.height && c >= 0 && c < map.width && !map.occupied[r * map.width + c]
    const [loRow, loCol] = worldToCell(x - radius, y + radius)   // y+r → 작은 row
    const [hiRow, hiCol] = worldToCell(x + radius, y - radius)
    const r2 = radius * radius
    for (let row = loRow; row <= hiRow; row++) {
        for (let col = loCol; col <= hiCol; col++) {
            if (freeCell(row, col)) continue
            const cx = map.originX + (col + 0.5) * res
            const cy = map.originY + (map.height - 1 - row + 0.5) * res
            const dx = x - Math.min(Math.max(x, cx - half), cx + half)
            const dy = y - Math.min(Math.max(y, cy - half), cy + half)
            if (dx * dx + dy * dy <= r2) return true
        }
    }
    return false
}

export function runHybridAStar(opts: HybridAStarOptions): TraceEvent[] {
    const {map, start, goal} = opts
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "hybrid_astar",
        params: {
            min_turn_radius: opts.minTurnRadius, arc_step: opts.arcStep,
            num_steering: opts.numSteering, allow_reverse: opts.allowReverse,
        },
    })

    const kappaMax = 1 / opts.minTurnRadius
    const binTheta = TWO_PI / opts.thetaBins
    const nSub = Math.max(2, Math.ceil(opts.arcStep / opts.footprintRadius))
    const kappaSpan = (2 * kappaMax) / (opts.numSteering - 1)
    const primitives: Array<[number, number, boolean]> = []
    for (let i = 0; i < opts.numSteering; i++) {
        primitives.push([-kappaMax + i * kappaSpan, opts.arcStep, false])
    }
    if (opts.allowReverse) {
        for (let i = 0; i < opts.numSteering; i++) {
            primitives.push([-kappaMax + i * kappaSpan, -opts.arcStep, true])
        }
    }

    const binOf = (p: Pose): string =>
        `${Math.floor(p[0] / opts.xyResolution)},${Math.floor(p[1] / opts.xyResolution)},`
        + `${Math.floor(wrapAngle(p[2]) / binTheta) % opts.thetaBins}`
    const h = (p: Pose): number => {
        const dx = p[0] - goal[0]
        const dy = p[1] - goal[1]
        return Math.sqrt(dx * dx + dy * dy)
    }
    const isGoal = (p: Pose): boolean => {
        const dx = p[0] - goal[0]
        const dy = p[1] - goal[1]
        return dx * dx + dy * dy <= opts.goalPosTolerance ** 2
            && angDiff(p[2], goal[2]) <= opts.goalHeadingTolerance
    }

    const startBin = binOf(start)
    const g = new Map<string, number>([[startBin, 0]])
    const poseOf = new Map<string, Pose>([[startBin, start]])
    const cameFrom = new Map<string, {parent: string; subs: Pose[]}>()
    const closed = new Set<string>()
    const heap: Array<{key: number; id: number; bin: string}> = []
    let counter = 0
    const push = (bin: string, key: number) => heap.push({key, id: counter++, bin})
    const popMin = (): string | null => {
        let bestAt = -1
        for (let k = 0; k < heap.length; k++) {
            const e = heap[k]
            if (bestAt < 0 || e.key < heap[bestAt].key
                || (e.key === heap[bestAt].key && e.id < heap[bestAt].id)) bestAt = k
        }
        if (bestAt < 0) return null
        return heap.splice(bestAt, 1)[0].bin
    }

    push(startBin, h(start))
    let expanded = 0
    let found = false
    let goalBin = startBin
    let guard = 0

    while (heap.length > 0 && guard++ < 200000) {
        const b = popMin()
        if (!b) break
        if (closed.has(b)) continue
        closed.add(b)
        const p = poseOf.get(b)!
        expanded++
        emit({event: "node_expanded", state: [p[0], p[1], p[2]], cost: g.get(b)!})
        if (isGoal(p)) {
            found = true
            goalBin = b
            break
        }
        for (const [kappa, length, reverse] of primitives) {
            const subs: Pose[] = []
            let blocked = false
            for (let j = 1; j <= nSub; j++) {
                const s = integrate(p, kappa, length * (j / nSub))
                if (isCollision(map, opts.footprintRadius, s[0], s[1])) {
                    blocked = true
                    break
                }
                subs.push(s)
            }
            if (blocked) continue
            const child = subs[subs.length - 1]
            const b2 = binOf(child)
            if (closed.has(b2)) continue
            const absL = Math.abs(length)
            const cost = absL * (reverse ? opts.reversePenalty : 1)
                + opts.steerPenalty * Math.abs(kappa) * absL
            const cand = g.get(b)! + cost
            const old = g.get(b2)
            if (old === undefined || cand < old) {
                g.set(b2, cand)
                poseOf.set(b2, child)
                cameFrom.set(b2, {parent: b, subs})
                emit({event: "candidate_evaluated", state: [child[0], child[1], child[2]], cost: cand})
                const chord = absL / nSub
                let par: Pose = p
                for (const s of subs) {
                    emit({event: "edge_added", state: [s[0], s[1], s[2]],
                          parent: [par[0], par[1], par[2]], cost: chord})
                    par = s
                }
                push(b2, cand + h(child))
            }
        }
    }

    if (found) {
        const arcs: Pose[][] = []
        let b = goalBin
        while (b !== startBin) {
            const entry = cameFrom.get(b)!
            arcs.push(entry.subs)
            b = entry.parent
        }
        const path: Pose[] = [start]
        for (let i = arcs.length - 1; i >= 0; i--) path.push(...arcs[i])
        emit({
            event: "path_found",
            path: path.map((p) => [p[0], p[1], p[2]]),
            cost: g.get(goalBin)!,
        })
        emit({
            event: "planning_finished",
            success: true,
            metrics: {path_cost: g.get(goalBin)!, expanded_nodes: expanded},
        })
    } else {
        emit({event: "planning_finished", success: false, metrics: {expanded_nodes: expanded}})
    }
    return events
}
