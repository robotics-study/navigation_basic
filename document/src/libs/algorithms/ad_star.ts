import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 AD*. 저장소 구현과 같은 improve → move → sense → repair 루프를
// 돌고 같은 trace 이벤트를 방출한다 (Likhachev, Ferguson, Gordon, Stentz & Thrun 2005).
// D* Lite 뼈대(backward g/rhs + k_m) 위에 ARA*의 ε-팽창 키와 INCONS를 얹은 구조다.
export interface ADStarOptions {
    map: GridMap;          // 실제 지도 (로봇은 모름)
    start: Cell;
    goal: Cell;
    epsStart: number;
    epsFinal: number;
    epsStep: number;
    sensorRadius: number;
}

const INF = Infinity;
const SQRT2 = Math.SQRT2;
type Key = [number, number];

const octile = (a: Cell, b: Cell): number => {
    const dr = Math.abs(a[0] - b[0])
    const dc = Math.abs(a[1] - b[1])
    const lo = Math.min(dr, dc)
    const hi = Math.max(dr, dc)
    return (hi - lo) + SQRT2 * lo
}

const keyLess = (a: Key, b: Key): boolean =>
    a[0] < b[0] || (a[0] === b[0] && a[1] < b[1])

export function runADStar({map, start, goal, epsStart, epsFinal, epsStep, sensorRadius}: ADStarOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "ad_star",
        params: {eps_start: epsStart, eps_final: epsFinal, eps_step: epsStep, sensor_radius: sensorRadius},
    })

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const cellOf = (i: number): Cell => [Math.floor(i / W), i % W]
    const trueBlocked = (c: Cell) => !inBounds(map, c[0], c[1]) || map.occupied[idx(c)]

    const blocked = new Set<number>()
    const g = new Map<number, number>()
    const rhs = new Map<number, number>()
    const keyOf = new Map<number, Key>()
    const closed = new Set<number>()
    const incons = new Set<number>()
    let open: Array<{key: Key; id: number; i: number}> = []
    let counter = 0
    let kM = 0
    let eps = Math.max(epsStart, epsFinal)
    let sStart: Cell = start
    let sLast: Cell = start
    let expanded = 0
    let replans = 0
    let sensed = 0

    const gOf = (c: Cell) => g.get(idx(c)) ?? INF
    const rhsOf = (c: Cell) => rhs.get(idx(c)) ?? INF
    const calcKey = (s: Cell): Key => {
        const gv = gOf(s)
        const rv = rhsOf(s)
        // over-consistent 정점만 ε를 곱한다 (인상은 admissible 키로 전파돼야 하므로).
        if (gv > rv) return [rv + eps * octile(sStart, s) + kM, rv]
        return [gv + octile(sStart, s) + kM, gv]
    }

    const DELTAS: Array<[number, number, number]> = [
        [-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
        [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2],
    ]
    const passable = (u: Cell): Array<[Cell, number]> => {
        const out: Array<[Cell, number]> = []
        for (const [dr, dc, cost] of DELTAS) {
            const c: Cell = [u[0] + dr, u[1] + dc]
            if (!inBounds(map, c[0], c[1]) || blocked.has(idx(c))) continue
            if (dr !== 0 && dc !== 0) {
                const a: Cell = [u[0] + dr, u[1]]
                const b: Cell = [u[0], u[1] + dc]
                if ((inBounds(map, a[0], a[1]) && blocked.has(idx(a)))
                    || (inBounds(map, b[0], b[1]) && blocked.has(idx(b)))) continue
            }
            out.push([c, cost])
        }
        return out
    }

    const queueInsert = (u: Cell, key: Key) => {
        keyOf.set(idx(u), key)
        open.push({key, id: counter++, i: idx(u)})
    }
    const peekTop = (): {key: Key; cell: Cell} | null => {
        let best: {key: Key; id: number; i: number} | null = null
        for (const e of open) {
            const live = keyOf.get(e.i)
            if (!live || live[0] !== e.key[0] || live[1] !== e.key[1]) continue
            if (!best || keyLess(e.key, best.key)
                || (!keyLess(best.key, e.key) && e.id < best.id)) best = e
        }
        return best ? {key: best.key, cell: cellOf(best.i)} : null
    }

    const updateState = (u: Cell) => {
        const ui = idx(u)
        if (u[0] !== goal[0] || u[1] !== goal[1]) {
            let best = INF
            let sbest: Cell | null = null
            let bestEdge = 0
            for (const [s2, cost] of passable(u)) {
                const v = cost + gOf(s2)
                if (v < best) {
                    best = v
                    sbest = s2
                    bestEdge = cost
                }
            }
            if (best !== rhsOf(u)) {
                rhs.set(ui, best)
                if (sbest && best < INF) {
                    emit({event: "candidate_evaluated", state: u, cost: best})
                    emit({event: "edge_added", state: u, parent: sbest, cost: bestEdge})
                }
            }
        }
        keyOf.delete(ui)
        incons.delete(ui)
        if (gOf(u) !== rhsOf(u)) {
            if (!closed.has(ui)) queueInsert(u, calcKey(u))
            else incons.add(ui)   // 이미 확장됨: 다음 reopen 으로 미룬다 (INCONS)
        }
    }

    const reopen = () => {
        const states = new Set<number>(incons)
        keyOf.forEach((_, i) => states.add(i))
        incons.clear()
        closed.clear()
        keyOf.clear()
        open = []
        Array.from(states).sort((a, b) => a - b).forEach((i) => queueInsert(cellOf(i), calcKey(cellOf(i))))
    }

    const computeOrImprovePath = () => {
        let guard = 0
        for (;;) {
            if (guard++ > map.width * map.height * 40) break
            const top = peekTop()
            if (!top) break
            if (!(keyLess(top.key, calcKey(sStart)) || rhsOf(sStart) !== gOf(sStart))) break
            const u = top.cell
            const ui = idx(u)
            keyOf.delete(ui)
            expanded++
            emit({event: "node_expanded", state: u, cost: Math.min(gOf(u), rhsOf(u))})
            if (gOf(u) > rhsOf(u)) {
                g.set(ui, rhsOf(u))
                closed.add(ui)
                for (const [s2] of passable(u)) updateState(s2)
            } else {
                g.set(ui, INF)
                updateState(u)
                for (const [s2] of passable(u)) updateState(s2)
            }
        }
    }

    const extractPlan = (): Cell[] | null => {
        if (gOf(sStart) === INF) return null
        const path: Cell[] = [sStart]
        const seen = new Set<number>([idx(sStart)])
        let cur = sStart
        while (cur[0] !== goal[0] || cur[1] !== goal[1]) {
            let best = INF
            let nxt: Cell | null = null
            for (const [s2, cost] of passable(cur)) {
                const v = cost + gOf(s2)
                if (v < best) {
                    best = v
                    nxt = s2
                }
            }
            if (!nxt || best === INF || seen.has(idx(nxt))) return null
            cur = nxt
            seen.add(idx(cur))
            path.push(cur)
        }
        return path
    }
    const publish = () => {
        const plan = extractPlan()
        if (plan) emit({event: "path_found", path: plan})
    }

    const sense = (robot: Cell): Cell[] => {
        const toUpdate: Cell[] = []
        const r = sensorRadius
        for (let dr = -r; dr <= r; dr++) {
            for (let dc = -r; dc <= r; dc++) {
                if (dr * dr + dc * dc > r * r && Math.max(Math.abs(dr), Math.abs(dc)) > 1) continue
                const c: Cell = [robot[0] + dr, robot[1] + dc]
                if (!inBounds(map, c[0], c[1])) continue
                if (blocked.has(idx(c)) || !trueBlocked(c)) continue
                blocked.add(idx(c))
                sensed++
                emit({event: "obstacle_revealed", state: c})
                for (const [n] of passable(c)) toUpdate.push(n)
            }
        }
        return toUpdate
    }

    rhs.set(idx(goal), 0)
    queueInsert(goal, calcKey(goal))
    for (const v of sense(sStart)) updateState(v)
    computeOrImprovePath()
    publish()

    const trajectory: Cell[] = [sStart]
    let realized = 0
    emit({event: "robot_moved", state: sStart})

    let reached = sStart[0] === goal[0] && sStart[1] === goal[1]
    let guard = 0
    while (!reached && guard++ < map.width * map.height * 8) {
        if (eps > epsFinal) {
            // Anytime 개선: ε를 조이고 reopen 후 수리. 로봇은 belief-최적 계획 전에는 안 움직인다.
            eps = Math.max(epsFinal, eps - epsStep)
            reopen()
            computeOrImprovePath()
            publish()
            continue
        }
        if (gOf(sStart) === INF) break
        let best = INF
        let nxt: Cell | null = null
        let stepCost = 0
        for (const [s2, cost] of passable(sStart)) {
            const cand = cost + gOf(s2)
            if (cand < best) {
                best = cand
                nxt = s2
                stepCost = cost
            }
        }
        if (!nxt || best === INF) break
        sStart = nxt
        realized += stepCost
        trajectory.push(sStart)
        emit({event: "robot_moved", state: sStart})
        if (sStart[0] === goal[0] && sStart[1] === goal[1]) {
            reached = true
            break
        }
        const changed = sense(sStart)
        if (changed.length > 0) {
            kM += octile(sLast, sStart)
            sLast = sStart
            for (const v of changed) updateState(v)
            // 감지된 변화는 유의미한 사건으로 취급: ε를 다시 올려 준최적 해를 빨리 얻고,
            // 이후 루프가 eps_final 까지 도로 조인다.
            eps = Math.max(epsStart, epsFinal)
            replans++
            reopen()
            computeOrImprovePath()
            publish()
        }
    }

    if (reached) emit({event: "path_found", path: trajectory, cost: realized})
    emit({
        event: "planning_finished",
        success: reached,
        metrics: {
            path_cost: reached ? realized : 0,
            expanded_nodes: expanded,
            replan_count: replans,
            sensed_cells: sensed,
        },
    })
    return events
}
