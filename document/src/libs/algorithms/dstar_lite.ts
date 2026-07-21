import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 D* Lite. 저장소 구현과 같은 move → sense → repair 루프를 돌고
// 같은 trace 이벤트를 방출한다 (Koenig & Likhachev 2002). 로봇은 지도를 모르는 채
// 출발하고, 센서 반경 안에서 발견한 벽만 belief 에 쌓으며 backward g/rhs 를 수리한다.
export interface DStarLiteOptions {
    map: GridMap;          // 실제 지도 (로봇은 모름)
    start: Cell;
    goal: Cell;
    sensorRadius: number;
}

// 재계획 시점의 belief 스냅샷 — sandbox 가 "매번 A* 재실행" 비용과 비교할 때 쓴다.
export interface ReplanSnapshot {
    blocked: Set<number>;  // belief (cell index)
    robot: Cell;
}

export interface DStarLiteRun {
    events: TraceEvent[];
    snapshots: ReplanSnapshot[];
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

export function runDStarLite({map, start, goal, sensorRadius}: DStarLiteOptions): DStarLiteRun {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "dstar_lite", params: {sensor_radius: sensorRadius}})

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const trueBlocked = (c: Cell) => !inBounds(map, c[0], c[1]) || map.occupied[idx(c)]

    const blocked = new Set<number>()           // belief
    const g = new Map<number, number>()
    const rhs = new Map<number, number>()
    const keyOf = new Map<number, Key>()
    const open: Array<{key: Key; id: number; cell: Cell}> = []
    let counter = 0
    let kM = 0
    let sStart: Cell = start
    let sLast: Cell = start
    let expanded = 0
    let replans = 0
    let sensed = 0
    const snapshots: ReplanSnapshot[] = []

    const gOf = (c: Cell) => g.get(idx(c)) ?? INF
    const rhsOf = (c: Cell) => rhs.get(idx(c)) ?? INF
    const calcKey = (s: Cell): Key => {
        const m = Math.min(gOf(s), rhsOf(s))
        return [m + octile(sStart, s) + kM, m]
    }

    const DELTAS: Array<[number, number, number]> = [
        [-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
        [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2],
    ]
    // belief 기준 통과 가능 이웃 (실제 지도가 아니라 로봇이 아는 세계).
    const passable = (u: Cell): Array<[Cell, number]> => {
        const out: Array<[Cell, number]> = []
        for (const [dr, dc, cost] of DELTAS) {
            const c: Cell = [u[0] + dr, u[1] + dc]
            if (!inBounds(map, c[0], c[1]) || blocked.has(idx(c))) continue
            // 대각 이동은 양 옆이 belief 상 뚫려 있어야 한다.
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
        open.push({key, id: counter++, cell: u})
    }
    const peekTop = (): {key: Key; cell: Cell} | null => {
        // 낡은 항목을 걷어내며 살아 있는 최소 키를 찾는다 (교육용 규모라 선형 탐색으로 충분).
        let best: {key: Key; id: number; cell: Cell} | null = null
        let bestAt = -1
        for (let i = 0; i < open.length; i++) {
            const e = open[i]
            const live = keyOf.get(idx(e.cell))
            if (!live || live[0] !== e.key[0] || live[1] !== e.key[1]) continue
            if (!best || keyLess(e.key, best.key) || (!keyLess(best.key, e.key) && e.id < best.id)) {
                best = e
                bestAt = i
            }
        }
        if (!best) {
            open.length = 0
            return null
        }
        open.splice(bestAt, 1)
        open.push(best)   // peek 이므로 유지 (pop 은 keyOf 삭제로 표현)
        return {key: best.key, cell: best.cell}
    }

    const updateVertex = (u: Cell) => {
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
                rhs.set(idx(u), best)
                if (sbest && best < INF) {
                    emit({event: "candidate_evaluated", state: u, cost: best})
                    emit({event: "edge_added", state: u, parent: sbest, cost: bestEdge})
                }
            }
        }
        keyOf.delete(idx(u))
        if (gOf(u) !== rhsOf(u)) queueInsert(u, calcKey(u))
    }

    const computeShortestPath = () => {
        for (;;) {
            const top = peekTop()
            if (!top) break
            const startKey = calcKey(sStart)
            if (!(keyLess(top.key, startKey) || rhsOf(sStart) !== gOf(sStart))) break
            const u = top.cell
            keyOf.delete(idx(u))
            expanded++
            emit({event: "node_expanded", state: u, cost: Math.min(gOf(u), rhsOf(u))})
            const kNew = calcKey(u)
            if (keyLess(top.key, kNew)) {
                queueInsert(u, kNew)
            } else if (gOf(u) > rhsOf(u)) {
                g.set(idx(u), rhsOf(u))
                for (const [s2] of passable(u)) updateVertex(s2)
            } else {
                g.set(idx(u), INF)
                updateVertex(u)
                for (const [s2] of passable(u)) updateVertex(s2)
            }
        }
    }

    const sense = (robot: Cell): Cell[] => {
        const toUpdate: Cell[] = []
        const r = sensorRadius
        for (let dr = -r; dr <= r; dr++) {
            for (let dc = -r; dc <= r; dc++) {
                // Euclidean disk + 로봇이 바로 밟을 수 있는 8-이웃은 항상 감지.
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

    // Initialize: goal 이 backward 탐색의 뿌리다.
    rhs.set(idx(goal), 0)
    queueInsert(goal, calcKey(goal))
    for (const v of sense(sStart)) updateVertex(v)
    computeShortestPath()

    const trajectory: Cell[] = [sStart]
    let realized = 0
    emit({event: "robot_moved", state: sStart})

    let reached = sStart[0] === goal[0] && sStart[1] === goal[1]
    let guard = 0
    while (!reached && guard++ < map.width * map.height * 4) {
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
            snapshots.push({blocked: new Set(blocked), robot: sStart})
            for (const v of changed) updateVertex(v)
            computeShortestPath()
            replans++
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
    return {events, snapshots}
}
