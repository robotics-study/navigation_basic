import {GridMap, isOccupied} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 A*. C++/Python 구현과 같은 알고리즘·같은 trace 이벤트를 방출해
// 재생 패널을 공유한다 (spec/trace_schema.json 계약). 성능 측정용이 아니라 시각화용이다.
export interface AStarOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
    // f = g + w·h. 0 이면 Dijkstra, 1 이면 A*(최적 보장), 1 초과면 weighted A*.
    heuristicWeight: number;
    connectivity: 4 | 8;
}

const SQRT2 = Math.SQRT2;

// 8-connected 격자의 admissible heuristic (octile distance). 4-connected 는 Manhattan.
function heuristic(a: Cell, b: Cell, connectivity: 4 | 8): number {
    const dr = Math.abs(a[0] - b[0])
    const dc = Math.abs(a[1] - b[1])
    if (connectivity === 4) return dr + dc
    return Math.max(dr, dc) + (SQRT2 - 1) * Math.min(dr, dc)
}

export function runAStar({map, start, goal, heuristicWeight, connectivity}: AStarOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})

    emit({
        event: "planning_started",
        algorithm: "astar",
        params: {heuristic_weight: heuristicWeight, connectivity},
    })

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const g = new Array<number>(map.width * map.height).fill(Infinity)
    const parent = new Array<number>(map.width * map.height).fill(-1)
    const closed = new Array<boolean>(map.width * map.height).fill(false)
    const h = (c: Cell) => heuristicWeight * heuristic(c, goal, connectivity)

    // 데모 규모(수백 노드)에서는 배열 + 정렬로 충분하다. f 동률이면 g 큰 쪽 우선 —
    // unit grid 의 f-동률 corridor 에서 낭비 확장을 막는 표준 tie-break.
    const open: Array<{i: number; f: number; g: number}> = []
    const startI = idx(start)
    g[startI] = 0
    open.push({i: startI, f: h(start), g: 0})

    const deltas: Array<[number, number, number]> = connectivity === 4
        ? [[-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1]]
        : [[-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
           [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2]]

    let expandedCount = 0
    while (open.length > 0) {
        open.sort((a, b) => (a.f - b.f) || (b.g - a.g))
        const cur = open.shift()!
        if (closed[cur.i]) continue
        closed[cur.i] = true
        const cell: Cell = [Math.floor(cur.i / W), cur.i % W]
        expandedCount++
        emit({event: "node_expanded", state: cell, cost: g[cur.i]})

        if (cur.i === idx(goal)) {
            const path: Cell[] = []
            let p = cur.i
            while (p >= 0) {
                path.unshift([Math.floor(p / W), p % W])
                p = parent[p]
            }
            emit({event: "path_found", path, cost: g[cur.i]})
            emit({
                event: "planning_finished",
                success: true,
                metrics: {path_cost: g[cur.i], expanded_nodes: expandedCount},
            })
            return events
        }

        for (const [dr, dc, cost] of deltas) {
            const nr = cell[0] + dr
            const nc = cell[1] + dc
            if (isOccupied(map, nr, nc)) continue
            // 대각 이동은 양 옆이 모두 뚫려 있어야 한다 (모서리 통과 금지).
            if (dr !== 0 && dc !== 0
                && (isOccupied(map, cell[0] + dr, cell[1]) || isOccupied(map, cell[0], cell[1] + dc))) continue
            const ni = nr * W + nc
            if (closed[ni]) continue
            const tentative = g[cur.i] + cost
            if (tentative < g[ni]) {
                g[ni] = tentative
                parent[ni] = cur.i
                emit({event: "candidate_evaluated", state: [nr, nc], cost: tentative})
                emit({event: "edge_added", state: [nr, nc], parent: cell, cost: tentative})
                open.push({i: ni, f: tentative + h([nr, nc]), g: tentative})
            }
        }
    }

    emit({event: "planning_finished", success: false, metrics: {expanded_nodes: expandedCount}})
    return events
}
