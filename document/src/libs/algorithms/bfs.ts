import {GridMap, isOccupied} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 BFS. 저장소 구현과 동일하게 FIFO frontier + 최초 발견 시 부모
// 고정으로 hop-최단 경로를 찾고, 보고 비용은 실제 edge cost 누적이다 (hop 최단 경로가
// 기하학적으로는 비쌀 수 있음을 그대로 드러낸다). trace 이벤트 계약도 동일하다.
export interface BFSOptions {
    map: GridMap;
    start: Cell;
    goal: Cell;
    connectivity: 4 | 8;
}

const SQRT2 = Math.SQRT2;

export function runBFS({map, start, goal, connectivity}: BFSOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})

    emit({event: "planning_started", algorithm: "bfs", params: {connectivity}})

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]
    const parent = new Array<number>(map.width * map.height).fill(-1)
    const costTo = new Array<number>(map.width * map.height).fill(Infinity)
    const discovered = new Array<boolean>(map.width * map.height).fill(false)

    const deltas: Array<[number, number, number]> = connectivity === 4
        ? [[-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1]]
        : [[-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
           [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2]]

    const startI = idx(start)
    discovered[startI] = true
    costTo[startI] = 0
    const frontier: number[] = [startI]
    let head = 0
    let expanded = 0
    let found = false

    while (head < frontier.length) {
        const cur = frontier[head++]
        const cell: Cell = [Math.floor(cur / W), cur % W]
        expanded++
        emit({event: "node_expanded", state: cell, cost: costTo[cur]})
        if (cur === idx(goal)) {
            found = true
            break
        }
        for (const [dr, dc, cost] of deltas) {
            const nr = cell[0] + dr
            const nc = cell[1] + dc
            if (isOccupied(map, nr, nc)) continue
            // 대각 이동은 양 옆이 모두 뚫려 있어야 한다 (모서리 통과 금지).
            if (dr !== 0 && dc !== 0
                && (isOccupied(map, cell[0] + dr, cell[1]) || isOccupied(map, cell[0], cell[1] + dc))) continue
            const ni = nr * W + nc
            if (discovered[ni]) continue
            discovered[ni] = true
            parent[ni] = cur
            costTo[ni] = costTo[cur] + cost
            emit({event: "edge_added", state: [nr, nc], parent: cell, cost})
            frontier.push(ni)
        }
    }

    if (!found) {
        emit({event: "planning_finished", success: false, metrics: {expanded_nodes: expanded}})
        return events
    }
    const path: Cell[] = []
    let p = idx(goal)
    while (p >= 0) {
        path.unshift([Math.floor(p / W), p % W])
        p = parent[p]
    }
    emit({event: "path_found", path, cost: costTo[idx(goal)]})
    emit({
        event: "planning_finished",
        success: true,
        metrics: {path_cost: costTo[idx(goal)], expanded_nodes: expanded},
    })
    return events
}
