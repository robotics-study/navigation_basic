import {TraceEvent} from "./types";

export type Cell = [number, number];  // (row, col)

// trace 이벤트 열을 재생 가능한 타임라인으로 접는다. step은 이벤트 index와 같고,
// 렌더러는 "step 이하"의 이벤트만 그린다 — 스크러버로 임의 시점 탐색이 가능하다.
export interface GridTimeline {
    steps: number;                                          // 마지막 step (= 이벤트 수)
    // 연속 상태 planner(Hybrid A* 등): state가 (x, y[, θ]) world 좌표다. 이때 아래의
    // cell 필드들은 [x, y]를 담고, 렌더러는 world→canvas 변환으로 그린다.
    continuous: boolean;
    expanded: Array<{step: number; cell: Cell; cost?: number}>;
    edges: Array<{step: number; from: Cell; to: Cell}>;
    candidates: Array<{step: number; cell: Cell}>;
    // 실행형 planner(D* Lite 등)의 주행·감지 이벤트. 비어 있으면 일반 one-shot 탐색.
    robot: Array<{step: number; cell: Cell}>;
    revealed: Array<{step: number; cell: Cell}>;
    // anytime planner(ARA* 등)는 path_found 를 여러 번 방출한다 — 개선 순서대로 쌓인다.
    // cost 가 이벤트에 없으면 8-connected unit/√2 스텝 합으로 계산해 채운다.
    paths: Array<{step: number; path: Cell[]; cost: number}>;
    path: Cell[];                                           // 마지막(최종) 경로
    // 최종 경로의 원본 상태열 (연속 planner 는 [x, y, θ]) — 차량 주행 애니메이션용.
    pathStates: number[][];
    pathStep: number;                                       // 첫 path_found 시점 (없으면 Infinity)
    params?: Record<string, unknown>;
    metrics?: Record<string, number>;
    success?: boolean;
}

const asCell = (state?: number[]): Cell | null =>
    state && state.length >= 2 ? [state[0], state[1]] : null

// 8-connected 격자 경로의 기하 비용 (unit/√2 스텝 합). cost 없는 path_found 의 대체값.
const geometricCost = (path: Cell[]): number => {
    // 폴리라인 유클리드 길이 — grid 경로에서는 unit/√2 스텝 합과 일치하고,
    // 연속(SE(2)) 경로에서도 그대로 성립한다.
    let total = 0
    for (let i = 1; i < path.length; i++) {
        total += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    }
    return total
}

export interface PathSampler {
    length: number;                                   // 경로 전체 호 길이
    at: (s: number) => [number, number, number];      // 호 길이 s 에서의 pose
}

// 상태열 [x, y, θ]를 누적 호 길이로 매개화해 pose 를 보간한다 — 찾은 경로를
// 차량이 실제로 주행하는 애니메이션에 쓴다. θ는 최단 각도차로 보간해
// 전진/후진 전환(θ와 진행 방향이 반대)에서도 저장된 heading 을 따른다.
export function pathSampler(states: number[][]): PathSampler {
    const cum: number[] = [0]
    for (let i = 1; i < states.length; i++) {
        cum.push(cum[i - 1] + Math.hypot(
            states[i][0] - states[i - 1][0], states[i][1] - states[i - 1][1]))
    }
    const length = cum[cum.length - 1]
    const at = (s: number): [number, number, number] => {
        const target = Math.max(0, Math.min(length, s))
        let i = 1
        while (i < cum.length - 1 && cum[i] < target) i++
        const seg = cum[i] - cum[i - 1]
        const u = seg > 0 ? (target - cum[i - 1]) / seg : 0
        const a = states[i - 1]
        const b = states[i]
        const ta = a[2] ?? 0
        let d = (b[2] ?? 0) - ta
        d -= 2 * Math.PI * Math.round(d / (2 * Math.PI))
        return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, ta + d * u]
    }
    return {length, at}
}

export function buildGridTimeline(events: TraceEvent[]): GridTimeline {
    const timeline: GridTimeline = {
        steps: events.length,
        continuous: false,
        expanded: [],
        edges: [],
        candidates: [],
        robot: [],
        revealed: [],
        paths: [],
        path: [],
        pathStates: [],
        pathStep: Infinity,
    }
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "planning_started":
                timeline.params = ev.params
                break
            default:
                break
        }
        // 정수가 아닌 상태 좌표가 하나라도 있으면 연속 상태 planner다.
        if (!timeline.continuous && ev.state
            && (!Number.isInteger(ev.state[0]) || !Number.isInteger(ev.state[1]))) {
            timeline.continuous = true
        }
        switch (ev.event) {
            case "node_expanded": {
                const cell = asCell(ev.state)
                if (cell) timeline.expanded.push({step, cell, cost: ev.cost})
                break
            }
            case "candidate_evaluated": {
                const cell = asCell(ev.state)
                if (cell) timeline.candidates.push({step, cell})
                break
            }
            case "edge_added":
            case "rewire": {
                const to = asCell(ev.state)
                const from = asCell(ev.parent)
                if (to && from) timeline.edges.push({step, from, to})
                break
            }
            case "robot_moved": {
                const cell = asCell(ev.state)
                if (cell) timeline.robot.push({step, cell})
                break
            }
            case "obstacle_revealed": {
                const cell = asCell(ev.state)
                if (cell) timeline.revealed.push({step, cell})
                break
            }
            case "path_found":
                if (ev.path) {
                    const path = ev.path
                        .map((s) => asCell(s))
                        .filter((c): c is Cell => c !== null)
                    timeline.paths.push({step, path, cost: ev.cost ?? geometricCost(path)})
                    timeline.path = path
                    timeline.pathStates = ev.path
                    timeline.pathStep = Math.min(timeline.pathStep, step)
                }
                break
            case "planning_finished":
                timeline.metrics = ev.metrics
                timeline.success = ev.success
                break
            default:
                break
        }
    })
    return timeline
}
