import {TraceEvent} from "./types";

export type Cell = [number, number];  // (row, col)

// trace 이벤트 열을 재생 가능한 타임라인으로 접는다. step은 이벤트 index와 같고,
// 렌더러는 "step 이하"의 이벤트만 그린다 — 스크러버로 임의 시점 탐색이 가능하다.
export interface GridTimeline {
    steps: number;                                          // 마지막 step (= 이벤트 수)
    expanded: Array<{step: number; cell: Cell; cost?: number}>;
    edges: Array<{step: number; from: Cell; to: Cell}>;
    candidates: Array<{step: number; cell: Cell}>;
    // 실행형 planner(D* Lite 등)의 주행·감지 이벤트. 비어 있으면 일반 one-shot 탐색.
    robot: Array<{step: number; cell: Cell}>;
    revealed: Array<{step: number; cell: Cell}>;
    path: Cell[];
    pathStep: number;                                       // path_found 시점 (없으면 Infinity)
    params?: Record<string, unknown>;
    metrics?: Record<string, number>;
    success?: boolean;
}

const asCell = (state?: number[]): Cell | null =>
    state && state.length >= 2 ? [state[0], state[1]] : null

export function buildGridTimeline(events: TraceEvent[]): GridTimeline {
    const timeline: GridTimeline = {
        steps: events.length,
        expanded: [],
        edges: [],
        candidates: [],
        robot: [],
        revealed: [],
        path: [],
        pathStep: Infinity,
    }
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "planning_started":
                timeline.params = ev.params
                break
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
                    timeline.path = ev.path
                        .map((s) => asCell(s))
                        .filter((c): c is Cell => c !== null)
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
