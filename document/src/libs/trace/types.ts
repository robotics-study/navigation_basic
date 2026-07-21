// spec/trace_schema.json 의 TypeScript 대응. C++/Python 데모가 방출한 trace 와
// 브라우저 라이브 엔진이 만드는 이벤트가 같은 타입을 공유한다 — 패널/플레이어 코드는 하나다.
export type TraceEventType =
    | "planning_started"
    | "node_expanded"
    | "edge_added"
    | "sample_drawn"
    | "rewire"
    | "candidate_evaluated"
    | "constraint_added"
    | "conflict_found"
    | "robot_moved"
    | "obstacle_revealed"
    | "path_found"
    | "planning_finished";

export interface TraceEvent {
    seq: number;
    event: TraceEventType;
    t?: number;
    agent?: number;
    state?: number[];
    parent?: number[];
    cost?: number;
    path?: number[][];
    algorithm?: string;
    map?: string;
    params?: Record<string, unknown>;
    success?: boolean;
    metrics?: Record<string, number>;
}
