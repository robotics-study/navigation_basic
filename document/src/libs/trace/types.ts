// spec/trace_schema.json의 TypeScript 대응. C++/Python 데모가 방출한 trace와
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
    | "force_computed"
    | "histogram_updated"
    | "band_updated"
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
    // planning_started 선택: local demo 가 참조 경로/goal 재현에 쓰는 시나리오 yaml 경로.
    scenario?: string;
    success?: boolean;
    metrics?: Record<string, number>;
    // 알고리즘별 부가 정보 (예: visibility A*의 interval run, robot_moved의 {v, omega},
    // force_computed의 {fx_att, fy_att, fx_rep, fy_rep, fx, fy}) — 렌더러는 몰라도 된다.
    data?: Record<string, unknown>;
    // histogram_updated 전용: 폴라 히스토그램 sector 값 (index 0 = world +x, 반시계).
    bins?: number[];
    // candidate_evaluated 선택: 후보 명령을 짧은 horizon 동안 적분한 예측 궤적 폴리라인
    // (DWA 등 롤아웃 채점 계열). data는 스칼라 맵이라 배열을 담지 못해 bins처럼 top-level.
    rollout?: number[][];
    // band_updated 전용: 탄성 밴드 상태 열. 항목 길이 3 = [x, y, radius] (Elastic Bands
    // bubble), 길이 4 = [x, y, theta, dt] (TEB pose + 직전 구간 dt). data는 스칼라 맵이라
    // 배열을 담지 못해 bins/rollout처럼 top-level.
    band?: number[][];
}
