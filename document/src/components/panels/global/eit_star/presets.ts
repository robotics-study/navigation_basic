import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// EIT* 전용: 두꺼운 가로 선반(rows 13..16, cols 0..23)이 위아래를 가르고 오른쪽
// 끝(cols 24..29)만 열린 선반 지형. start(왼쪽 위)와 goal(왼쪽 아래)이 같은 쪽에
// 있어 둘을 잇는 직선은 선반을 수직 관통하므로, 실제 경로는 오른쪽 끝을 크게
// 돌아온다. 이 우회가 검증 노력(effort, 충돌 검사 sub-segment 수)을 부풀려,
// EIT*가 AIT*에 더한 두 번째 역방향 heuristic(effort-to-go)이 드러난다.
//
// batchSize 120, seed 1, step_size 0.5 엔진 실측 (직선거리 21.00,
// 직선거리 effort = round(21/0.5) = 42):
//   maxBatches 2  cost-to-go 52.16 · effort-to-go 103
//   maxBatches 4  cost-to-go 50.91 · effort-to-go  97
//   maxBatches 6  cost-to-go 49.72 · effort-to-go  95
// cost-to-go(≈50 m)와 effort-to-go(≈100 segments) 모두 직선 baseline(21 m,
// 42 segments)을 크게 웃돌아 오른쪽으로 도는 우회를 반영한다. effort-to-go는
// 거리가 아니라 충돌 검사 segment 수를 재며, 최소 segment 경로를 독립적으로
// 찾는다. cost-to-go는 같은 파라미터의 AIT* 역방향 heuristic과 정확히 일치한다 —
// EIT*는 AIT*의 cost heuristic을 그대로 두고 effort heuristic을 더한 것임을
// 실측으로 보인다.
export const GATE_N = 30;
export const GATE_START: Point = [2.5, 25.0];
export const GATE_GOAL: Point = [2.5, 4.0];

export function shelfMap(): GridMap {
    const map = emptyGrid("eit_shelf", GATE_N, GATE_N)
    // 선반은 rows 13..16, cols 0..23. 오른쪽 cols 24..29만 통로로 열린다.
    for (let r = 13; r <= 16; r++) {
        for (let c = 0; c <= 23; c++) map.occupied[r * GATE_N + c] = true
    }
    return map
}
