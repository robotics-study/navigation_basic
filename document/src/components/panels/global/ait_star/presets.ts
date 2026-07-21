import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// AIT* 전용: 두꺼운 세로벽(cols 12..15)이 좌우를 가르고 위쪽(rows 0..4)에만 문이
// 뚫린 gate 지형. start(좌하단)에서 goal(우상단)로 가는 직선은 벽을 정면으로
// 관통하므로, 직선거리 heuristic(BIT* 방식)은 벽을 못 보고 비용을 과소평가한다.
// AIT*의 역방향 그래프 heuristic은 문 위로 도는 실제 우회를 반영해 참값에 붙는다.
// batchSize 120, seed 1 엔진 실측(straight-line = 35.36, aitStarReadout):
//   maxBatches 2  reverse-h 39.15 · optimal 39.97
//   maxBatches 4  reverse-h 38.61 · optimal 39.56
//   maxBatches 6  reverse-h 39.55 · optimal 39.55
// reverse-h는 직선거리(35.36)를 항상 10% 넘게 웃돌아 벽을 우회하는 참 비용에 붙고,
// 배치가 쌓여 무효 간선이 더 발견될수록 optimal에 정확히 조여든다 (문을 지우면
// reverse-h와 직선거리가 같은 값으로 다시 만난다 — 장애물이 없으면 둘은 일치한다).
export const GATE_N = 28;
export const GATE_START: Point = [1.5, 1.5];
export const GATE_GOAL: Point = [26.5, 26.5];

export function gateMap(): GridMap {
    const map = emptyGrid("ait_gate", GATE_N, GATE_N)
    // 벽은 row 5..27, cols 12..15. row 0..4(맵 위쪽 = world y 큰 쪽)만 문으로 열린다.
    for (let r = 5; r < GATE_N; r++) {
        for (let c = 12; c <= 15; c++) map.occupied[r * GATE_N + c] = true
    }
    return map
}
