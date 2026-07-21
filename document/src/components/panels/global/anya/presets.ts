import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";

// Anya 전용: 대각선 경로를 가로막는 블록 하나. 참 최단 경로는 블록의 두 모서리를
// 정확히 스치는데, 그 모서리는 격자 꼭짓점이라 셀 중심만 쓰는 planner는 표현할
// 수 없다. 이 배치에서 Anya 17.10 vs Visibility A* 17.73, 확장 11 vs 52.
export const BLOCK_N = 14;
export const BLOCK_START: Cell = [12, 1];
export const BLOCK_GOAL: Cell = [1, 12];

export function blockMap(): GridMap {
    const map = emptyGrid("corner_block", BLOCK_N, BLOCK_N)
    for (let r = 4; r <= 9; r++) {
        for (let c = 5; c <= 8; c++) map.occupied[r * BLOCK_N + c] = true
    }
    return map
}
