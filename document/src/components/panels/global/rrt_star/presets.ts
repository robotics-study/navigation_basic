import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// RRT* 전용: 가운데 블록 하나뿐인 거의 빈 맵. 최적 경로가 블록을 스치는 매끈한
// 대각선이라 수렴이 눈에 보인다 (seed 1 실측: RRT는 첫 경로 29.16에서 멈추고,
// RRT*은 800회 23.58 → 2000회 22.15 → 5000회 22.03으로 하한 ~21.7에 다가간다).
export const BLOCK_N = 20;
export const BLOCK_START: Point = [2.5, 2.5];
export const BLOCK_GOAL: Point = [17.5, 17.5];

export function centerBlockMap(): GridMap {
    const map = emptyGrid("center_block", BLOCK_N, BLOCK_N)
    for (let r = 8; r <= 11; r++) {
        for (let c = 8; c <= 11; c++) map.occupied[r * BLOCK_N + c] = true
    }
    return map
}
