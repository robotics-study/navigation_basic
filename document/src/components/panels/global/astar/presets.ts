import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";

export const SANDBOX_N = 20;
export const SANDBOX_START: Cell = [SANDBOX_N - 2, 1];
export const SANDBOX_GOAL: Cell = [1, SANDBOX_N - 2];

// sandbox/히어로 기본 벽: 시작→목표 대각선을 막아서는 ⌐ 모양 pocket.
// 입구가 시작점 쪽으로 열려 있어 과대평가 heuristic(greedy)은 안으로 뛰어들었다가
// 더 긴 경로를 내놓는다 — heuristic 별 차이가 한눈에 보이는 배치다.
export function pocketMap(): GridMap {
    const map = emptyGrid("sandbox", SANDBOX_N, SANDBOX_N)
    const set = (row: number, col: number) => {
        if (row >= 0 && row < SANDBOX_N && col >= 0 && col < SANDBOX_N)
            map.occupied[row * SANDBOX_N + col] = true
    }
    for (let c = 5; c <= 14; c++) set(6, c)
    for (let r = 6; r <= 14; r++) set(r, 14)
    for (let r = 10; r <= 18; r++) set(r, 7)
    return map
}
