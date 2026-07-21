import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";

// maps/grid/wastar_greedy01과 같은 배치. goal 옆의 벽 두 칸이 부풀린 heuristic을
// 유인해, 첫 반복(큰 ε)은 돌아가는 경로를 내고 ε이 조여지면 경로가 짧아진다.
export const GREEDY_W = 13;
export const GREEDY_H = 14;
export const GREEDY_START: Cell = [11, 12];
export const GREEDY_GOAL: Cell = [0, 11];

export function greedyTrapMap(): GridMap {
    const map = emptyGrid("wastar_greedy", GREEDY_W, GREEDY_H)
    map.occupied[1 * GREEDY_W + 10] = true
    map.occupied[2 * GREEDY_W + 11] = true
    return map
}
