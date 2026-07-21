import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";

// BFS 전용: 문 하나로 이어진 두 방. hop 동심원이 왼쪽 방을 가득 채우고, 문틈으로
// 스며든 뒤 오른쪽 방에서 다시 동심원으로 퍼지는 파동이 가장 잘 보이는 지형.
export const ROOMS_N = 20;
export const ROOMS_START: Cell = [10, 3];
export const ROOMS_GOAL: Cell = [10, 16];

export function roomsMap(): GridMap {
    const map = emptyGrid("rooms", ROOMS_N, ROOMS_N)
    for (let r = 0; r < ROOMS_N; r++) {
        if (r === 9 || r === 10) continue   // 문
        map.occupied[r * ROOMS_N + 9] = true
    }
    return map
}
