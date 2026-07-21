import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// PRM* 전용: 흩어진 블록. 통로가 여럿이라 연결 자체는 쉽고, 관심사는 경로
// 품질이다. 표본 수를 올리면 r_n이 줄면서도 비용이 직선 하한(21.21)으로
// 수렴한다 (seed 2 실측: n=100 → 21.83, n=300 → 21.76, n=800 → 21.64.
// 고정 반경 PRM은 n=100에서 아예 끊긴다).
export const SCATTER_N = 20;
export const SCATTER_START: Point = [2.5, 2.5];
export const SCATTER_GOAL: Point = [17.5, 17.5];

export function scatterMap(): GridMap {
    const map = emptyGrid("scatter_blocks", SCATTER_N, SCATTER_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * SCATTER_N + c] = true
        }
    }
    fill(3, 6, 4, 6)
    fill(12, 15, 8, 10)
    fill(5, 8, 13, 15)
    return map
}
