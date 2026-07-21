import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// Informed RRT* 전용: start/goal 대각선을 부분적으로 막는 블록 둘을 둔 30x30 개방
// 맵. 넓은 공간이라 RRT*의 균일 표본은 첫 해 이후에도 맵 전체에 흩어져 낭비되는
// 반면, Informed RRT*은 첫 해 즉시 표본을 start/goal 초점의 타원 안으로 좁혀 같은
// 예산에서 더 싼 경로로 수렴한다 (Gammell, Srinivasa & Barfoot 2014).
//
// seed 3 실측 (같은 seed·예산에서 RRT* vs Informed 최종 비용, 직선 하한 35.36):
//   500회  RRT* 38.36 / Informed 37.52
//   1000회 RRT* 38.23 / Informed 36.54
//   2000회 RRT* 37.00 / Informed 35.49
// 타원 집중도 (seed 3, 2000회): 첫 해는 약 330회째 나오고, 그 뒤 표본의 95.6%가
// 대각선에서 수직거리 5.5 이내에 떨어진다 (해 이전 균일 표본은 45.6%). 표본의
// 평균 수직거리가 6.71에서 2.28로 준다.
export const OPEN_N = 30;
export const OPEN_START: Point = [2.5, 2.5];
export const OPEN_GOAL: Point = [OPEN_N - 2.5, OPEN_N - 2.5];

// [row0, row1, col0, col1] 폐구간 블록 — 대각선을 가로질러 최적 경로를 하한보다
// 살짝 길게 만들어 (타원이 완전히 납작해지지 않게) 수렴 여지를 남긴다.
const BLOCKS: ReadonlyArray<readonly [number, number, number, number]> = [
    [10, 14, 6, 10],
    [16, 20, 19, 23],
];

export function openEllipseMap(): GridMap {
    const map = emptyGrid("informed_open", OPEN_N, OPEN_N)
    for (const [r0, r1, c0, c1] of BLOCKS) {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * OPEN_N + c] = true
        }
    }
    return map
}
