import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// BIT* 전용: 2×2 기둥 아홉 개가 3×3으로 깔린 기둥밭. 통로가 여러 갈래라 informed
// 타원 안에서도 배치마다 다른 틈새가 열리고, 간선 큐가 goal 쪽 간선부터 꺼내며
// 기둥 사이를 lazy하게 꿰는 모습이 잘 보인다. batch_size 40, gamma 30, seed 1
// 엔진 실측 (직선 하한 29.70):
//   1배치 34.65 → 2배치 32.27 → 4배치 30.65 → 6배치 30.24
//   같은 예산(240 표본)의 Informed RRT*는 아직 경로 없음.
// 간선 큐가 목표에 닿을 수 있는 간선부터 확장하므로, BIT*는 훨씬 적은 표본으로
// 먼저 경로를 얻고 배치마다 하한으로 조여든다 (Gammell et al. 2015).
export const BIT_N = 26;
export const BIT_START: Point = [2.5, 2.5];
export const BIT_GOAL: Point = [23.5, 23.5];
export const BIT_BATCH_SIZE = 40;
export const BIT_GAMMA = 30;

export function pillarFieldMap(): GridMap {
    const map = emptyGrid("bit_pillars", BIT_N, BIT_N)
    for (const r of [5, 12, 19]) {
        for (const c of [5, 12, 19]) {
            for (let rr = r; rr <= r + 1; rr++) {
                for (let cc = c; cc <= c + 1; cc++) map.occupied[rr * BIT_N + cc] = true
            }
        }
    }
    return map
}
