import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// FCIT* 전용: 대각선을 엇갈리게 막는 블록 셋. 최적 경로는 블록 사이 긴 자유 구간을
// 직선으로 꿰어야 하는데, FCIT*의 완전 연결 그래프는 그 먼 표본 쌍을 직행 간선
// 하나로 이어 첫 배치부터 최적 근처에 붙는다. 줄어드는 반경 RGG인 AIT*는 같은
// 거리를 중간 표본 사슬로 이어야 해 배치가 쌓여야 따라온다. batchSize 20 엔진
// 실측 (AIT*는 gamma 30, 같은 표본 예산):
//   seed 1: 1배치 FCIT* 30.01 vs AIT* 35.94 · 2배치 30.01 vs 33.44 · 4배치 29.85 vs 30.10
//   seed 3: 1배치 FCIT* 30.06 vs AIT* 32.84 · 4배치 29.89 vs 29.96
// (Wilson, Thomason, Kingston, Kavraki & Gammell 2025)
export const FCIT_N = 26;
export const FCIT_START: Point = [2.5, 2.5];
export const FCIT_GOAL: Point = [23.5, 23.5];
export const FCIT_BATCH_SIZE = 20;

export function staggeredMap(): GridMap {
    const map = emptyGrid("fcit_staggered", FCIT_N, FCIT_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * FCIT_N + c] = true
        }
    }
    fill(6, 9, 8, 11)
    fill(12, 16, 12, 15)
    fill(19, 22, 17, 19)
    return map
}
