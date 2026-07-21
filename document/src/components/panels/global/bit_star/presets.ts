import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// BIT* 전용: 가운데 정사각 블록 하나가 start(좌하)–goal(우상) 직선을 막아 모서리를
// 돌게 하는 지형. BIT*의 특성은 배치 단위 informed 표본 + 간선 큐의 lazy 확장으로
// 얻는 anytime 개선이다. 배치를 늘리면 informed 타원이 통로에 집중되어 첫 배치의
// 꺾인 경로가 최적(모서리 밀착, cost ≈ 32.0)으로 점점 조여진다. batch_size=40,
// gamma=30, seed 1 실측(engine, 26×26 center block):
//   BIT*  1배치  cost 33.738 · 간선 12 · 개선 1회
//         2배치  cost 32.512 · 간선 34 · 개선 2회
//         4배치  cost 32.210 · 간선 94 · 개선 4회
//         6배치  cost 32.074 · 간선 167 · 개선 5회
//   같은 표본 예산의 Informed RRT*: 40·80·160표본은 아직 경로 없음, 240표본 cost 35.554.
// 간선 큐가 목표에 닿을 수 있는 간선부터 꺼내므로, BIT*는 훨씬 적은 표본으로 먼저
// 경로를 얻고 더 낮은 비용에 닿는다 (Gammell et al. 2015).
export const BIT_N = 26;
export const BIT_START: Point = [2.5, 2.5];
export const BIT_GOAL: Point = [23.5, 23.5];
export const BIT_BATCH_SIZE = 40;
export const BIT_GAMMA = 30;

export function blockMap(): GridMap {
    const map = emptyGrid("bit_block", BIT_N, BIT_N)
    for (let r = 9; r <= 16; r++) {
        for (let c = 9; c <= 16; c++) map.occupied[r * BIT_N + c] = true
    }
    return map
}
