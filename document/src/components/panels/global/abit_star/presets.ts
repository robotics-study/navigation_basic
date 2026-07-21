import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// ABIT* 전용: 대각선을 두껍게 가로막는 중앙 블록과 우하 보조 블록을 둔 26x26 맵. 넓은
// free 공간이 배치마다 후보 간선을 많이 만들어, truncation이 현직 해를 거의 못 줄이는
// 간선 처리를 건너뛸 여지를 준다. 같은 표본 위에서 ABIT*(inflation 10→1, truncation
// 2→1)와 un-inflated 기준선(inflation 1, truncation 1, 곧 BIT*)을 나란히 돌린 seed 2
// 실측(engine), 직선 하한 29.70:
//   4배치  ABIT* 70간선 31.02  vs  BIT* 156간선 31.11   (-55% 간선)
//   6배치  ABIT* 93간선 31.03  vs  BIT* 257간선 30.94   (-64% 간선)
//   8배치  ABIT* 104간선 31.02 vs  BIT* 356간선 30.94   (-71% 간선)
// 비용은 자릿수까지 사실상 같은데 ABIT*가 세우는 간선(= 통과한 충돌 검사)은 절반
// 아래다. truncation이 마지막 비싼 검사를 건너뛰고, 배치가 늘수록 격차가 벌어진다
// (Strub & Gammell 2020).
export const ABIT_N = 26;
export const ABIT_START: Point = [2.5, 2.5];
export const ABIT_GOAL: Point = [ABIT_N - 2.5, ABIT_N - 2.5];

// 스케줄·연결 반경 (sandbox와 페이지가 공유). ABIT*는 이 계수들을 배치마다
// inflation_final / 1.0 으로 감소시키고, 기준선은 1 로 고정해 BIT*로 환원한다.
export const ABIT_BATCH_SIZE = 70;
export const ABIT_GAMMA = 30;
export const ABIT_INFLATION = 10;
export const ABIT_INFLATION_FINAL = 1;
export const ABIT_TRUNCATION = 2;
export const ABIT_BATCH_COUNTS = [4, 6, 8];

// [row0, row1, col0, col1] 폐구간 블록 — 대각선을 막아 최적 경로를 직선 하한보다
// 길게 우회시키되, 양옆 통로를 남겨 여러 배치에 걸쳐 수렴 여지를 준다.
const BLOCKS: ReadonlyArray<readonly [number, number, number, number]> = [
    [6, 15, 9, 15],
    [17, 21, 17, 22],
];

export function chamberMap(): GridMap {
    const map = emptyGrid("abit_chamber", ABIT_N, ABIT_N)
    for (const [r0, r1, c0, c1] of BLOCKS) {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * ABIT_N + c] = true
        }
    }
    return map
}
