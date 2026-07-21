import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// ABIT* 전용: 대각선을 엇갈리게 자르는 계단형 쐐기 벽 둘이 만드는 협곡. 경로가
// 계단 사이 좁은 협곡을 비스듬히 통과해야 해서 후보 간선이 많고, truncation이
// 현직 해를 거의 못 줄이는 간선 처리를 건너뛸 여지가 크다. 같은 표본 위에서
// ABIT*(inflation 10→1, truncation 2→1)와 un-inflated 기준선(inflation 1,
// truncation 1, 곧 BIT*)을 나란히 돌린 seed 2 실측(engine):
//   4배치  ABIT* 143간선 37.67  vs  BIT* 180간선 37.87   (-21% 간선)
//   6배치  ABIT* 216간선 37.62  vs  BIT* 280간선 37.83   (-23% 간선)
//   8배치  ABIT* 274간선 37.67  vs  BIT* 390간선 37.82   (-30% 간선)
// 비용은 사실상 같은데(오히려 근소 우위) ABIT*가 세우는 간선(= 통과한 충돌 검사)이
// 두드러지게 적고, 배치가 늘수록 격차가 벌어진다 (Strub & Gammell 2020).
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

// 대각 계단 벽 둘 — 협곡이 대각선을 엇갈리게 잘라 우회를 강제하되, 계단 사이
// 통로를 남겨 여러 배치에 걸쳐 수렴 여지를 준다.
const BLOCKS: ReadonlyArray<readonly [number, number, number, number]> = (() => {
    const out: Array<[number, number, number, number]> = []
    for (let i = 0; i < 8; i++) out.push([6 + i, 6 + i, 4 + i, 5 + i])
    for (let i = 0; i < 8; i++) out.push([11 + i, 11 + i, 13 + i, 14 + i])
    return out
})();

export function wedgeMap(): GridMap {
    const map = emptyGrid("abit_wedge", ABIT_N, ABIT_N)
    for (const [r0, r1, c0, c1] of BLOCKS) {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * ABIT_N + c] = true
        }
    }
    return map
}
