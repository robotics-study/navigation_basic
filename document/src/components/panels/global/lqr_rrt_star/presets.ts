import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// LQR-RRT* 전용: start→goal 대각선을 가로질러 위/아래로 번갈아 놓인 자립 블록 셋(chevron
// gauntlet). 차량이 세 블록을 지그재그로 감아 돌며, LQR 피드백이 그리는 rest→rest 곡선
// 간선과 방향 인지 metric이 함께 드러난다. 벽에 붙은 선반이나 기둥 격자가 아니라, 대각
// 진행을 막는 어긋난 블록이라 매 확장이 각도를 틀며 접근한다.
// (sandbox 파라미터(seed 1, 1200 반복, goal_tol 1.0) 엔진 실측: r_ctrl 0.2 → LQR cost
// 33.2 / 1037 노드, 1.0 → 40.6 / 1036, 5.0 → 56.3 / 1030. 제어 페널티가 클수록 같은
// 트리 크기에서 궤적 비용이 오른다 — control을 아낄수록 굼뜬 조절이 비싸진다.)
export const LQR_N = 24;
export const LQR_START: Point = [3.0, 3.0];
export const LQR_GOAL: Point = [21.0, 21.0];

// 블록 셋 (grid row 위=0 기준, [r0, r1, c0, c1] 포함 범위).
const BLOCKS: Array<[number, number, number, number]> = [
    [14, 17, 3, 8],    // 아래-왼쪽
    [9, 12, 11, 16],   // 가운데-오른쪽
    [4, 7, 6, 11],     // 위-가운데
];

export function chevronMap(): GridMap {
    const map = emptyGrid("lqr_chevron", LQR_N, LQR_N);
    for (const [r0, r1, c0, c1] of BLOCKS) {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * LQR_N + c] = true;
        }
    }
    return map;
}
