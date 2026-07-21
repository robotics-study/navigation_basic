import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// SST 전용: 가운데 큰 섬을 두고 왼쪽 아래에서 오른쪽 중앙까지 섬을 반 바퀴 돌아야
// 하는 순환 지형. unicycle이 긴 곡선을 그리며 달리는 것이 잘 보이고, 대부분 열려
// 있어 여러 witness 반경 δ_s에서 안정적으로 goal에 닿는다.
// (sandbox 파라미터(seed 1, 4000 반복, goal_tol 0.8, footprint 0.4) 실측: δ_s
// 0.3 → active 733, 0.4 → 485, 0.5 → 324로 트리가 희소해진다. 같은 예산·같은 맵의
// RRT*는 3천 노드 남짓을 전부 들고 있다. footprint 0.4는 그려지는 차 폭의 절반과
// 같아, 경로가 차체만큼 벽에서 물러난다.)
export const SST_N = 24;
export const SST_START: Point = [3.0, 5.0];
export const SST_GOAL: Point = [21.0, 12.0];

export function islandMap(): GridMap {
    const map = emptyGrid("sst_island", SST_N, SST_N)
    for (let r = 6; r <= 17; r++) {
        for (let c = 6; c <= 17; c++) map.occupied[r * SST_N + c] = true
    }
    return map
}
