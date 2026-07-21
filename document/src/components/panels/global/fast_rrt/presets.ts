import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// Fast-RRT 전용: 가운데 세로 벽을 한 칸 gap만 남기고 막은 narrow-passage 맵.
// Fast-Sampling(미탐사 공간 집중) + Random Steering(막힌 확장을 무작위 방향으로
// 우회)이 좁은 통로 통과에 유리하다는 Wu et al. (2021)의 주장을 엔진 실측으로
// 보여 준다. 대각 start/goal이라 트리가 gap을 찾아 꺾어 들어가야 한다.
//
// seed 1~12, budget 800 실측 (같은 파라미터, 엔진 직접 실행):
//   Fast-RRT  성공 11/12  평균 비용 19.88
//   RRT*      성공  9/12  평균 비용 20.66
//   RRT       성공  9/12  평균 비용 25.24
// seed 1 단독(sandbox 기본): Fast-RRT 19.85 vs RRT 25.21 (RRT는 첫 경로에서 얼어붙는다).
// budget을 400 으로 낮추면 RRT/RRT* 성공률이 7/12 로 더 떨어져 threading 격차가 커진다.
export const GAP_N = 20;
export const GAP_START: Point = [2.5, 3.5];
export const GAP_GOAL: Point = [17.5, 16.5];
// 벽은 grid column 10 전체를 막되 grid row 9(world y∈[10,11)) 한 칸만 통로로 남긴다.
const WALL_COL = 10;
const GAP_ROW = 9;

export function narrowPassageMap(): GridMap {
    const map = emptyGrid("narrow_passage", GAP_N, GAP_N)
    for (let r = 0; r < GAP_N; r++) {
        if (r === GAP_ROW) continue
        map.occupied[r * GAP_N + WALL_COL] = true
    }
    return map
}
