import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// Fast-RRT 전용: 가운데 세로 벽을 한 칸 gap만 남기고 막은 narrow-passage 맵.
// Fast-Sampling(미탐사 공간 집중) + Random Steering(막힌 확장을 무작위 방향으로
// 우회)이 좁은 통로 통과에 유리하다는 Wu et al. (2021)의 주장을 엔진 실측으로
// 보여 준다. gap을 start-goal 직선에서 멀리(위쪽) 두어, 경로가 눈에 띄게 꺾여
// 통로를 지나는 것이 화면에서 분명히 보인다.
//
// seed 1~12, budget 800 실측 (같은 파라미터, 엔진 직접 실행):
//   Fast-RRT  성공 11/12  평균 비용 28.68
//   RRT*      성공  8/12  평균 비용 30.81
//   RRT       성공  8/12  평균 비용 36.40
// seed 2 단독(sandbox 기본): Fast-RRT 28.45 vs RRT 35.90 (RRT는 첫 경로에서 얼어붙는다).
export const GAP_N = 20;
export const GAP_START: Point = [2.5, 3.5];
export const GAP_GOAL: Point = [17.5, 3.5];
// 벽은 grid column 10 전체를 막되 grid row 4(world y∈[15,16)) 한 칸만 통로로 남긴다.
const WALL_COL = 10;
const GAP_ROW = 4;

export function narrowPassageMap(): GridMap {
    const map = emptyGrid("narrow_passage", GAP_N, GAP_N)
    for (let r = 0; r < GAP_N; r++) {
        if (r === GAP_ROW) continue
        map.occupied[r * GAP_N + WALL_COL] = true
    }
    return map
}
