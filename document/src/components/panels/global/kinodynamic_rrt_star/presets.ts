import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// Kinodynamic RRT* 전용: 왼쪽 위 모서리에 붙어 가운데까지 내려오는 대각 slash 하나.
// 시작(왼쪽 아래)과 목표(오른쪽 위)는 이 벽 반대편에 놓이고, 벽이 위 모서리에 막혀
// 있어 유일한 통로는 벽의 자유 끝(가운데)을 아래로 감아 도는 것뿐이다. double
// integrator는 관성을 실어 그 끝을 넓은 호로 쓸고 지나가므로, 직선 간선을 긋는
// 기하 planner와 달리 곡선 궤적이 선명히 드러난다.
// (sandbox 실측: seed 1, 2000 반복, goal_tol 1.0, neighbor_radius 2.0, v_max 1.5.
// control_weight r을 키우면 제어 노력에 페널티가 커져 비용이 단조로 오른다 —
// r 0.3 → 제어비용 30.3·트리 1642, r 1.0 → 38.2·1612, r 3.0 → 56.3·1577. 같은 맵의
// 기하 RRT*는 직선 길이 29.0로 끝나지만 관성을 무시한 값이라 단위가 다르다.)
export const KINO_N = 24;
export const KINO_START: Point = [3.0, 3.0];
export const KINO_GOAL: Point = [21.0, 21.0];

export function slashMap(): GridMap {
    const map = emptyGrid("kino_slash", KINO_N, KINO_N)
    const set = (r: number, c: number) => {
        if (r >= 0 && r < KINO_N && c >= 0 && c < KINO_N) map.occupied[r * KINO_N + c] = true
    }
    // 위 모서리(0,0)에 붙어 (16,16) 끝까지 내려오는 두께 3 대각 벽. 오른쪽 아래는 열려
    // 있어 경로가 그 자유 끝을 감아 돌아야 한다.
    for (let d = 0; d <= 15; d++) {
        set(d, d)
        set(d, d + 1)
        set(d + 1, d)
    }
    return map
}
