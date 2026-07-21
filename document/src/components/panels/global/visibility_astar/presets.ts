import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";

// Visibility A* 전용: 엇갈린 두 슬래브. 최단 any-angle 경로가 두 슬래브의 안쪽
// 모서리를 차례로 감아야 해서, 부모 사슬 하나에 갇히는 Theta*는 한쪽 taut 턴을
// 놓친다 (Daniel et al. 2010 류 반례 형태). visibility graph 최적은 그 차이를
// 실측으로 보여 준다: 이 배치에서 18.44 vs 18.69, 확장 9 vs 38.
export const SLAB_N = 16;
export const SLAB_START: Cell = [14, 1];
export const SLAB_GOAL: Cell = [1, 14];

export function slabsMap(): GridMap {
    const map = emptyGrid("staggered_slabs", SLAB_N, SLAB_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * SLAB_N + c] = true
        }
    }
    fill(3, 5, 1, 7)      // 위 슬래브 (왼쪽 벽에 붙음)
    fill(8, 10, 8, 14)    // 아래 슬래브 (오른쪽 벽에 붙음)
    return map
}

// Fan figure 전용: L자 벽 + 기둥 하나. 한 점에서 보이는 영역이 벽 뒤 그림자와
// 함께 또렷이 드러나는 배치다.
export const FAN_N = 14;
export const FAN_SOURCE: Cell = [10, 3];

export function fanMap(): GridMap {
    const map = emptyGrid("visibility_fan", FAN_N, FAN_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * FAN_N + c] = true
        }
    }
    fill(3, 8, 6, 7)      // 세로 벽
    fill(3, 4, 8, 11)     // 위쪽 가로 날개 (L자)
    fill(10, 11, 10, 11)  // 떨어진 기둥
    return map
}
