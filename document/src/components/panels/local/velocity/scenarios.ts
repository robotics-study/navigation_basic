import {AgentSpec} from "../../../../libs/algorithms/agent_sim";
import {GridMap} from "../../../../libs/grid";

// VO/RVO/ORCA sandbox 3개가 공유하는 시나리오 빌더 -- "head_on/circle_swap을
// 페이지 간 재사용해 독자가 VO의 진동 -> RVO/ORCA의 해소를 보게 한다"는 요구를
// 세 파일에 각각 복제하는 대신 여기 한 곳에 둔다. 좌표는 저장소
// maps/scenarios/velocity/*.yaml 및 maps/grid/open_arena.yaml을 그대로 따른다
// (15m x 15m 오픈 아레나, resolution 0.5m).

const RESOLUTION = 0.5;
const GRID_SIZE = 30; // 30 x 30 cell = 15m x 15m, open_arena.yaml과 동일.

export function openArenaMap(): GridMap {
    const width = GRID_SIZE
    const height = GRID_SIZE
    const occupied = new Array(width * height).fill(false)
    for (let r = 0; r < height; r++) {
        for (let c = 0; c < width; c++) {
            if (r === 0 || r === height - 1 || c === 0 || c === width - 1) occupied[r * width + c] = true
        }
    }
    return {name: "open_arena", width, height, occupied, resolution: RESOLUTION, originX: 0, originY: 0}
}

const spec = (
    x: number, y: number, theta: number, goal: [number, number], radius: number,
    scriptedVelocity?: [number, number],
): AgentSpec => ({start: {pose: [x, y, theta], v: 0, omega: 0}, goal, radius, scriptedVelocity})

// crossing: planner 1대 + 비협조적 등속 mover 1대 (maps/scenarios/velocity/crossing.yaml
// 미러) -- VO의 정석적인 단일-로봇-vs-이동장애물 케이스. mover는 절대 반응하지 않으므로
// planner 혼자 양보해야 한다.
export function crossingAgents(radius = 0.3): AgentSpec[] {
    return [
        spec(7.5, 2.5, Math.PI / 2, [7.5, 12.5], radius),
        spec(2.5, 7.5, 0, [12.5, 7.5], radius, [0.6, 0]),
    ]
}

// head_on: 거의 대칭인 두 몸체 정면 마주침. 저장소 head_on.yaml은 좌우 대칭을 0.9m
// 오프셋으로 깨서 pass/avoid 테스트로 쓰지만(그 파일 주석 참고), 그 오프셋은 VO조차
// 눈에 띄는 회피 동작 없이 그냥 지나치게 할 만큼 크다 -- 이 sandbox는 "상호 회피
// 진동"을 보여줘야 하므로 훨씬 작은 0.2m 오프셋만 둔다. 정확히 0(완전 대칭)이면
// ORCA의 선형계획이 정확한 수치적 동률에 빠져 STALLED로 굳는 실제 퇴화 케이스를
// 실측으로 확인했다(오프셋이 조금이라도 있으면 사라진다) -- VO/RVO는 이 작은
// 오프셋에서도 여전히 아슬아슬한 근접(진동/타이트한 회피)을 보인다.
export function headOnAgents(radius = 0.3): AgentSpec[] {
    return [
        spec(2.5, 7.3, 0, [12.5, 7.3], radius),
        spec(12.5, 7.7, Math.PI, [2.5, 7.7], radius),
    ]
}

// circle_swap: maps/scenarios/velocity/circle_swap.yaml과 동일한 좌표 -- 수평/수직
// 두 쌍이 0.9m 레인 오프셋을 두고 직교로 교차한다. 완전한 원형 대칭 스왑은 VO/RVO/
// ORCA 전부에게 진짜 교착(모든 후보 속도가 똑같이 막히는 상황)이라, 그 대칭을
// 오프셋으로 깨 네 몸체 모두 여유 있게 회피하고 도달하게 한다(그 yaml의 주석 그대로).
export function circleSwapAgents(radius = 0.3): AgentSpec[] {
    return [
        spec(1.5, 6.6, 0, [13.5, 6.6], radius),
        spec(13.5, 8.4, Math.PI, [1.5, 8.4], radius),
        spec(6.6, 3.0, Math.PI / 2, [6.6, 12.0], radius),
        spec(8.4, 12.0, -Math.PI / 2, [8.4, 3.0], radius),
    ]
}
