import {AlgoCategory, Localized} from "../../../types/global";

// 홈 카드가 쓰는 알고리즘 한 줄 소개. 페이지를 집필하면 여기에 함께 적는다.
export interface AlgoBlurb {
    slug: string;
    blurb: Localized<string>;
}

export const ALGO_BLURBS: AlgoBlurb[] = [
    {
        slug: "astar",
        blurb: {
            en: "Dijkstra plus a compass: order the frontier by g + h and an admissible " +
                "heuristic finds the same optimal path while expanding far fewer nodes.",
            ko: "Dijkstra에 나침반을 더한 탐색. frontier 를 g + h 로 정렬하면, admissible " +
                "heuristic 만으로 같은 최적 경로를 훨씬 적은 노드 확장으로 찾는다.",
        },
    },
];

// 홈에서 알고리즘을 묶어 보여 주는 카테고리 구분. range 대신 category 필드로 묶는다.
export const CATEGORIES: Array<{
    key: AlgoCategory;
    title: Localized<string>;
    desc: Localized<string>;
}> = [
    {
        key: "search",
        title: {en: "Graph Search", ko: "Graph Search"},
        desc: {
            en: "Discrete search over grids and graphs: from uninformed BFS to heuristic, " +
                "incremental, any-angle, and kinematically feasible variants.",
            ko: "격자·그래프 위의 이산 탐색: 무정보 BFS 에서 heuristic, incremental, " +
                "any-angle, 기구학 제약 변형까지.",
        },
    },
    {
        key: "sampling",
        title: {en: "Sampling-Based Planning", ko: "Sampling-Based Planning"},
        desc: {
            en: "Planning in continuous space by random sampling: RRT and PRM families, " +
                "and asymptotically optimal batch planners.",
            ko: "무작위 샘플링으로 연속 공간을 탐색하는 planner: RRT·PRM 계열과 " +
                "점근 최적 batch planner.",
        },
    },
    {
        key: "local",
        title: {en: "Local Planning", ko: "Local Planning"},
        desc: {
            en: "Reactive obstacle avoidance and path tracking on the robot: DWA, Pure Pursuit, VFH, MPC.",
            ko: "로봇 위에서 도는 반응형 회피·경로 추종: DWA, Pure Pursuit, VFH, MPC.",
        },
    },
    {
        key: "multi",
        title: {en: "Multi-Agent Planning", ko: "Multi-Agent Planning"},
        desc: {
            en: "Coordinating many robots without collisions: prioritized, joint-space, and conflict-based search.",
            ko: "여러 로봇을 충돌 없이 조율하는 계획: prioritized, joint-space, conflict-based search.",
        },
    },
];
