import {AlgoCategory, AlgoSection, Localized} from "../../../types/global";

// 홈 카드가 쓰는 알고리즘 한 줄 소개. 페이지를 집필하면 여기에 함께 적는다.
export interface AlgoBlurb {
    slug: string;
    blurb: Localized<string>;
}

export const ALGO_BLURBS: AlgoBlurb[] = [
    {
        slug: "bfs",
        blurb: {
            en: "Search in waves with a FIFO queue: the fewest-edge path on any graph, " +
                "and the seed every other planner in this section grows from.",
            ko: "FIFO 큐로 파도처럼 훑는 탐색. 어떤 그래프에서든 최소 edge 경로를 찾고, " +
                "이 섹션의 모든 planner가 여기서 자라난다.",
        },
    },
    {
        slug: "dijkstra",
        blurb: {
            en: "Swap BFS's FIFO for a priority queue on g and fewest-edge becomes " +
                "cheapest-cost: optimal for any non-negative edge costs.",
            ko: "BFS의 FIFO를 g 기준 priority queue로 바꾸면 최소 edge가 최소 비용이 " +
                "된다. 음수 아닌 모든 edge 비용에서 최적.",
        },
    },
    {
        slug: "dstar_lite",
        blurb: {
            en: "Planning when the map is a guess: drive, sense, and repair only the part " +
                "of the backward search a discovered wall invalidates.",
            ko: "지도가 추측일 때의 계획. 주행하고, 감지하고, 발견된 벽이 무효화한 " +
                "backward 탐색 부분만 수리한다.",
        },
    },
    {
        slug: "ara_star",
        blurb: {
            en: "Anytime A*: publish an ε-bounded path almost instantly, then tighten ε " +
                "and reuse the search until the answer is optimal.",
            ko: "Anytime A*. ε-한계가 보장된 경로를 거의 즉시 발표하고, ε을 조이며 " +
                "탐색을 재사용해 최적까지 간다.",
        },
    },
    {
        slug: "astar",
        blurb: {
            en: "Dijkstra plus a compass: order the frontier by g + h and an admissible " +
                "heuristic finds the same optimal path while expanding far fewer nodes.",
            ko: "Dijkstra에 나침반을 더한 탐색. frontier를 g + h로 정렬하면, admissible " +
                "heuristic 만으로 같은 최적 경로를 훨씬 적은 노드 확장으로 찾는다.",
        },
    },
];

// 중분류 — 알고리즘 계열. 페이지 eyebrow·홈 sub-heading·사이드바 sub-label이 공유한다.
export const CATEGORIES: Array<{
    key: AlgoCategory;
    title: Localized<string>;
}> = [
    {key: "search", title: {en: "Graph Search", ko: "Graph Search"}},
    {key: "sampling", title: {en: "Sampling", ko: "Sampling"}},
    {key: "local", title: {en: "Local Planning", ko: "Local Planning"}},
    {key: "multi", title: {en: "Multi-Agent", ko: "Multi-Agent"}},
];

// 대분류 — 홈의 큰 섹션이자 사이드바 disclosure 단위. 파일 구조(pages/algorithms/<section>/)도
// 이 구분을 따른다.
export const SECTIONS: Array<{
    key: AlgoSection;
    title: Localized<string>;
    desc: Localized<string>;
    categories: AlgoCategory[];
}> = [
    {
        key: "global",
        title: {en: "Global Planning", ko: "Global Planning"},
        desc: {
            en: "Start-to-goal path planning over the whole map: discrete search on grids " +
                "and graphs, and sampling-based planners for continuous spaces.",
            ko: "맵 전체를 보고 시작→목표 경로를 찾는 계획: 격자·그래프 위의 이산 탐색과, " +
                "연속 공간을 위한 sampling 기반 planner.",
        },
        categories: ["search", "sampling"],
    },
    {
        key: "local",
        title: {en: "Local Planning", ko: "Local Planning"},
        desc: {
            en: "Following the global path in the here-and-now: reactive avoidance and " +
                "local trajectory optimization — DWA, Pure Pursuit, VFH, MPC.",
            ko: "전역 경로를 지금-여기서 따라가는 계획: 반응형 회피와 지역 궤적 최적화 — " +
                "DWA, Pure Pursuit, VFH, MPC.",
        },
        categories: ["local"],
    },
    {
        key: "multi",
        title: {en: "Multi-Agent", ko: "Multi-Agent"},
        desc: {
            en: "Coordinating many robots without collisions: prioritized, joint-space, and conflict-based search.",
            ko: "여러 로봇을 충돌 없이 조율하는 계획: prioritized, joint-space, conflict-based search.",
        },
        categories: ["multi"],
    },
];

export const sectionOf = (category: AlgoCategory): AlgoSection =>
    SECTIONS.find((s) => s.categories.includes(category))?.key ?? "global"
