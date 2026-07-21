import {AlgoCategory, AlgoSection, Localized} from "../../../types/global";

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

// 중분류 — 알고리즘 계열. 페이지 eyebrow·홈 sub-heading·사이드바 sub-label 이 공유한다.
export const CATEGORIES: Array<{
    key: AlgoCategory;
    title: Localized<string>;
}> = [
    {key: "search", title: {en: "Graph Search", ko: "Graph Search"}},
    {key: "sampling", title: {en: "Sampling", ko: "Sampling"}},
    {key: "control", title: {en: "Control", ko: "Control"}},
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
        key: "planning",
        title: {en: "Planning", ko: "Planning"},
        desc: {
            en: "Global path planning: discrete search over grids and graphs, and " +
                "sampling-based planners for continuous spaces.",
            ko: "전역 경로 계획: 격자·그래프 위의 이산 탐색과, 연속 공간을 위한 " +
                "sampling 기반 planner.",
        },
        categories: ["search", "sampling"],
    },
    {
        key: "control",
        title: {en: "Control", ko: "Control"},
        desc: {
            en: "Reactive local planning and path tracking on the robot: DWA, Pure Pursuit, VFH, MPC.",
            ko: "로봇 위에서 도는 반응형 local planning 과 경로 추종: DWA, Pure Pursuit, VFH, MPC.",
        },
        categories: ["control"],
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
    SECTIONS.find((s) => s.categories.includes(category))?.key ?? "planning"
