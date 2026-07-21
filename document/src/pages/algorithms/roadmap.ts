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
        slug: "ad_star",
        blurb: {
            en: "ARA* and D* Lite fused: anytime ε-bounded plans while driving an " +
                "unknown map, repaired incrementally at every discovery.",
            ko: "ARA*와 D* Lite의 결합. 미지 지도를 주행하며 anytime ε-한계 계획을 " +
                "발표하고, 발견마다 incremental하게 수리한다.",
        },
    },
    {
        slug: "theta_star",
        blurb: {
            en: "Any-angle planning: try a straight line-of-sight shortcut to the " +
                "grandparent at every relaxation, and zigzags collapse into taut segments.",
            ko: "Any-angle planning. relaxation마다 조부모로의 직선 지름길을 시험하면 " +
                "지그재그가 팽팽한 직선 구간으로 접힌다.",
        },
    },
    {
        slug: "lazy_theta_star",
        blurb: {
            en: "Theta* with the expensive check deferred: assume line of sight at " +
                "generation, verify once per expansion, repair if the view was blocked.",
            ko: "비싼 검사를 미룬 Theta*. 생성 때는 line of sight를 가정하고, 확장마다 " +
                "한 번 확인하고, 막혀 있었으면 수리한다.",
        },
    },
    {
        slug: "jps",
        blurb: {
            en: "Kill grid symmetry with geometry: scan straight lines to the few jump " +
                "points where turning is forced — A*'s optimal paths at a fraction of the work.",
            ko: "기하로 grid 대칭을 제거한다. 꺾임이 강제되는 소수의 jump point까지 " +
                "직선 스캔해, A*의 최적 경로를 몇 분의 일의 작업으로 찾는다.",
        },
    },
    {
        slug: "visibility_astar",
        blurb: {
            en: "Make line of sight the successor relation itself: every visible cell is a " +
                "neighbor at straight-line cost, and A* returns the cell-centre visibility-graph optimum.",
            ko: "line of sight 자체를 successor 관계로 삼는다. 보이는 모든 셀이 직선거리 " +
                "비용의 이웃이 되고, A*가 셀 중심 visibility graph의 최적을 돌려준다.",
        },
    },
    {
        slug: "anya",
        blurb: {
            en: "Turn where obstacles actually bend: search corner roots with row-interval " +
                "sweeps and return the exact Euclidean shortest any-angle path.",
            ko: "장애물이 실제로 꺾이는 곳에서 꺾는다. corner root를 행 interval sweep으로 " +
                "탐색해 정확한 유클리드 최단 any-angle 경로를 돌려준다.",
        },
    },
    {
        slug: "hybrid_astar",
        blurb: {
            en: "Search the car's own pose space: constant-curvature arc primitives, a " +
                "binned closed set, and paths a real vehicle can actually drive.",
            ko: "차량의 pose 공간을 직접 탐색한다. 일정 곡률 arc primitive와 bin " +
                "closed set으로, 실제 차량이 달릴 수 있는 경로를 찾는다.",
        },
    },
    {
        slug: "prm",
        blurb: {
            en: "Throw random points into free space, wire the ones that can see each " +
                "other, and answer start–goal queries with graph search over the roadmap.",
            ko: "free 공간에 무작위 점을 뿌리고 서로 보이는 점끼리 이어, roadmap 위 " +
                "그래프 탐색으로 시작–목표 질의에 답한다.",
        },
    },
    {
        slug: "prm_star",
        blurb: {
            en: "PRM whose connection radius shrinks as γ(log n/n)^(1/d): Θ(log n) " +
                "neighbors per node, and the roadmap path converges to the true optimum.",
            ko: "연결 반경이 γ(log n/n)^(1/d)로 줄어드는 PRM. 노드마다 Θ(log n)개의 " +
                "이웃을 유지해, roadmap 경로가 참 최적으로 수렴한다.",
        },
    },
    {
        slug: "rrt",
        blurb: {
            en: "Grow a tree from the start, one steered step toward each random sample: " +
                "Voronoi bias pulls it into unexplored space until a branch touches the goal.",
            ko: "시작점에서 트리를 키운다. 무작위 표본 쪽으로 한 스텝씩. Voronoi bias가 " +
                "미탐사 공간으로 트리를 끌고 가고, 가지가 goal에 닿으면 끝난다.",
        },
    },
    {
        slug: "rrt_connect",
        blurb: {
            en: "Two trees, one from each end: EXTEND explores, CONNECT sprints greedily " +
                "at the other tree's newest node, and the fronts splice in the middle.",
            ko: "양끝에서 트리 하나씩. EXTEND가 탐사하고 CONNECT가 상대 트리의 최신 " +
                "노드로 내달려, 두 전선이 가운데에서 접합된다.",
        },
    },
    {
        slug: "rrt_star",
        blurb: {
            en: "RRT that never settles: every new node shops for the cheapest parent, " +
                "then rewires its neighbors — and the path converges to the optimum.",
            ko: "안주하지 않는 RRT. 새 노드마다 가장 싼 부모를 골라잡고 이웃을 다시 " +
                "배선해, 경로가 최적으로 수렴한다.",
        },
    },
    {
        slug: "informed_rrt_star",
        blurb: {
            en: "RRT* that stops wasting samples: once a path exists, draw only from the " +
                "ellipse of states that could still beat it, and the cost converges far faster.",
            ko: "표본을 낭비하지 않는 RRT*. 경로가 하나 생기면 그것을 아직 이길 수 있는 " +
                "상태들의 타원 안에서만 표본을 뽑아, 비용이 훨씬 빨리 수렴한다.",
        },
    },
    {
        slug: "fast_rrt",
        blurb: {
            en: "RRT* with three accelerators: sample only where the tree hasn't reached, " +
                "rescue a blocked extension with a random probe, and shortcut every path taut.",
            ko: "가속기 셋을 단 RRT*. 트리가 닿지 않은 곳만 표본으로 뽑고, 막힌 확장은 " +
                "무작위 탐침으로 살리며, 모든 경로를 지름길로 팽팽하게 당긴다.",
        },
    },
    {
        slug: "fmt_star",
        blurb: {
            en: "PRM*'s batch and shrinking radius, marched like Dijkstra: one " +
                "cost-ordered wave builds the tree, collision-checking a single lazy " +
                "edge per node instead of every neighbor.",
            ko: "PRM*의 배치와 줄어드는 반경을 Dijkstra처럼 행진한다. cost 순 파면 " +
                "한 번이 트리를 세우되, 이웃마다가 아니라 노드마다 lazy 간선 하나만 " +
                "충돌 검사한다.",
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
