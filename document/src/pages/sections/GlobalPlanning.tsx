import {ReactNode} from "react";
import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import DiscretizeVsSample from "../../components/panels/intro/DiscretizeVsSample";

// navigation stack 파이프라인 다이어그램 — 외부 자산 없이 토큰 색만으로 그린다.
const StackBox = ({label, sub, accent}: {label: string; sub: string; accent?: boolean}) => (
    <div className={`flex-1 min-w-[120px] rounded-xl border px-3 py-2 text-center ${
        accent ? "border-[var(--accent)] bg-[color-mix(in_srgb,var(--accent)_10%,transparent)]" : "border-border bg-surface"}`}>
        <div className="font-semibold text-sm">{label}</div>
        <div className="text-xs text-muted">{sub}</div>
    </div>
)

const Arrow = ({label}: {label: string}) => (
    <div className="flex flex-col items-center justify-center px-1 text-muted">
        <span className="text-xs">{label}</span>
        <span aria-hidden="true">→</span>
    </div>
)

const StackDiagram = ({children}: {children?: ReactNode}) => {
    const t = useTr()
    return (
        <div className="my-5">
            <div className="flex flex-wrap items-stretch gap-1.5">
                <StackBox label={t("Map + Goal", "Map + Goal")} sub={t("known world", "알려진 지도")}/>
                <Arrow label="plan"/>
                <StackBox label="Global Planner" sub={t("this section", "이 섹션")} accent/>
                <Arrow label="path"/>
                <StackBox label="Local Planner" sub={t("follow + avoid", "추종 + 회피")}/>
                <Arrow label="cmd"/>
                <StackBox label={t("Robot", "로봇")} sub={t("sensors feed back", "센서 피드백")}/>
            </div>
            {children}
        </div>
    )
}

const GlobalPlanning = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Global planning answers the first question of navigation: <em>given a map, how do
                    I get from here to there at all?</em> Everything else in the stack — trajectory
                    tracking, obstacle dodging, coordination — assumes some notion of a route already
                    exists. This section studies the algorithms that produce that route.
                </p>}
                ko={<p>
                    Global planning은 navigation의 첫 질문에 답한다: <em>지도가 주어졌을 때,
                    여기서 저기까지 도대체 어떻게 가는가?</em> trajectory 추종, 장애물 회피,
                    다중 로봇 조율 같은 스택의 나머지는 모두 "경로가 이미 있다"는 전제 위에서
                    동작한다. 이 섹션은 그 경로를 만들어 내는 알고리즘들을 다룬다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<>
                    <p>
                        Formally: given a map of the environment, a start
                        state <InlineMath math="s"/>, and a goal state <InlineMath math="g"/>, find a
                        collision-free path from <InlineMath math="s"/> to <InlineMath math="g"/> —
                        ideally one that minimizes a cost such as length, and ideally with a
                        guarantee (optimality, or a bound on suboptimality). Three assumptions
                        distinguish it from what comes later in the stack:
                    </p>
                    <ul>
                        <li>The map is <strong>known in advance</strong> (occupancy grid, graph, or
                            geometric obstacle list).</li>
                        <li>Planning happens <strong>before moving</strong> — we can afford to think
                            in milliseconds-to-seconds, not microseconds.</li>
                        <li>The robot is simplified — usually a point or disk; kinematics and
                            dynamics are mostly someone else's problem (with exceptions like
                            Hybrid A* and kinodynamic planners).</li>
                    </ul>
                </>}
                ko={<>
                    <p>
                        형식적으로: 환경 지도와 시작 상태 <InlineMath math="s"/>, 목표 상태{" "}
                        <InlineMath math="g"/>가 주어졌을 때, <InlineMath math="s"/>에서{" "}
                        <InlineMath math="g"/>로 가는 충돌 없는 경로를 찾는다. 가능하면 길이 같은
                        비용을 최소화하고, 최적성이나 준최적 한계 같은 보장도 갖추면 좋다. 스택의
                        뒷단과 구분되는 가정이 셋 있다:
                    </p>
                    <ul>
                        <li>지도를 <strong>미리 알고 있다</strong> (occupancy grid, graph, 기하
                            장애물 리스트).</li>
                        <li><strong>움직이기 전에</strong> 계획한다. 마이크로초가 아니라
                            밀리초에서 초 단위로 생각할 여유가 있다.</li>
                        <li>로봇을 단순화한다. 보통 점이나 원판으로 취급하고, 기구학·동역학은
                            대개 다른 단계의 몫이다 (Hybrid A*, kinodynamic 계열 같은 예외가
                            있다).</li>
                    </ul>
                </>}
            />

            <h2>{t("Where It Sits in the Navigation Stack", "Navigation Stack에서의 위치")}</h2>
            <StackDiagram/>
            <T
                en={<p>
                    The global planner consumes the map and produces a path; the local planner
                    consumes that path and produces motor commands, reacting to obstacles the map
                    didn't know about. The division of labor is deliberate: the global planner is
                    allowed to be slow and thorough because it runs rarely, while the local planner
                    must be fast and myopic because it runs every control cycle. When the world
                    changes enough that the path becomes invalid, the global planner replans —
                    incremental algorithms (D* Lite, AD*) exist precisely to make that replanning
                    cheap.
                </p>}
                ko={<p>
                    Global planner는 지도를 소비해 경로를 내놓고, local planner는 그 경로를
                    소비해 모터 명령을 내놓으며 지도에 없던 장애물에 반응한다. 이 분업은 의도된
                    것이다: global planner는 드물게 돌기 때문에 느리고 꼼꼼해도 되고, local
                    planner는 매 제어 주기마다 돌기 때문에 빠르고 근시안적이어야 한다. 세상이
                    변해 경로가 무효가 되면 global planner가 다시 계획한다. incremental 계열
                    (D* Lite, AD*)은 바로 그 재계획을 싸게 만들기 위해 존재한다.
                </p>}
            />

            <h2>{t("Two Families: Search and Sampling", "두 계열: Search와 Sampling")}</h2>
            <T
                en={<>
                    <p>
                        The fundamental obstacle is that the space of positions is continuous. The
                        two families in this section are two answers to "how do you search an
                        uncountable set":
                    </p>
                    <ul>
                        <li>
                            <strong>Graph search</strong> discretizes first — impose a grid or graph,
                            then search it exactly. You inherit strong guarantees (completeness,
                            optimality on the graph) at the cost of resolution: the answer is only as
                            good as the discretization, and grids explode combinatorially with
                            dimension.
                        </li>
                        <li>
                            <strong>Sampling</strong> keeps the space continuous and probes it with
                            random samples, connecting them into trees (RRT family) or roadmaps (PRM
                            family). Guarantees weaken to <em>probabilistic</em> completeness and{" "}
                            <em>asymptotic</em> optimality — but the approach scales to high
                            dimensions and awkward constraint sets where grids are hopeless.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <p>
                        근본 장애물은 위치 공간이 연속이라는 것이다. 이 섹션의 두 계열은 "셀 수
                        없는 집합을 어떻게 탐색하는가"에 대한 두 가지 답이다:
                    </p>
                    <ul>
                        <li>
                            <strong>Graph search</strong>는 먼저 이산화한다. 격자나 그래프를 씌운 뒤
                            그것을 정확하게 탐색한다. 강한 보장(완전성, 그래프 위 최적성)을 얻는
                            대신 해상도가 대가다. 답의 품질은 이산화 해상도가 결정하고, 격자
                            크기는 차원에 따라 조합적으로 폭발한다.
                        </li>
                        <li>
                            <strong>Sampling</strong>은 공간을 연속인 채로 두고 무작위 샘플로
                            찔러 본 뒤, 트리(RRT 계열)나 roadmap(PRM 계열)으로 잇는다. 보장은{" "}
                            <em>확률적</em> 완전성과 <em>점근적</em> 최적성으로 약해지지만, 격자로는
                            감당할 수 없는 고차원 공간과 복잡한 제약으로도 확장된다.
                        </li>
                    </ul>
                </>}
            />
            <DiscretizeVsSample/>
            <table>
                <thead>
                <tr>
                    <th></th>
                    <th>Graph Search</th>
                    <th>Sampling</th>
                </tr>
                </thead>
                <tbody>
                <tr>
                    <td>{t("space", "공간")}</td>
                    <td>{t("discretized (grid/graph)", "이산화 (grid/graph)")}</td>
                    <td>{t("continuous", "연속")}</td>
                </tr>
                <tr>
                    <td>{t("completeness", "완전성")}</td>
                    <td>{t("complete (on the graph)", "완전 (그래프 위에서)")}</td>
                    <td>{t("probabilistically complete", "확률적 완전")}</td>
                </tr>
                <tr>
                    <td>{t("optimality", "최적성")}</td>
                    <td>{t("optimal (A*, admissible h)", "최적 (A*, admissible h)")}</td>
                    <td>{t("asymptotically optimal (RRT*, BIT*)", "점근 최적 (RRT*, BIT*)")}</td>
                </tr>
                <tr>
                    <td>{t("scales with", "규모 한계")}</td>
                    <td>{t("map size / dimension", "맵 크기·차원")}</td>
                    <td>{t("narrow passages", "좁은 통로")}</td>
                </tr>
                <tr>
                    <td>{t("representative", "대표 알고리즘")}</td>
                    <td>Dijkstra, A*, D* Lite, Theta*, JPS</td>
                    <td>RRT, RRT*, PRM, Informed RRT*, BIT*</td>
                </tr>
                </tbody>
            </table>

            <h2>{t("How This Repository Abstracts It", "이 저장소의 추상화")}</h2>
            <T
                en={<>
                    <p>
                        Every algorithm here is implemented twice — independently in C++ and Python —
                        against the same abstractions, so the two implementations can be compared
                        event-for-event:
                    </p>
                    <ul>
                        <li><strong>Capabilities, not map types.</strong> A planner declares what it
                            needs (<code>DiscreteSpace</code> for search,{" "}
                            <code>SamplingSpace</code> for sampling planners) and any map that
                            supports the capability works. Adding a map type touches no algorithm
                            code.</li>
                        <li><strong>Declared parameters.</strong> Each algorithm declares its
                            parameters with types, defaults, and valid ranges; values load
                            from <code>configs/</code> yaml shared by both languages.</li>
                        <li><strong>Trace events.</strong> Planners emit a JSON event stream
                            (<code>node_expanded</code>, <code>sample_drawn</code>,{" "}
                            <code>path_found</code>, …). The demos on these pages replay exactly that
                            stream — the visualizations never reach into algorithm internals.</li>
                    </ul>
                </>}
                ko={<>
                    <p>
                        여기의 모든 알고리즘은 같은 추상화 위에서 C++/Python으로 각각 독립적으로
                        구현된다. 그래서 두 구현을 이벤트 단위로 비교할 수 있다:
                    </p>
                    <ul>
                        <li><strong>맵 타입이 아니라 capability.</strong> planner는 자기가 필요한
                            것(search는 <code>DiscreteSpace</code>, sampling 은{" "}
                            <code>SamplingSpace</code>)을 선언하고, 그 capability를 지원하는 맵은
                            무엇이든 붙는다. 맵 타입을 추가해도 알고리즘 코드는 안 바뀐다.</li>
                        <li><strong>선언된 parameter.</strong> 각 알고리즘은 parameter를 타입·기본값·
                            유효 범위와 함께 선언하고, 값은 두 언어가 공유하는{" "}
                            <code>configs/</code> yaml에서 로드된다.</li>
                        <li><strong>Trace 이벤트.</strong> planner는 JSON 이벤트 스트림
                            (<code>node_expanded</code>, <code>sample_drawn</code>,{" "}
                            <code>path_found</code>, …)을 방출한다. 이 페이지들의 demo는 정확히 그
                            스트림을 재생한다. 시각화는 알고리즘 내부를 만지지 않는다.</li>
                    </ul>
                </>}
            />

            <h2>{t("Suggested Reading Order", "권장 읽기 순서")}</h2>
            <T
                en={<>
                    <p>
                        The pages build on each other. A sensible path through graph search:{" "}
                        <strong>BFS → Dijkstra → A*</strong> establishes costs and heuristics; then
                        anytime/incremental variants (<strong>ARA*, D* Lite, AD*</strong>) relax
                        "plan once, perfectly"; any-angle methods (<strong>Theta*, Anya</strong>)
                        remove the grid's 45° artifacts; <strong>JPS</strong> exploits grid symmetry
                        for speed; <strong>Hybrid A*</strong> adds vehicle kinematics. For sampling:{" "}
                        <strong>RRT → RRT-Connect → RRT*</strong> builds the core ideas, then
                        informed and batch variants (<strong>Informed RRT*, FMT*, BIT*</strong> and
                        successors) sharpen them.
                    </p>
                </>}
                ko={<>
                    <p>
                        각 페이지는 앞 내용을 전제로 한다. graph search 는{" "}
                        <strong>BFS → Dijkstra → A*</strong>로 비용과 heuristic을 다지고,
                        anytime/incremental 변형(<strong>ARA*, D* Lite, AD*</strong>)이 "한 번에
                        완벽하게"라는 가정을 풀고, any-angle 계열(<strong>Theta*, Anya</strong>)이
                        격자 특유의 45° 꺾임을 없애고, <strong>JPS</strong>는 격자 대칭성으로 속도를
                        얻고, <strong>Hybrid A*</strong>는 차량 기구학을 더한다. sampling 은{" "}
                        <strong>RRT → RRT-Connect → RRT*</strong>로 핵심 아이디어를 세운 뒤
                        informed·batch 변형(<strong>Informed RRT*, FMT*, BIT*</strong>와 후속들)이
                        그것을 다듬는다.
                    </p>
                </>}
            />
        </>
    )
}

export default GlobalPlanning
