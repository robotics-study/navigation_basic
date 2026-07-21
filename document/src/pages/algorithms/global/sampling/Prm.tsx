import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import PrmSandbox from "../../../../components/panels/global/prm/PrmSandbox";
import prmPy from "../../../../../../python/navigation/global_planning/sampling/prm.py?raw";
import roadmapPy from "../../../../../../python/navigation/global_planning/sampling/_roadmap.py?raw";
import prmCpp from "../../../../../../cpp/src/global_planning/sampling/prm.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Prm = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every planner so far walked a structure we built by hand: a lattice of
                    cells, corners of obstacles. PRM (Kavraki, Švestka, Latombe &amp;
                    Overmars, 1996) starts the other family: <em>throw random points</em> into
                    free space, wire the ones that can see each other, and let graph search do
                    the rest. The roadmap doesn't care about grids, dimensions, or where the
                    query is — build it once, answer many start–goal queries against it.
                </p>}
                ko={<p>
                    지금까지의 planner는 손으로 만든 구조물 위를 걸었다. 셀 격자, 장애물
                    모서리. PRM(Kavraki, Švestka, Latombe &amp; Overmars, 1996)은 다른 가족의
                    시조다. free 공간에 <em>무작위 점을 뿌리고</em>, 서로 보이는 점끼리 잇고,
                    나머지는 그래프 탐색에 맡긴다. roadmap은 격자도, 차원도, 질의가 어디서
                    오는지도 신경 쓰지 않는다. 한 번 지어 두고 여러 시작–목표 질의에 재사용한다.
                </p>}
            />

            <h2>{t("Sample, Connect, Query", "뿌리고, 잇고, 묻는다")}</h2>
            <T
                en={<>
                    <p>
                        Three phases, cleanly separated. <strong>Sample</strong>: draw uniform
                        random states, keep the collision-free ones as roadmap nodes.{" "}
                        <strong>Connect</strong>: for each node, try a straight collision-free
                        motion to every earlier node within a fixed{" "}
                        <InlineMath math="r"/>; feasible pairs become undirected edges weighted
                        by Euclidean length. <strong>Query</strong>: insert start and goal as
                        nodes, then run Dijkstra over the roadmap.
                    </p>
                    <p>
                        The catch hides in the word <em>random</em>. Whether the roadmap
                        connects two rooms depends on whether samples happened to land on both
                        sides of the doorway:
                    </p>
                    <BlockMath math="\Pr[\text{no node in region } V] \;=\; \left(1 - \frac{\mu(V)}{\mu(C_{\text{free}})}\right)^{\!n}"/>
                    <Terms items={[
                        ["V", "the region that must be hit for connectivity — e.g. the neighborhood of a narrow doorway"],
                        ["\\mu(\\cdot)", "area (volume) of a region"],
                        ["C_{\\text{free}}", "the collision-free part of the state space, where samples are drawn uniformly"],
                        ["n", "number of samples in the roadmap"],
                    ]}/>
                    <p>
                        The failure probability decays exponentially in{" "}
                        <InlineMath math="n"/> — that is probabilistic completeness — but the
                        decay rate is proportional to the passage's volume. Narrow passages are
                        exactly where PRM struggles, and the sandbox below is built around one.
                    </p>
                </>}
                ko={<>
                    <p>
                        세 단계가 깔끔하게 분리된다. <strong>Sample</strong>: 균일 무작위
                        상태를 뽑아 충돌 없는 것만 roadmap 노드로 남긴다.{" "}
                        <strong>Connect</strong>: 각 노드를 고정 반경 <InlineMath math="r"/>{" "}
                        안의 앞선 노드들과 직선 충돌 검사로 이어 본다. 통과한 쌍이 유클리드
                        길이 가중치의 무향 간선이 된다. <strong>Query</strong>: 시작과 목표를
                        노드로 넣고 roadmap 위에서 Dijkstra를 돌린다.
                    </p>
                    <p>
                        함정은 <em>무작위</em>라는 단어에 숨어 있다. roadmap이 두 방을
                        이어 주는지는 표본이 문 양쪽에 우연히 떨어졌는지에 달려 있다:
                    </p>
                    <BlockMath math="\Pr[\text{영역 } V \text{에 노드 없음}] \;=\; \left(1 - \frac{\mu(V)}{\mu(C_{\text{free}})}\right)^{\!n}"/>
                    <Terms items={[
                        ["V", "연결에 꼭 맞아야 하는 영역. 예를 들어 좁은 문 주변"],
                        ["\\mu(\\cdot)", "영역의 넓이(부피)"],
                        ["C_{\\text{free}}", "상태 공간의 충돌 없는 부분. 표본은 여기서 균일하게 뽑힌다"],
                        ["n", "roadmap의 표본 수"],
                    ]}/>
                    <p>
                        실패 확률은 <InlineMath math="n"/>에 지수적으로 줄어든다. 그것이
                        확률적 완전성이다. 하지만 감쇠율이 통로의 부피에 비례하므로, 좁은
                        통로가 정확히 PRM의 약점이다. 아래 sandbox가 그 약점 하나를 중심으로
                        지어져 있다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Probabilistically complete</strong>: if a path exists, the
                        probability of finding one goes to 1 as <InlineMath math="n"/> grows —
                        but any finite roadmap can miss it (reroll the sandbox seed).</li>
                    <li><strong>Not optimal</strong>: the answer is the shortest path{" "}
                        <em>in the roadmap</em>, a polyline through random points. Its length
                        depends on <InlineMath math="n"/> and <InlineMath math="r"/>; PRM*
                        (next page) fixes the radius schedule to make it converge to the true
                        optimum.</li>
                    <li><strong>Multi-query</strong>: sampling and connection never look at
                        start or goal, so one roadmap amortizes over many queries — the trait
                        that separates PRM from the RRT family.</li>
                    <li><strong>Cost</strong>: naive connection tests all pairs,{" "}
                        <InlineMath math="O(n^2)"/> distance checks plus a collision test per
                        near pair; the query itself is a plain Dijkstra.</li>
                </ul>}
                ko={<ul>
                    <li><strong>확률적 완전</strong>: 경로가 존재하면 찾을 확률이{" "}
                        <InlineMath math="n"/>과 함께 1로 간다. 하지만 유한한 roadmap 은
                        언제든 놓칠 수 있다 (sandbox의 seed를 다시 추첨해 보라).</li>
                    <li><strong>최적이 아니다</strong>: 답은 <em>roadmap 안에서의</em> 최단
                        경로, 곧 무작위 점을 지나는 폴리라인이다. 길이는{" "}
                        <InlineMath math="n"/>과 <InlineMath math="r"/>에 달려 있다. PRM*
                        (다음 페이지)이 반경 스케줄을 고쳐 참 최적으로 수렴하게 만든다.</li>
                    <li><strong>Multi-query</strong>: 표본과 연결은 시작·목표를 전혀 보지
                        않으므로 roadmap 하나가 여러 질의에 상각된다. RRT 가족과 갈리는
                        지점이다.</li>
                    <li><strong>비용</strong>: 순진한 연결은 전 쌍 검사라{" "}
                        <InlineMath math="O(n^2)"/> 거리 계산에, 가까운 쌍마다 충돌 검사가
                        붙는다. 질의 자체는 평범한 Dijkstra다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The single-query form used in this repository (start and goal join the
                    roadmap before connection, so one pass answers one query):
                </p>}
                ko={<p>
                    이 저장소가 쓰는 single-query 형태다 (시작·목표가 연결 전에 roadmap 에
                    들어가므로 한 번의 패스가 한 질의에 답한다):
                </p>}
            />
            <Pseudocode code={`nodes ← [start, goal]
repeat until n free samples collected:                       # 1
    q ← uniform random state
    if q is collision-free:  append q to nodes
for each node v (in insertion order):                        # 2
    for each earlier node u with ‖u − v‖ ≤ r:                # 3
        if straight motion u → v is collision-free:
            add undirected edge (u, v) with weight ‖u − v‖
return dijkstra(nodes, edges, start, goal)                   # 4`}/>
            <T
                en={<ol>
                    <li>Rejection sampling: draw, test, keep. An attempt cap keeps a nearly
                        full map from looping forever.</li>
                    <li>Connecting each node only to <em>earlier</em> ones adds every
                        undirected edge exactly once.</li>
                    <li>Fixed radius <InlineMath math="r"/> for every node — this constant is
                        exactly what PRM* later replaces with a shrinking schedule.</li>
                    <li>The query is ordinary graph search; any shortest-path algorithm
                        works, and everything learned in the Graph Search section applies
                        unchanged.</li>
                </ol>}
                ko={<ol>
                    <li>기각 표집이다. 뽑고, 검사하고, 남긴다. 시도 상한이 거의 가득 찬
                        맵에서의 무한 루프를 막는다.</li>
                    <li>각 노드를 <em>앞선</em> 노드와만 이으면 무향 간선이 정확히 한 번씩
                        들어간다.</li>
                    <li>모든 노드에 같은 고정 반경 <InlineMath math="r"/>을 쓴다. PRM* 이
                        나중에 줄어드는 스케줄로 바꾸는 상수가 바로 이것이다.</li>
                    <li>질의는 평범한 그래프 탐색이다. 어떤 최단 경로 알고리즘이든 되고,
                        Graph Search 섹션에서 배운 것이 그대로 적용된다.</li>
                </ol>}
            />
            <Proof title={t("Why more samples always help (probabilistic completeness sketch)",
                "표본이 늘면 반드시 나아지는 이유 (확률적 완전성 스케치)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> A feasible path{" "}
                            <InlineMath math="\sigma"/> exists with clearance{" "}
                            <InlineMath math="\delta > 0"/>. Cover it with{" "}
                            <InlineMath math="k"/> balls of radius{" "}
                            <InlineMath math="\delta/2"/> spaced so that any choice of one
                            sample per ball yields consecutive samples that are mutually
                            visible and within <InlineMath math="r"/>.
                        </p>
                        <BlockMath math="\Pr[\text{failure}] \;\le\; \sum_{i=1}^{k} \Pr[\text{ball } B_i \text{ empty}] \;=\; k\left(1 - \tfrac{\mu(B)}{\mu(C_{\text{free}})}\right)^{\!n} \;\xrightarrow[n \to \infty]{}\; 0"/>
                        <Terms items={[
                            ["\\sigma,\\ \\delta", "a feasible path and its clearance: the distance it keeps from obstacles"],
                            ["B_i,\\ k", <>the covering balls of radius <InlineMath math="\delta/2"/> along <InlineMath math="\sigma"/> and how many are needed</>],
                            ["\\mu(B)", "area of one ball — fixed and positive, which drives the exponential decay"],
                            ["n", "number of samples"],
                        ]}/>
                        <p>
                            One sample per ball suffices for a connected roadmap path, so
                            failure requires some ball to stay empty — and that probability
                            vanishes exponentially. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 여유 간격 <InlineMath math="\delta > 0"/>을
                            가진 실행 가능 경로 <InlineMath math="\sigma"/>가 존재한다. 이
                            경로를 반지름 <InlineMath math="\delta/2"/>짜리 공{" "}
                            <InlineMath math="k"/>개로 덮되, 공마다 표본 하나씩만 있으면
                            이웃 표본끼리 서로 보이고 <InlineMath math="r"/> 안에 들도록
                            간격을 잡는다.
                        </p>
                        <BlockMath math="\Pr[\text{실패}] \;\le\; \sum_{i=1}^{k} \Pr[B_i \text{ 비어 있음}] \;=\; k\left(1 - \tfrac{\mu(B)}{\mu(C_{\text{free}})}\right)^{\!n} \;\xrightarrow[n \to \infty]{}\; 0"/>
                        <Terms items={[
                            ["\\sigma,\\ \\delta", "실행 가능 경로와 그 여유 간격. 장애물과 유지하는 거리다"],
                            ["B_i,\\ k", <><InlineMath math="\sigma"/>를 덮는 반지름 <InlineMath math="\delta/2"/> 공들과 그 개수</>],
                            ["\\mu(B)", "공 하나의 넓이. 고정된 양수라서 지수 감쇠를 만든다"],
                            ["n", "표본 수"],
                        ]}/>
                        <p>
                            공마다 표본이 하나씩만 있으면 roadmap 경로가 이어지므로, 실패는
                            어떤 공이 비어 있어야만 가능하다. 그 확률은 지수적으로 사라진다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox puts a narrow doorway between start and goal. At 60 samples
                    the rooms almost never connect; at 300 they almost always do — and the
                    resample button shows that everything in between is a coin flip. The
                    replay below is the repository demo on the benchmark maps: watch the three
                    phases (dots, edges, then a Dijkstra wave over the roadmap).
                </p>}
                ko={<p>
                    sandbox는 시작과 목표 사이에 좁은 문을 세워 둔다. 표본 60개면 두 방이
                    거의 이어지지 않고, 300개면 거의 항상 이어진다. 다시 추첨 버튼은 그
                    사이가 동전 던지기임을 보여 준다. 아래 replay는 벤치마크 맵 위의 저장소
                    demo다. 세 단계(점, 간선, 그리고 roadmap 위 Dijkstra 물결)를 눈으로
                    따라가 보라.
                </p>}
            />
            <PrmSandbox/>
            <TraceReplay algo="prm" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's PRM demo — samples land, the roadmap wires up, then Dijkstra answers the query over it",
                "저장소 PRM demo의 실제 trace. 표본이 떨어지고, roadmap이 이어지고, 그 위에서 Dijkstra가 질의에 답한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The roadmap container, connection step, and Dijkstra query are shared
                    plumbing (PRM* reuses them unchanged); PRM itself is the sampling loop
                    plus a fixed-radius connection pass. Embedded below in full.
                </p>}
                ko={<p>
                    roadmap 컨테이너, 연결 단계, Dijkstra 질의는 공유 배관이다 (PRM* 이
                    그대로 재사용한다). PRM 자체는 표본 루프에 고정 반경 연결 패스를 더한
                    것이다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/global_planning/sampling/prm.py",
                                code: prmPy,
                                href: `${REPO}/python/navigation/global_planning/sampling/prm.py`,
                            },
                            {
                                name: "python/navigation/global_planning/sampling/_roadmap.py",
                                code: roadmapPy,
                                href: `${REPO}/python/navigation/global_planning/sampling/_roadmap.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/prm.cpp",
                            code: prmCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/prm.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete PRM implementation, embedded from the repository sources",
                    "PRM 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    L. E. Kavraki, P. Švestka, J.-C. Latombe, M. H. Overmars,{" "}
                    <a href="https://doi.org/10.1109/70.508439" target="_blank"
                       rel="noopener noreferrer">
                        <em>Probabilistic Roadmaps for Path Planning in High-Dimensional
                            Configuration Spaces</em>
                    </a>,
                    IEEE Transactions on Robotics and Automation, 1996.
                </li>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank"
                       rel="noopener noreferrer">
                        <em>Sampling-based Algorithms for Optimal Motion Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2011.
                </li>
            </ol>
        </>
    )
}

export default Prm
