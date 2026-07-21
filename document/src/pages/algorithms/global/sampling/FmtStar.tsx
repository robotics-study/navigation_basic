import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import FmtStarSandbox from "../../../../components/panels/global/fmt_star/FmtStarSandbox";
import fmtStarPy from "../../../../../../python/navigation/global_planning/sampling/fmt_star.py?raw";
import fmtStarCpp from "../../../../../../cpp/src/global_planning/sampling/fmt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const FmtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    RRT* earns optimality by never settling: every new node shops for a
                    cheaper parent and rewires its neighbors, and every candidate edge is
                    collision-checked. FMT* (Janson, Schmerling, Clark &amp; Pavone, 2015)
                    reaches the same optimum with neither. It fixes one batch of samples, then
                    marches a single tree outward from the start in order of cost-to-come, and
                    at each node it collision-checks exactly one edge — the locally cheapest
                    connection. One pass, no rewiring, a fraction of the collision checks.
                </p>}
                ko={<p>
                    RRT*는 안주하지 않음으로써 최적성을 얻는다. 새 노드마다 더 싼 부모를
                    찾고 이웃을 다시 배선하며, 후보 간선을 모두 충돌 검사한다. FMT*(Janson,
                    Schmerling, Clark &amp; Pavone, 2015)는 둘 다 없이 같은 최적에 닿는다.
                    표본 한 배치를 고정한 뒤 시작점에서 트리 하나를 cost-to-come 순으로 바깥을
                    향해 행진시키고, 노드마다 정확히 간선 하나, 곧 근방에서 가장 싼 연결만
                    충돌 검사한다. 한 번의 패스, rewire 없음, 충돌 검사는 몇 분의 일이다.
                </p>}
            />

            <h2>{t("From Rewiring to Marching Order", "rewire에서 행진 순서로")}</h2>
            <T
                en={<>
                    <p>
                        FMT* keeps PRM*'s two good ideas — one batch of samples and the
                        shrinking connection radius — and throws away the graph search that
                        follows. In its place it borrows Dijkstra's wavefront. An{" "}
                        <em>open</em> frontier of settled-cost nodes marches outward; the
                        lowest-cost frontier node <InlineMath math="z"/> pulls each of its
                        still-unconnected neighbors into the tree through that neighbor's
                        cheapest open connection. Because the front always advances from the
                        cheapest node, a sample's cost-to-come is already final the moment it
                        joins — the level-set sweep the name <em>fast marching</em> nods to.
                    </p>
                    <p>
                        The one twist that makes it fast is <strong>laziness</strong>. When a
                        neighbor <InlineMath math="x"/> is about to join, FMT* computes its
                        cheapest open connector on the <em>collision-oblivious</em> graph and
                        collision-checks only that single edge. RRT* and PRM* instead test
                        every feasible neighbor edge; FMT* bets that the geometrically cheapest
                        one is usually free, and defers <InlineMath math="x"/> to a later{" "}
                        <InlineMath math="z"/> on the rare miss. That bet turns an{" "}
                        <InlineMath math="\Theta(n \log n)"/> pile of collision checks into
                        roughly one per node.
                    </p>
                </>}
                ko={<>
                    <p>
                        FMT*는 PRM*의 좋은 두 아이디어, 곧 표본 한 배치와 줄어드는 연결
                        반경을 그대로 두고, 그 뒤에 오던 그래프 탐색만 버린다. 대신 Dijkstra의
                        파면을 빌린다. 비용이 확정된 노드들의 <em>open</em> frontier가 바깥으로
                        행진하고, 그중 비용이 가장 낮은 frontier 노드{" "}
                        <InlineMath math="z"/>가 아직 연결되지 않은 이웃들을 각자의 가장 싼
                        open 연결을 통해 트리로 끌어들인다. 파면이 항상 가장 싼 노드에서
                        전진하므로, 표본의 cost-to-come은 트리에 합류하는 순간 이미 확정된다.{" "}
                        <em>fast marching</em>이라는 이름이 가리키는 level-set 훑기다.
                    </p>
                    <p>
                        빠르게 만드는 한 가지 비틀기는 <strong>게으름</strong>이다. 이웃{" "}
                        <InlineMath math="x"/>가 합류하려 할 때, FMT*는 충돌을 무시한 그래프
                        위에서 가장 싼 open 연결을 계산하고 그 간선 하나만 충돌 검사한다.
                        RRT*와 PRM*는 실행 가능한 이웃 간선을 모두 검사한다. FMT*는 기하적으로
                        가장 싼 간선이 대개 비어 있다는 데 걸고, 드물게 빗나가면{" "}
                        <InlineMath math="x"/>를 뒤의 <InlineMath math="z"/>로 미룬다. 이
                        내기가 <InlineMath math="\Theta(n \log n)"/> 더미의 충돌 검사를 노드당
                        대략 한 번으로 바꾼다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Asymptotically optimal</strong>: with the same radius{" "}
                        <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/> as PRM* and{" "}
                        <InlineMath math="\gamma"/> above the threshold, the returned cost
                        converges to the optimum almost surely as{" "}
                        <InlineMath math="n \to \infty"/>.</li>
                    <li><strong>Probabilistically complete</strong>, over the fixed batch: if a
                        solution exists at the sample density, the march finds it.</li>
                    <li><strong>Single-query</strong>, unlike PRM*: the tree is rooted at the
                        start, so it answers one start–goal pair; a new query re-runs the
                        march.</li>
                    <li><strong>Far fewer collision checks</strong>: roughly one per connected
                        node instead of one per feasible neighbor edge — the practical win over
                        PRM*/RRT*, and the readout the sandbox measures.</li>
                    <li><strong>Lazy suboptimality</strong>: because only the cheapest edge is
                        tested, a node can end up connected through a costlier route when that
                        edge collides. The gap vanishes as <InlineMath math="n \to \infty"/>,
                        but at finite <InlineMath math="n"/> FMT* is not guaranteed to match the
                        exact sample-graph optimum.</li>
                    <li><strong>Cost</strong>: the marching itself is Dijkstra-like,{" "}
                        <InlineMath math="O(n \log n)"/> over the <InlineMath math="\Theta(n \log n)"/>{" "}
                        radius-graph edges; this repository's naive radius graph is{" "}
                        <InlineMath math="O(n^2)"/> distance checks to build.</li>
                </ul>}
                ko={<ul>
                    <li><strong>점근 최적</strong>: PRM*와 같은 반경{" "}
                        <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/>에{" "}
                        <InlineMath math="\gamma"/>가 임계값을 넘으면, 반환 비용이{" "}
                        <InlineMath math="n \to \infty"/>에서 최적으로 거의 확실히 수렴한다.</li>
                    <li>고정 배치 위에서 <strong>확률적 완전</strong>이다. 그 표본 밀도에서 해가
                        있으면 행진이 그것을 찾는다.</li>
                    <li>PRM*와 달리 <strong>single-query</strong>다. 트리가 시작점에 뿌리를
                        두므로 하나의 시작–목표 쌍에 답하고, 새 질의는 행진을 다시 돌린다.</li>
                    <li><strong>충돌 검사가 훨씬 적다</strong>: 실행 가능한 이웃 간선마다가
                        아니라 연결된 노드마다 대략 한 번이다. PRM*/RRT* 대비 실전 이점이자
                        sandbox가 재는 값이다.</li>
                    <li><strong>Lazy 준최적</strong>: 가장 싼 간선만 검사하므로, 그 간선이
                        막히면 노드가 더 비싼 경로로 연결될 수 있다. 이 격차는{" "}
                        <InlineMath math="n \to \infty"/>에서 사라지지만, 유한한{" "}
                        <InlineMath math="n"/>에서 FMT*는 표본 그래프의 정확한 최적과 일치한다는
                        보장이 없다.</li>
                    <li><strong>비용</strong>: 행진 자체는 Dijkstra류로{" "}
                        <InlineMath math="\Theta(n \log n)"/>개의 반경 그래프 간선 위에서{" "}
                        <InlineMath math="O(n \log n)"/>이다. 이 저장소의 순진한 반경 그래프
                        구축은 <InlineMath math="O(n^2)"/> 거리 계산이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The state is four arrays over the sample set — cost-to-come, parent, an{" "}
                    <em>open</em> flag, and a visited flag — plus a min-heap of open nodes keyed
                    on cost. The heap is the marching front; each outer step closes the cheapest
                    open node <InlineMath math="z"/> and grows the tree into its unvisited
                    neighbors before advancing.
                </p>}
                ko={<p>
                    상태는 표본 집합 위의 배열 넷, 곧 cost-to-come, parent,{" "}
                    <em>open</em> 플래그, visited 플래그와, 비용을 키로 하는 open 노드 min-heap
                    이다. 이 heap이 행진 파면이다. 바깥 스텝마다 가장 싼 open 노드{" "}
                    <InlineMath math="z"/>를 닫고, 전진하기 전에 그 미방문 이웃으로 트리를
                    키운다.
                </p>}
            />
            <Pseudocode code={`V ← [start, goal] + n free samples
r ← γ · (log |V| / |V|)^(1/d)                     # 1  batch radius (same as PRM*)
cost[start] ← 0;  Open ← {start};  z ← start
loop:
    for each unvisited x with ‖x − z‖ ≤ r:        # 2
        y* ← argmin over open y (‖y−x‖ ≤ r) of cost[y] + ‖y − x‖   # 3
        if y* exists and motion y* → x is collision-free:          # 4
            cost[x] ← cost[y*] + ‖y* − x‖;  parent[x] ← y*;  add x to Open
    remove z from Open                            # 5
    z ← lowest-cost node in Open
    if Open is empty: return failure              # 6
    if z = goal: return path(goal)                # 7`}/>
            <T
                en={<ol>
                    <li>The connection radius is computed once, from the final sample count
                        including start and goal — the exact PRM* formula. Batch construction
                        permits the single computation.</li>
                    <li>Only the frontier node's still-<em>unvisited</em> neighbors are
                        considered. This is what makes each sample enter the tree at most once,
                        with no later rewiring.</li>
                    <li>The dynamic-programming step: among <InlineMath math="x"/>'s neighbors
                        already settled (in <em>Open</em>), pick the one giving the lowest
                        cost-to-come. On the collision-free graph this is exactly{" "}
                        <InlineMath math="x"/>'s optimal cost.</li>
                    <li>The one lazy collision check. FMT* tests only that locally cheapest
                        edge, not every feasible neighbor. If it collides,{" "}
                        <InlineMath math="x"/> is left unvisited and may connect from a later,
                        higher-cost <InlineMath math="z"/> — the source of the lazy
                        suboptimality above.</li>
                    <li>Marching order. Closing <InlineMath math="z"/> and popping the
                        least-cost open node is Dijkstra's wavefront; since cost-to-come only
                        rises along the pop order and no settled cost is ever lowered, one pass
                        suffices.</li>
                    <li>An empty frontier means the goal is unreachable on this sample set —
                        densify (raise <InlineMath math="n"/>) and re-run.</li>
                    <li>The goal is detected when it is <em>popped</em> from Open, not when it
                        is first connected — the same pop-time test as Dijkstra and A*, so the
                        cost is final on arrival.</li>
                </ol>}
                ko={<ol>
                    <li>연결 반경은 시작·목표를 포함한 최종 표본 수로 한 번만 계산한다. PRM*와
                        같은 공식이고, batch 구축이라 한 번 계산이 가능하다.</li>
                    <li>frontier 노드의 아직 <em>미방문</em>인 이웃만 본다. 각 표본이 트리에
                        많아야 한 번 들어오고 뒤에 rewire가 없는 이유다.</li>
                    <li>동적 계획 스텝이다. 이미 확정된(<em>Open</em>){" "}
                        <InlineMath math="x"/>의 이웃 중 cost-to-come을 가장 낮게 만드는 것을
                        고른다. 충돌 없는 그래프에서는 이것이 정확히{" "}
                        <InlineMath math="x"/>의 최적 비용이다.</li>
                    <li>단 한 번의 lazy 충돌 검사다. FMT*는 실행 가능한 모든 이웃이 아니라 그
                        근방 최소비용 간선만 검사한다. 막히면{" "}
                        <InlineMath math="x"/>는 미방문으로 남아 뒤의 더 비싼{" "}
                        <InlineMath math="z"/>에서 연결될 수 있다. 위 lazy 준최적의 원천이다.</li>
                    <li>행진 순서다. <InlineMath math="z"/>를 닫고 최소비용 open 노드를 꺼내는
                        것이 Dijkstra의 파면이다. cost-to-come이 pop 순서를 따라 오르기만 하고
                        확정된 비용이 낮아지는 일이 없으므로, 한 번의 패스로 충분하다.</li>
                    <li>frontier가 비면 이 표본 집합에서 목표에 도달할 수 없다는 뜻이다.{" "}
                        <InlineMath math="n"/>을 키워 다시 돌린다.</li>
                    <li>목표는 Open에서 <em>꺼낼</em> 때 판정한다. 처음 연결될 때가 아니다.
                        Dijkstra·A*와 같은 pop 시점 검사라, 도달하는 순간 비용이 확정이다.</li>
                </ol>}
            />
            <Proof title={t("Why the march needs no rewiring", "행진에 rewire가 필요 없는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Assumptions.</strong> Edge weights are Euclidean, so{" "}
                            <InlineMath math="\|y - x\| \ge 0"/>. FMT* pops open nodes in
                            nondecreasing cost and never re-opens a closed node. Let{" "}
                            <InlineMath math="z"/> be the node just popped, with cost-to-come{" "}
                            <InlineMath math="g(z)"/>.
                        </p>
                        <p>
                            Any node <InlineMath math="x"/> still open satisfies
                        </p>
                        <BlockMath math="g(x) \;\ge\; g(z),"/>
                        <Terms items={[
                            ["g(z)", <>cost-to-come of the popped node <InlineMath math="z"/>: the length of its tree path back to the start</>],
                            ["g(x)", <>cost-to-come of any other open node <InlineMath math="x"/></>],
                        ]}/>
                        <p>
                            because the heap returns the minimum. Any future edge into{" "}
                            <InlineMath math="z"/> would route through some open node{" "}
                            <InlineMath math="y"/> with <InlineMath math="g(y) \ge g(z)"/>, so
                            its candidate cost is
                        </p>
                        <BlockMath math="g(y) + \|y - z\| \;\ge\; g(z) + 0 \;=\; g(z)."/>
                        <Terms items={[
                            ["g(y)", <>cost-to-come of a would-be later parent <InlineMath math="y"/>, itself still open</>],
                            ["\\|y - z\\|", <>the Euclidean length of the new edge, <strong>the new term</strong>, and nonnegative</>],
                            ["g(z)", "cost-to-come of the popped node, fixed at this moment"],
                        ]}/>
                        <p>
                            <strong>Conclusion.</strong> No later connection can lower{" "}
                            <InlineMath math="g(z)"/>, so it is final at pop time — the tree
                            edge chosen for <InlineMath math="z"/> is optimal among the
                            collision-free edges, and no rewiring pass can improve it. This is
                            Dijkstra's argument, and it is exactly why FMT* settles each cost
                            once. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 간선 가중치는 유클리드 거리라{" "}
                            <InlineMath math="\|y - x\| \ge 0"/>이다. FMT*는 open 노드를 비용
                            비감소 순으로 꺼내고, 닫힌 노드를 다시 열지 않는다. 방금 꺼낸
                            노드를 <InlineMath math="z"/>, 그 cost-to-come을{" "}
                            <InlineMath math="g(z)"/>라 하자.
                        </p>
                        <p>
                            아직 open인 임의의 노드 <InlineMath math="x"/>는
                        </p>
                        <BlockMath math="g(x) \;\ge\; g(z),"/>
                        <Terms items={[
                            ["g(z)", <>꺼낸 노드 <InlineMath math="z"/>의 cost-to-come. 시작점까지 트리 경로의 길이다</>],
                            ["g(x)", <>다른 임의 open 노드 <InlineMath math="x"/>의 cost-to-come</>],
                        ]}/>
                        <p>
                            을 만족한다. heap이 최소를 돌려주기 때문이다.{" "}
                            <InlineMath math="z"/>로 들어올 미래의 간선은{" "}
                            <InlineMath math="g(y) \ge g(z)"/>인 어떤 open 노드{" "}
                            <InlineMath math="y"/>를 거치므로, 그 후보 비용은
                        </p>
                        <BlockMath math="g(y) + \|y - z\| \;\ge\; g(z) + 0 \;=\; g(z)."/>
                        <Terms items={[
                            ["g(y)", <>뒤늦게 부모가 될 <InlineMath math="y"/>의 cost-to-come. 그 자신도 아직 open이다</>],
                            ["\\|y - z\\|", <>새 간선의 유클리드 길이. <strong>새로 추가된 항</strong>이며 음이 아니다</>],
                            ["g(z)", "꺼낸 노드의 cost-to-come. 이 순간 고정된다"],
                        ]}/>
                        <p>
                            <strong>결론.</strong> 어떤 나중 연결도{" "}
                            <InlineMath math="g(z)"/>를 낮출 수 없으므로 pop 시점에 확정이다.{" "}
                            <InlineMath math="z"/>에 고른 트리 간선은 충돌 없는 간선들 중
                            최적이고, 어떤 rewire 패스도 그것을 개선하지 못한다. Dijkstra의
                            논증이며, FMT*가 각 비용을 한 번에 확정하는 이유가 바로 이것이다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs FMT* and PRM* on the <em>same samples</em> through an S-shaped
                    corridor. Both reach the same cost — but watch the edge counts: FMT*'s lazy
                    march builds one edge per node (a tree), while PRM* wires every feasible
                    neighbor pair into a dense roadmap, an order of magnitude more. Raise the
                    sample count and the gap widens. The replay below is the repository demo on
                    the benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 <em>같은 표본</em> 위에서 FMT*와 PRM*를 S자 통로에 돌린다. 둘 다
                    같은 비용에 닿지만 간선 수를 보라. FMT*의 lazy 행진은 노드마다 간선 하나를
                    세워 트리를 이루는데, PRM*는 실행 가능한 이웃 쌍을 모두 이어 자릿수만큼 더
                    빽빽한 roadmap을 만든다. 표본 수를 올리면 격차가 벌어진다. 아래 replay는
                    벤치마크 맵 위의 저장소 demo다.
                </p>}
            />
            <FmtStarSandbox/>
            <TraceReplay algo="fmt_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's FMT* demo: samples drawn, then a single cost-ordered march that connects each node through one lazily-checked edge",
                "저장소 FMT* demo의 실제 trace. 표본을 뿌린 뒤, cost 순 행진 한 번이 각 노드를 lazy 검사한 간선 하나로 연결한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The radius graph and shrinking-radius formula are shared with the batch
                    planners; FMT* itself is the marching loop with its single lazy collision
                    check. Embedded below in full.
                </p>}
                ko={<p>
                    반경 그래프와 줄어드는 반경 공식은 batch planner들과 공유한다. FMT* 자체는
                    단 한 번의 lazy 충돌 검사를 갖는 행진 루프다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/fmt_star.py",
                            code: fmtStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/fmt_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/fmt_star.cpp",
                            code: fmtStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/fmt_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete FMT* implementation, embedded from the repository sources",
                    "FMT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    L. Janson, E. Schmerling, A. Clark, M. Pavone,{" "}
                    <a href="https://doi.org/10.1177/0278364915577958" target="_blank"
                       rel="noopener noreferrer">
                        <em>Fast Marching Tree: A Fast Marching Sampling-Based Method for
                            Optimal Motion Planning in Many Dimensions</em>
                    </a>,
                    The International Journal of Robotics Research, 2015.
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

export default FmtStar
