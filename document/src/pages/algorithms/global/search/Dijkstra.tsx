import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import DijkstraSandbox from "../../../../components/panels/global/dijkstra/DijkstraSandbox";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import CodeTabs from "../../../../components/CodeTabs";
import Pseudocode from "../../../../components/Pseudocode";
import dijkstraPy from "../../../../../../python/navigation/global_planning/search/dijkstra.py?raw";
import bestFirstPy from "../../../../../../python/navigation/global_planning/search/_bestfirst.py?raw";
import dijkstraCpp from "../../../../../../cpp/src/global_planning/search/dijkstra.cpp?raw";
import discreteSearchHpp from "../../../../../../cpp/include/navigation/global_planning/search/discrete_search.hpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Dijkstra = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Dijkstra's algorithm is BFS grown up. Published in 1959 as "a note on two
                    problems in connexion with graphs", it answers the question BFS cannot: what is
                    the <em>cheapest</em> path when edges have different costs? Sixty-five years
                    later it is still the backbone of routing — and, in this section, the direct
                    parent of A*.
                </p>}
                ko={<p>
                    Dijkstra 알고리즘은 BFS에 비용 개념을 얹은 확장이다. 1959년 "그래프에 관한 두
                    문제에 대한 소고"로 발표되어, BFS가 답하지 못하는 질문에 답한다. edge 비용이
                    제각각일 때 <em>가장 싼</em> 경로는 무엇인가? 65년이 지난 지금도 라우팅의
                    근간이고, 이 섹션에서는 A*로 바로 이어지는 전 단계다.
                </p>}
            />

            <h2>{t("From BFS to Dijkstra", "BFS에서 Dijkstra로")}</h2>
            <T
                en={<>
                    <p>
                        BFS expands nodes in order of <em>discovery time</em>, which coincides with
                        hop count. Dijkstra makes one substitution: expand in order of{" "}
                        <em>cost from the start</em>,
                    </p>
                    <BlockMath math="g(n) = \min_{\text{paths } s \to n} \sum_{\text{edges}} c(e),"/>
                    <Terms items={[
                        ["g(n)", <>cost of the cheapest start→<InlineMath math="n"/> path found so far; on settle it equals the true minimum</>],
                        ["s", "the start node"],
                        ["c(e)", <><strong>the new ingredient</strong>: each edge <InlineMath math="e"/> now carries its own non-negative cost, replacing BFS's implicit "every hop costs 1"</>],
                    ]}/>
                    <p>
                        maintained with a priority queue instead of a FIFO. Where BFS fixes a node's
                        parent at first discovery, Dijkstra keeps <em>relaxing</em>: whenever a
                        cheaper way to a discovered node appears, its <InlineMath math="g"/> and
                        parent are updated. A node's answer becomes final only when it is popped —
                        <em>settled</em> — because nothing cheaper can reach it afterwards.
                    </p>
                </>}
                ko={<>
                    <p>
                        BFS는 노드를 <em>발견 시각</em> 순으로 확장하고, 그것은 hop 수와
                        일치한다. Dijkstra는 한 가지만 바꾼다. <em>시작점부터의 비용</em>
                    </p>
                    <BlockMath math="g(n) = \min_{\text{paths } s \to n} \sum_{\text{edges}} c(e)"/>
                    <Terms items={[
                        ["g(n)", <>지금까지 찾은 시작→<InlineMath math="n"/> 최소 비용. settle 되는 순간 참 최솟값과 같아진다</>],
                        ["s", "시작 노드"],
                        ["c(e)", <><strong>새로 추가된 재료</strong>: 간선 <InlineMath math="e"/>마다 자기 비용(음수 아님)이 붙는다. BFS의 암묵적 "모든 hop = 1"을 대체한다</>],
                    ]}/>
                    <p>
                        순으로 확장하고, 그것을 FIFO 대신 priority queue로 유지한다. BFS가 최초
                        발견에서 부모를 고정하는 것과 달리 Dijkstra는 계속 <em>relax</em> 한다.
                        발견된 노드로 가는 더 싼 길이 나타날 때마다 <InlineMath math="g"/>와
                        부모를 갱신한다. 노드의 답은 꺼내지는 순간, 즉 <em>settle</em> 되는
                        순간에야 확정된다. 그 뒤로는 더 싼 길이 도달할 수 없기 때문이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Complete</strong> on finite graphs; <strong>optimal</strong> for
                        non-negative edge costs.</li>
                    <li><strong>Time</strong> <InlineMath math="O((V + E)\log V)"/> with a binary
                        heap (<InlineMath math="O(E + V\log V)"/> with a Fibonacci heap, mostly of
                        theoretical interest); <strong>memory</strong> <InlineMath math="O(V)"/>.</li>
                    <li>Expands nodes in non-decreasing <InlineMath math="g"/> — the frontier is a
                        cost ripple, which is precisely the blindness A* fixes with a heuristic.</li>
                    <li>One-to-all by nature: run to exhaustion and you get shortest paths to{" "}
                        <em>every</em> node, which is how costmap potentials are built.</li>
                </ul>}
                ko={<ul>
                    <li>유한 그래프에서 <strong>완전</strong>하고, 음수 아닌 edge 비용에서{" "}
                        <strong>최적</strong>이다.</li>
                    <li><strong>시간</strong>은 binary heap 으로{" "}
                        <InlineMath math="O((V + E)\log V)"/> (Fibonacci heap 으로는{" "}
                        <InlineMath math="O(E + V\log V)"/> 이지만 대체로 이론적 관심사),{" "}
                        <strong>메모리</strong>는 <InlineMath math="O(V)"/>.</li>
                    <li>노드를 <InlineMath math="g"/> 비감소 순으로 확장한다. frontier가 비용
                        등고선처럼 사방으로 퍼진다는 뜻이고, A*가 heuristic으로 고치는 맹목
                        탐색이 정확히 이것이다.</li>
                    <li>본질적으로 one-to-all이다. 끝까지 돌리면 <em>모든</em> 노드로의 최단
                        경로가 나오고, costmap potential이 그렇게 만들어진다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The state is a min-heap <InlineMath math="Q"/> keyed by{" "}
                    <InlineMath math="g"/>, plus per-node <InlineMath math="g"/> values and
                    parents. Each iteration does three things: pop the cheapest node and settle it
                    (its answer is now final), check the goal, and <em>relax</em> every outgoing
                    edge — if a cheaper route to a neighbor appears, record it and push the
                    neighbor:
                </p>}
                ko={<p>
                    상태는 <InlineMath math="g"/>를 키로 하는 min-heap <InlineMath math="Q"/>와,
                    노드별 <InlineMath math="g"/> 값·부모다. 매 반복은 세 가지를 한다. 가장 싼
                    노드를 꺼내 settle 하고(이제 그 답은 확정이다), 목표인지 확인하고, 나가는
                    간선을 전부 <em>relax</em> 한다. 이웃으로 가는 더 싼 길이 나타나면 기록하고 그
                    이웃을 push 한다:
                </p>}
            />
            <Pseudocode code={`Q ← min-heap keyed by g;  g[start] ← 0;  push (0, start)   # 1
while Q is not empty:
    n ← pop_min(Q)                                           # 2
    if n is settled: continue                                # 3
    settle n
    if n = goal:                                             # 4
        return reconstruct(parent, goal)
    for each neighbor n' with edge cost c(n, n'):
        if g[n] + c(n, n') < g[n']:                          # 5
            g[n'] ← g[n] + c(n, n');  parent[n'] ← n
            push (g[n'], n') into Q
return failure`}/>
            <T
                en={<ol>
                    <li>Put the start in the heap with cost 0.</li>
                    <li>Pop the cheapest frontier node — by the theorem below, its{" "}
                        <InlineMath math="g"/> is now the true shortest distance.</li>
                    <li>Skip stale entries: the same node may sit in the heap several times with
                        outdated costs (the <em>lazy queue</em> idiom — no decrease-key needed).</li>
                    <li>Check the goal at pop time; popping it settles it, so the path is
                        optimal.</li>
                    <li>Relax every outgoing edge: if going through <InlineMath math="n"/> is
                        cheaper than the best known route to <InlineMath math="n'"/>, record the
                        improvement and push <InlineMath math="n'"/> (again).</li>
                </ol>}
                ko={<ol>
                    <li>시작 노드를 비용 0으로 heap에 넣는다.</li>
                    <li>frontier에서 가장 싼 노드를 꺼낸다. 아래 정리에 의해 이 시점의{" "}
                        <InlineMath math="g"/>가 실제 최단 거리다.</li>
                    <li>stale 항목은 건너뛴다. 같은 노드가 갱신 전 비용으로 heap에 여러 번
                        들어 있을 수 있다 (<em>lazy queue</em> 관용구, decrease-key 불필요).</li>
                    <li>goal 검사를 pop 시점에 한다. 꺼내지는 순간 settle 되므로 경로는 최적이다.</li>
                    <li>나가는 간선을 전부 relax 한다. <InlineMath math="n"/>을 거치는 길이{" "}
                        <InlineMath math="n'"/>의 지금까지 최선보다 싸면, 개선을 기록하고{" "}
                        <InlineMath math="n'"/>을 (다시) push 한다.</li>
                </ol>}
            />
            <T
                en={<p>
                    The push in the relaxation step is the <em>lazy queue</em> idiom this
                    repository uses: instead of a decrease-key structure, stale entries are simply
                    skipped when popped (the settled check). It costs a few extra queue entries and
                    buys a much simpler implementation.
                </p>}
                ko={<p>
                    relaxation 단계의 push가 이 저장소가 쓰는 <em>lazy queue</em> 관용구다.
                    decrease-key 자료구조 대신, 낡은 항목은 꺼낼 때 건너뛴다 (settle 검사). 큐
                    항목 몇 개를 더 쓰는 대신 구현이 훨씬 단순해진다.
                </p>}
            />

            <h2>{t("Why Greedy Works", "Greedy가 왜 통하는가")}</h2>
            <T
                en={<p>
                    Settling the cheapest frontier node looks greedy, and greedy usually breaks.
                    Here it doesn't, for one reason: <strong>edge costs are non-negative</strong>.
                    Any path that would improve on the node being settled must leave through the
                    frontier — and every frontier exit is already at least as expensive. The formal
                    version is folded below.
                </p>}
                ko={<p>
                    가장 싼 frontier 노드를 settle 하는 것은 greedy 전략이고, greedy는 대개
                    최적성을 보장하지 못한다. 그런데 여기서는 통한다. 이유는 하나,{" "}
                    <strong>edge 비용이 음수가 아니기 때문</strong>이다. settle 되는 노드보다 나은
                    경로가 있다면 frontier를 거쳐 나가야 하는데, frontier의 모든 출구가 이미
                    그만큼 비싸다. 형식적 서술은 아래에 접어 두었다.
                </p>}
            />
            <Proof title={t("Theorem (settled means final)", "정리 (settle = 확정)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Edge costs <InlineMath math="\ge 0"/>; induction
                            on pop order, so every settled <InlineMath math="u"/> already has{" "}
                            <InlineMath math="g(u) = \delta(s, u)"/>. Suppose <InlineMath math="n"/>{" "}
                            pops with <InlineMath math="g(n) > \delta(s, n)"/>.
                        </p>
                        <p>
                            Take a shortest path <InlineMath math="\sigma"/> to{" "}
                            <InlineMath math="n"/>; let <InlineMath math="(u, v)"/> be its first
                            edge leaving the settled set. Then:
                        </p>
                        <BlockMath math="g(v) \;\overset{\text{relaxed}}{\le}\; \delta(s, u) + c(u, v) \;\overset{c\,\ge\,0}{\le}\; \delta(s, n) \;<\; g(n)"/>
                        <Terms items={[
                            ["\\delta(s, x)", <>true shortest-path cost from the start to <InlineMath math="x"/></>],
                            ["(u, v)", <>the first edge of the shortest path <InlineMath math="\\sigma"/> that leaves the settled set: <InlineMath math="u"/> settled, <InlineMath math="v"/> not yet</>],
                            ["g(\\cdot)", "cheapest cost found so far; relaxing (u, v) already offered v the value δ(s,u) + c(u,v)"],
                        ]}/>
                        <p>
                            <InlineMath math="\Rightarrow v"/> pops before <InlineMath math="n"/> —
                            contradiction. One negative edge breaks the middle step; that failure
                            mode is Bellman–Ford's reason to exist. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 간선 비용 <InlineMath math="\ge 0"/>. 꺼낸 순서에
                            대한 귀납으로, settle 된 모든 <InlineMath math="u"/>는 이미{" "}
                            <InlineMath math="g(u) = \delta(s, u)"/>다. 귀류법으로{" "}
                            <InlineMath math="n"/>이 <InlineMath math="g(n) > \delta(s, n)"/>으로
                            꺼내졌다고 하자.
                        </p>
                        <p>
                            <InlineMath math="n"/>으로 가는 최단 경로 <InlineMath math="\sigma"/>
                            에서, settle 집합을 처음 벗어나는 간선을 <InlineMath math="(u, v)"/>라
                            하면:
                        </p>
                        <BlockMath math="g(v) \;\overset{\text{relaxed}}{\le}\; \delta(s, u) + c(u, v) \;\overset{c\,\ge\,0}{\le}\; \delta(s, n) \;<\; g(n)"/>
                        <Terms items={[
                            ["\\delta(s, x)", <>시작→<InlineMath math="x"/>의 참 최단 비용</>],
                            ["(u, v)", <>최단 경로 <InlineMath math="\\sigma"/>가 settle 집합을 처음 벗어나는 간선. <InlineMath math="u"/>는 settle 됨, <InlineMath math="v"/>는 아직</>],
                            ["g(\\cdot)", "지금까지 찾은 최소 비용. (u, v)를 relax 할 때 v는 이미 δ(s,u) + c(u,v)를 제안받았다"],
                        ]}/>
                        <p>
                            <InlineMath math="\Rightarrow v"/>가 <InlineMath math="n"/>보다 먼저
                            꺼내진다. 모순. 음수 간선 하나가 가운데 부등호를 깨는데, 그 실패
                            모드가 Bellman–Ford의 존재 이유다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs BFS and Dijkstra on the same problem, 8-connected so that
                    diagonal steps cost <InlineMath math="\sqrt{2}"/>. Toggle the queue type and
                    watch two things: the frontier changes from hop-rings to a cost-ripple, and the
                    reported path cost drops — the fewest-edge path is not the cheapest one.
                </p>}
                ko={<p>
                    sandbox는 같은 문제를 BFS와 Dijkstra로 푼다. 대각 스텝이{" "}
                    <InlineMath math="\sqrt{2}"/> 비용을 갖도록 8-connected다. 큐 종류를 바꿔 가며
                    두 가지를 보라. frontier가 hop 동심원에서 비용 등고선으로 바뀌는 것, 그리고
                    보고되는 path cost가 내려가는 것. 최소 edge 경로는 최소 비용 경로가 아니다.
                </p>}
            />
            <DijkstraSandbox/>
            <TraceReplay algo="dijkstra" maps={["maze01", "open01"]} label={t(
                "Real traces from the repository's Dijkstra demo on the benchmark maps",
                "저장소 Dijkstra demo가 벤치마크 맵에서 방출한 실제 trace",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    In this repository Dijkstra is the degenerate case of the shared best-first
                    core: the same loop as A* with the heuristic switched off
                    (<InlineMath math="f = g"/>). The subclass below is nearly empty on purpose.
                </p>}
                ko={<p>
                    이 저장소에서 Dijkstra는 공유 best-first 코어의 퇴화 사례다. A*와 같은
                    루프에서 heuristic만 끈 것이다 (<InlineMath math="f = g"/>). 아래 subclass가
                    거의 비어 있는 것은 의도된 것이다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/global_planning/search/dijkstra.py",
                                code: dijkstraPy,
                                href: `${REPO}/python/navigation/global_planning/search/dijkstra.py`,
                            },
                            {
                                name: "python/navigation/global_planning/search/_bestfirst.py",
                                code: bestFirstPy,
                                href: `${REPO}/python/navigation/global_planning/search/_bestfirst.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/src/global_planning/search/dijkstra.cpp",
                                code: dijkstraCpp,
                                href: `${REPO}/cpp/src/global_planning/search/dijkstra.cpp`,
                            },
                            {
                                name: "cpp/include/navigation/global_planning/search/discrete_search.hpp",
                                code: discreteSearchHpp,
                                href: `${REPO}/cpp/include/navigation/global_planning/search/discrete_search.hpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The near-empty Dijkstra subclass and the shared best-first core it rides on",
                    "거의 비어 있는 Dijkstra subclass와 그것이 올라타는 공유 best-first 코어",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    E. W. Dijkstra,{" "}
                    <a href="https://doi.org/10.1007/BF01386390" target="_blank"
                       rel="noopener noreferrer">
                        <em>A Note on Two Problems in Connexion with Graphs</em>
                    </a>,
                    Numerische Mathematik, 1959.
                </li>
                <li>
                    M. L. Fredman, R. E. Tarjan,{" "}
                    <a href="https://doi.org/10.1145/28869.28874" target="_blank"
                       rel="noopener noreferrer">
                        <em>Fibonacci Heaps and Their Uses in Improved Network Optimization
                            Algorithms</em>
                    </a>,
                    Journal of the ACM, 1987.
                </li>
            </ol>
        </>
    )
}

export default Dijkstra
