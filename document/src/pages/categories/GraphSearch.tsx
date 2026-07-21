import {T, useTr} from "../../libs/i18n";
import Terms from "../../components/math/Terms";
import {BlockMath, InlineMath} from "../../components/math/Tex";
import GridAsGraph from "../../components/panels/intro/GridAsGraph";

const GraphSearch = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Graph search is the older and stricter of the two global-planning families:
                    discretize the world into a graph, then search that graph <em>exactly</em>.
                    Twelve of this site's planners are variations on that one move, and they differ
                    mainly in what they are willing to trade for speed.
                </p>}
                ko={<p>
                    Graph search는 global planning의 두 계열 중 더 오래되고 더 엄격한 쪽이다.
                    세계를 그래프로 이산화한 뒤, 그 그래프를 <em>정확하게</em> 탐색한다. 이
                    사이트의 planner 열두 개가 이 한 가지 아이디어의 변주이고, 차이는 주로 속도를 위해
                    무엇을 내주는가에 있다.
                </p>}
            />

            <h2>{t("What Graph Search Is", "Graph Search 란")}</h2>
            <T
                en={<p>
                    An occupancy grid becomes a graph the moment you decide two things: which cells
                    are <em>vertices</em> (the free ones) and which pairs are <em>edges</em>{" "}
                    (4- or 8-connected neighbors, with costs 1 and <InlineMath math="\sqrt{2}"/>).
                    After that decision, "find a path for the robot" becomes "find a least-cost path
                    in a graph" — a problem with sixty years of theory behind it. The price was paid
                    up front: the answer is only as good as the discretization, and the graph's size
                    grows exponentially with the dimension of the state.
                </p>}
                ko={<p>
                    occupancy grid는 두 가지를 정하는 순간 그래프가 된다. 어떤 셀이{" "}
                    <em>vertex</em> 인가(자유 셀), 어떤 쌍이 <em>edge</em> 인가(4- 또는
                    8-connected 이웃, 비용 1과 <InlineMath math="\sqrt{2}"/>). 이 결정 뒤에는
                    "로봇의 경로를 찾아라"가 "그래프에서 최소 비용 경로를 찾아라"가 되고, 이
                    문제에는 60년치 이론이 쌓여 있다. 대신 치러야 할 대가가 있다. 답의 품질은 이산화 해상도가 결정하고,
                    그래프 크기는 상태 차원에 따라 지수적으로 커진다.
                </p>}
            />

            <GridAsGraph/>

            <h2>{t("The Search Model", "탐색 모델")}</h2>
            <T
                en={<>
                    <p>
                        Every algorithm in this category runs on the same four-method interface (the
                        repository's <code>DiscreteSpace</code> capability): enumerate{" "}
                        <code>neighbors(state)</code> with edge costs, and optionally
                        estimate <code>heuristic(a, b)</code>. The planners are all instances of one
                        skeleton — keep a frontier of discovered states, repeatedly take the "best"
                        one, and relax its outgoing edges:
                    </p>
                    <BlockMath math="f(n) = g(n) + w \cdot h(n)"/>
                    <Terms items={[
                        ["f(n)", <>priority of node <InlineMath math="n"/> in the frontier — smallest pops first</>],
                        ["g(n)", <>actual cost of the cheapest path found so far from the start to <InlineMath math="n"/></>],
                        ["h(n)", <>heuristic estimate of the remaining cost from <InlineMath math="n"/> to the goal</>],
                        ["w", <>heuristic weight: <InlineMath math="w = 0"/> is Dijkstra, <InlineMath math="w = 1"/> is A*, <InlineMath math="w > 1"/> is greedier weighted A*</>],
                    ]}/>
                    <p>
                        Choosing the priority <InlineMath math="f"/> chooses the algorithm: BFS uses
                        hop count, Dijkstra uses <InlineMath math="g"/> alone, A* adds an admissible{" "}
                        <InlineMath math="h"/>, weighted A* inflates it. Nearly everything else in
                        the list is this skeleton plus one extra idea.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 카테고리의 모든 알고리즘은 같은 4-메서드 인터페이스(저장소의{" "}
                        <code>DiscreteSpace</code> capability) 위에서 돈다. 간선 비용과 함께{" "}
                        <code>neighbors(state)</code>를 열거하고, 선택적으로{" "}
                        <code>heuristic(a, b)</code>를 추정한다. planner들은 모두 같은 뼈대 위의
                        변형이다. 발견한 상태의 frontier를 유지하고, "최선"을 반복해서 꺼내고,
                        나가는 간선을 relax 한다:
                    </p>
                    <BlockMath math="f(n) = g(n) + w \cdot h(n)"/>
                    <Terms items={[
                        ["f(n)", <>frontier에서 노드 <InlineMath math="n"/>의 우선순위. 가장 작은 것부터 꺼낸다</>],
                        ["g(n)", <>지금까지 찾은 시작→<InlineMath math="n"/> 최소 실비용</>],
                        ["h(n)", <><InlineMath math="n"/>에서 목표까지 남은 비용의 heuristic 추정치</>],
                        ["w", <>heuristic 가중치. <InlineMath math="w = 0"/>이면 Dijkstra, <InlineMath math="w = 1"/>이면 A*, <InlineMath math="w > 1"/>이면 더 탐욕적인 weighted A*</>],
                    ]}/>
                    <p>
                        우선순위 <InlineMath math="f"/>를 고르는 것이 곧 알고리즘을 고르는 것이다.
                        BFS는 hop 수, Dijkstra는 <InlineMath math="g"/>만, A*는 admissible{" "}
                        <InlineMath math="h"/>를 더하고, weighted A*는 그것을 부풀린다. 목록의
                        나머지 대부분은 이 뼈대에 아이디어 하나를 더한 것이다.
                    </p>
                </>}
            />

            <h2>{t("Guarantees", "보장")}</h2>
            <T
                en={<ul>
                    <li><strong>Completeness</strong>: on a finite graph, if a path exists it is
                        found. No probability involved.</li>
                    <li><strong>Optimality</strong>: Dijkstra and A* (admissible{" "}
                        <InlineMath math="h"/>) return least-cost paths <em>on the graph</em>;
                        any-angle variants close the gap between graph-optimal and truly shortest.</li>
                    <li><strong>Cost</strong>: worst case exponential in state dimension; in
                        practice, memory for the frontier and closed set is the binding limit — the
                        motivation for the anytime and incremental variants.</li>
                </ul>}
                ko={<ul>
                    <li><strong>완전성</strong>: 유한 그래프에서 경로가 존재하면 반드시 찾는다.
                        확률이 개입하지 않는다.</li>
                    <li><strong>최적성</strong>: Dijkstra와 A*(admissible <InlineMath math="h"/>)는{" "}
                        <em>그래프 위에서</em> 최소 비용 경로를 반환한다. any-angle 변형이 그래프
                        최적과 진짜 최단의 간극을 좁힌다.</li>
                    <li><strong>비용</strong>: 최악의 경우 상태 차원에 지수적이다. 실전에서는
                        frontier와 closed 집합의 메모리가 먼저 한계가 되고, 이것이 anytime·
                        incremental 변형의 동기다.</li>
                </ul>}
            />

            <h2>{t("A Map of the Algorithms", "알고리즘 지도")}</h2>
            <T
                en={<ul>
                    <li><strong>Foundations</strong> — BFS (uniform hops), Dijkstra (weighted costs),
                        A* (heuristic guidance). Everything else assumes these.</li>
                    <li><strong>Anytime &amp; incremental</strong> — ARA* (start greedy, tighten the
                        bound), D* Lite (repair the plan when the map changes), AD* (both at once).</li>
                    <li><strong>Any-angle</strong> — Theta* and Lazy Theta* (shortcut parents during
                        search), Visibility A* (plan on the visibility graph), Anya (optimal
                        any-angle over interval projections).</li>
                    <li><strong>Structure exploitation</strong> — Jump Point Search prunes the grid's
                        symmetric paths and expands orders of magnitude fewer nodes.</li>
                    <li><strong>Kinematics-aware</strong> — Hybrid A* searches over continuous
                        headings with motion primitives, producing paths a car can actually follow.</li>
                </ul>}
                ko={<ul>
                    <li><strong>기초</strong>: BFS(균일 hop), Dijkstra(균일하지 않은 비용),
                        A*(heuristic 유도). 나머지 전부가 이 셋을 전제한다.</li>
                    <li><strong>Anytime &amp; incremental</strong>: ARA*(탐욕적으로 시작해 한계를
                        조이고), D* Lite(지도가 바뀌면 계획을 수선하고), AD*(둘을 동시에).</li>
                    <li><strong>Any-angle</strong>: Theta*와 Lazy Theta*(탐색 중 부모를 지름길로
                        잇고), Visibility A*(visibility graph 위에서 계획), Anya(interval 투영
                        위의 최적 any-angle).</li>
                    <li><strong>구조 활용</strong>: Jump Point Search는 격자의 대칭 경로를 쳐내
                        수십 배 적은 노드로 같은 답을 얻는다.</li>
                    <li><strong>기구학 인지</strong>: Hybrid A*는 motion primitive로 연속 heading
                        을 탐색해, 차량이 실제로 따라갈 수 있는 경로를 만든다.</li>
                </ul>}
            />
        </>
    )
}

export default GraphSearch
