import {T, useTr} from "../../../../libs/i18n";
import {InlineMath} from "../../../../components/math/Tex";
import BfsSandbox from "../../../../components/panels/global/bfs/BfsSandbox";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import CodeTabs from "../../../../components/CodeTabs";
import Pseudocode from "../../../../components/Pseudocode";
import bfsPy from "../../../../../../python/navigation/global_planning/search/bfs.py?raw";
import bfsCpp from "../../../../../../cpp/src/global_planning/search/bfs.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const Bfs = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Breadth-first search is where graph search begins. It has no priority queue, no
                    costs, no heuristic — just a FIFO queue and one invariant — and that is exactly
                    why it is worth studying first: every planner in this section is BFS with one
                    of those missing pieces added.
                </p>}
                ko={<p>
                    Breadth-first search 는 graph search 의 출발점이다. priority queue 도, 비용도,
                    heuristic 도 없이 FIFO 큐와 불변식 하나만 갖고 있다. 바로 그래서 처음 공부할
                    가치가 있다. 이 섹션의 모든 planner 는 BFS 에 그 빠진 조각 중 하나를 더한
                    것이다.
                </p>}
            />

            <h2>{t("The Idea", "아이디어")}</h2>
            <T
                en={<>
                    <p>
                        Explore the graph in <em>waves</em>. First visit everything one edge away
                        from the start, then everything two edges away, and so on. The FIFO queue
                        enforces this by construction: nodes discovered earlier are expanded
                        earlier, so the frontier is always a ring of (almost) equal hop count.
                    </p>
                    <p>
                        The invariant that makes it correct: <strong>when a node is first
                        discovered, the path that discovered it is a fewest-edge path</strong>. So
                        each node's parent is fixed once, at discovery — there is no relaxation, no
                        "found a better way later". This is also why BFS is so cheap: one queue
                        operation and one set lookup per edge.
                    </p>
                </>}
                ko={<>
                    <p>
                        그래프를 <em>파도</em>처럼 훑는다. 시작점에서 edge 하나 거리인 곳을 전부
                        방문하고, 다음은 둘 거리, 그 다음은 셋. FIFO 큐가 이것을 구조적으로
                        강제한다. 먼저 발견된 노드가 먼저 확장되므로, frontier 는 항상 (거의) 같은
                        hop 수의 고리다.
                    </p>
                    <p>
                        정확성을 만드는 불변식은 이것이다. <strong>노드가 처음 발견되는 순간, 그
                        발견 경로가 곧 최소 edge 경로다</strong>. 그래서 부모는 발견 시점에 한 번
                        고정되고, relaxation 도 "나중에 더 나은 길 발견"도 없다. BFS 가 그토록 싼
                        이유이기도 하다. edge 하나당 큐 연산 한 번, 집합 조회 한 번이 전부다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Complete</strong> on finite graphs.</li>
                    <li><strong>Optimal in hop count</strong>; optimal in cost only when all edges
                        cost the same.</li>
                    <li><strong>Time</strong> <InlineMath math="O(V + E)"/>,{" "}
                        <strong>memory</strong> <InlineMath math="O(V)"/> — no priority queue, so
                        both bounds are as tight as graph traversal gets.</li>
                    <li>Each node is enqueued at most once; there is no re-expansion of any kind.</li>
                </ul>}
                ko={<ul>
                    <li>유한 그래프에서 <strong>완전</strong>하다.</li>
                    <li><strong>hop 수 기준 최적</strong>이다. 비용 기준 최적은 모든 edge 비용이
                        같을 때만 성립한다.</li>
                    <li><strong>시간</strong> <InlineMath math="O(V + E)"/>,{" "}
                        <strong>메모리</strong> <InlineMath math="O(V)"/>. priority queue 가 없어 두
                        한계 모두 그래프 순회의 하한에 붙어 있다.</li>
                    <li>노드는 최대 한 번 큐에 들어가고, 어떤 종류의 재확장도 없다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Two data structures carry everything: a FIFO <em>queue</em> holding the
                    frontier, and a <em>discovered</em> set that guarantees each node enters the
                    queue at most once. The loop repeats three moves — pop the front, check the
                    goal, and enqueue any neighbor seen for the first time (fixing its parent at
                    that moment):
                </p>}
                ko={<p>
                    자료구조 두 개가 전부다. frontier 를 담는 FIFO <em>queue</em> 와, 각 노드가
                    큐에 최대 한 번만 들어가게 보장하는 <em>discovered</em> 집합. 루프는 세 동작의
                    반복이다. 맨 앞을 꺼내고, 목표인지 확인하고, 처음 보는 이웃을 큐에 넣는다
                    (부모는 그 순간 고정된다):
                </p>}
            />
            <Pseudocode code={`queue ← FIFO [start];  discovered ← {start}
while queue is not empty:
    n ← pop_front(queue)
    if n = goal:
        return reconstruct(parent, goal)
    for each neighbor n' of n:
        if n' not in discovered:
            discovered ← discovered ∪ {n'}    # parent fixed at first discovery
            parent[n'] ← n
            push_back(queue, n')
return failure`}/>
            <T
                en={<p>
                    Note what is <em>not</em> here: edge costs never influence the order. The queue
                    position depends only on when a node was discovered — its hop depth.
                </p>}
                ko={<p>
                    여기 <em>없는</em> 것에 주목하라. edge 비용은 순서에 전혀 개입하지 않는다. 큐
                    위치는 노드가 언제 발견됐는가, 즉 hop 깊이만으로 정해진다.
                </p>}
            />

            <h2>{t("When Hops Lie", "최소 hop 이 최단이 아닐 때")}</h2>
            <T
                en={<>
                    <p>
                        BFS returns the path with the fewest edges. On a 4-connected grid where
                        every step costs 1, fewest edges <em>is</em> cheapest, and BFS is exactly as
                        optimal as Dijkstra. The moment edges stop being equal — diagonal steps
                        costing <InlineMath math="\sqrt{2}"/>, terrain weights, turn penalties —
                        "fewest" and "cheapest" part ways, and BFS confidently returns the wrong
                        answer.
                    </p>
                    <p>
                        The map below was built to show this: BFS finds a path with fewer edges
                        through the diagonal shortcut, but its true cost is higher than the path
                        Dijkstra takes. Play both and compare the reported path costs.
                    </p>
                </>}
                ko={<>
                    <p>
                        BFS 는 edge 수가 가장 적은 경로를 반환한다. 모든 스텝 비용이 1 인
                        4-connected grid 에서는 최소 edge 가 곧 최소 비용이라, BFS 는 Dijkstra 와
                        똑같이 최적이다. edge 가 균등하기를 멈추는 순간, 즉 대각 스텝이{" "}
                        <InlineMath math="\sqrt{2}"/> 가 되고 지형 가중치나 회전 페널티가 붙는
                        순간, "최소 edge"와 "최소 비용"은 갈라지고, BFS 는 그 사실을 알지
                        못한 채 틀린 답을 내놓는다.
                    </p>
                    <p>
                        아래 맵은 그것을 보여 주려고 만든 것이다. BFS 는 대각 지름길로 edge 수가
                        더 적은 경로를 찾지만, 실제 비용은 Dijkstra 의 경로보다 높다. 둘 다
                        재생해서 보고된 path cost 를 비교해 보라.
                    </p>
                </>}
            />
            <TraceReplay algo="bfs" maps={["bfs_hopcost01"]} label={t(
                "BFS on the counterexample map: fewest edges, not lowest cost",
                "counterexample 맵 위의 BFS. edge 수는 최소지만 비용은 최소가 아니다",
            )}/>
            <TraceReplay algo="dijkstra" maps={["bfs_hopcost01"]} label={t(
                "Dijkstra on the same map returns the genuinely cheapest path",
                "같은 맵의 Dijkstra 는 진짜 최소 비용 경로를 반환한다",
            )}/>

            <h2>Demo</h2>
            <T
                en={<p>
                    Watch the frontier: it grows as concentric rings around the start, utterly
                    indifferent to where the goal is. Compare the ring shape against Dijkstra's
                    cost-ripple and A*'s directed wedge on their pages — same skeleton, different
                    queue.
                </p>}
                ko={<p>
                    frontier 를 보라. 목표가 어디 있든 아랑곳없이 시작점 둘레의 동심원으로
                    자란다. 이 고리 모양을 Dijkstra 페이지의 비용 등고선, A* 페이지의 방향성 있는
                    쐐기와 비교해 보라. 뼈대는 같고 큐만 다르다.
                </p>}
            />
            <BfsSandbox/>
            <TraceReplay algo="bfs" maps={["maze01", "open01"]} label={t(
                "Real traces from the repository's BFS demo on the benchmark maps",
                "저장소 BFS demo 가 벤치마크 맵에서 방출한 실제 trace",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    BFS is the one planner in this section that does not use the shared best-first
                    core — a FIFO queue needs no priority ordering. The sources below are embedded
                    directly from the repository.
                </p>}
                ko={<p>
                    BFS 는 이 섹션에서 유일하게 공유 best-first 코어를 쓰지 않는 planner 다. FIFO
                    큐에는 우선순위 정렬이 필요 없기 때문이다. 아래 코드는 저장소 소스를 그대로
                    embed 한 것이다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/bfs.py",
                            code: bfsPy,
                            href: `${REPO}/python/navigation/global_planning/search/bfs.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/bfs.cpp",
                            code: bfsCpp,
                            href: `${REPO}/cpp/src/global_planning/search/bfs.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete BFS implementation, embedded from the repository sources",
                    "BFS 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    E. F. Moore, <em>The Shortest Path Through a Maze</em>, Proceedings of the
                    International Symposium on the Theory of Switching, 1959.
                </li>
                <li>
                    C. Y. Lee,{" "}
                    <a href="https://doi.org/10.1109/TEC.1961.5219222" target="_blank"
                       rel="noopener noreferrer">
                        <em>An Algorithm for Path Connections and Its Applications</em>
                    </a>,
                    IRE Transactions on Electronic Computers, 1961.
                </li>
                <li>
                    T. H. Cormen, C. E. Leiserson, R. L. Rivest, C. Stein,{" "}
                    <a href="https://mitpress.mit.edu/9780262046305/introduction-to-algorithms/"
                       target="_blank" rel="noopener noreferrer">
                        <em>Introduction to Algorithms</em>
                    </a>, MIT Press — chapter on elementary graph algorithms.
                </li>
            </ol>
        </>
    )
}

export default Bfs
