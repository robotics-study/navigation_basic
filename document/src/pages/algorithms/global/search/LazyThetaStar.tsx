import {T, useTr} from "../../../../libs/i18n";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import LazyThetaSandbox from "../../../../components/panels/global/lazy_theta_star/LazyThetaSandbox";
import lazyPy from "../../../../../../python/navigation/global_planning/search/lazy_theta_star.py?raw";
import lazyCpp from "../../../../../../cpp/src/global_planning/search/lazy_theta_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const LazyThetaStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Profile Theta* on a big map and the surprise is that the priority queue is not
                    the bottleneck — the line-of-sight checks are. Theta* fires one per generated
                    edge, and most of them are wasted on nodes that never end up mattering. Lazy
                    Theta* (Nash &amp; Koenig, 2010) keeps the paths and deletes most of the
                    checks with one move: assume first, verify later.
                </p>}
                ko={<p>
                    큰 맵에서 Theta*를 프로파일하면 의외의 결과가 나온다. 병목은 priority queue
                    가 아니라 line-of-sight 검사다. Theta*는 생성되는 edge 마다 한 번씩 쏘는데,
                    그 대부분이 끝내 중요해지지 않는 노드에 낭비된다. Lazy Theta*(Nash &amp;
                    Koenig, 2010)는 한 수로 경로는 유지하고 검사 대부분을 없앤다. 일단 가정하고,
                    나중에 확인하는 것이다.
                </p>}
            />

            <h2>{t("Assume First, Verify at Expansion", "일단 가정하고, 확장할 때 확인한다")}</h2>
            <T
                en={<>
                    <p>
                        When Theta* generates a neighbor it must decide immediately between path 1
                        and path 2, so it must check line of sight immediately. Lazy Theta*
                        refuses to decide: it <em>always</em> records the optimistic path 2 —
                        parent set to the grandparent, cost by straight line, no check at all.
                    </p>
                    <p>
                        The debt comes due only when a vertex is popped for expansion. At that
                        moment (<code>set_vertex</code>) one line-of-sight query verifies the
                        assumed parent. If the view is actually blocked, the vertex is repaired:
                        its parent becomes the cheapest already-expanded grid neighbor — a fallback
                        that always exists, because whoever generated the vertex is an adjacent,
                        expanded cell. Nodes that are generated but never expanded — the majority,
                        in a heuristic search — never pay for a check at all.
                    </p>
                </>}
                ko={<>
                    <p>
                        Theta*는 이웃을 생성하는 순간 path 1과 path 2 중 하나를 정해야 하므로,
                        line of sight 도 그 순간 검사해야 한다. Lazy Theta*는 결정을 거부한다.{" "}
                        <em>항상</em> 낙관적인 path 2를 기록한다. 부모는 조부모로, 비용은
                        직선으로, 검사는 아예 없이.
                    </p>
                    <p>
                        빚은 그 vertex 가 확장을 위해 꺼내질 때에만 청산된다. 그 시점
                        (<code>set_vertex</code>)에 line-of-sight 질의 한 번으로 가정했던 부모를
                        확인한다. 시야가 실제로 막혀 있었다면 vertex 를 수리한다. 부모를 이미
                        확장된 grid 이웃 중 가장 싼 것으로 바꾸는데, 이 fallback 은 항상
                        존재한다. 그 vertex 를 생성한 노드가 곧 인접한 확장 완료 셀이기 때문이다.
                        생성만 되고 확장되지 않는 노드, 즉 heuristic 탐색의 다수는 검사 비용을
                        아예 내지 않는다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Line-of-sight checks: one per expansion</strong> instead of up to
                        eight per expansion — the sandbox below counts both on the same problem.</li>
                    <li><strong>Paths match Theta* almost always.</strong> Because a repaired
                        vertex keeps its (now larger) cost without re-queuing, expansion order can
                        deviate slightly; on this site's maze benchmark the cost difference is
                        0.03% (27.7566 vs 27.7478), and frequently zero.</li>
                    <li><strong>Same asymptotics as A*</strong> in queue work; the practical win
                        is the constant on the expensive geometric primitive.</li>
                </ul>}
                ko={<ul>
                    <li><strong>line-of-sight 검사가 확장마다 한 번</strong>이다. 확장마다 최대
                        여덟 번에서 줄어든다. 아래 sandbox 가 같은 문제에서 양쪽을 센다.</li>
                    <li><strong>경로는 거의 항상 Theta*와 같다.</strong> 수리된 vertex 가
                        (커진) 비용을 가진 채 재정렬 없이 확장되므로 확장 순서가 근소하게 어긋날
                        수 있다. 이 사이트의 maze 벤치마크에서 비용 차이는 0.03%(27.7566 vs
                        27.7478)이고, 아예 0인 경우도 흔하다.</li>
                    <li>큐 작업의 <strong>점근 복잡도는 A*와 같다.</strong> 실전의 이득은 비싼
                        기하 primitive 에 붙는 상수다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Two edits to Theta*: the relaxation loses its check, and the pop gains a
                    verification step:
                </p>}
                ko={<p>
                    Theta*에서 두 군데만 고친다. relaxation 에서 검사가 빠지고, pop 에 확인
                    단계가 생긴다:
                </p>}
            />
            <Pseudocode code={`while OPEN is not empty:
    s ← pop_min(OPEN)
    p ← parent[s]
    if p ≠ s and not line_of_sight(p, s):                     # 1  (set_vertex)
        parent[s] ← cheapest expanded grid neighbor n of s    # 2
        g[s] ← g[n] + c(n, s)
    move s to CLOSED;  return path if s = goal
    for each neighbor s' of s not in CLOSED:
        candidate ← g[parent[s]] + euclidean(parent[s], s')   # 3  (no check!)
        if candidate < g[s']:
            g[s'] ← candidate;  parent[s'] ← parent[s]
            push s' with key g[s'] + h(s')`}/>
            <T
                en={<ol>
                    <li>The deferred check: exactly one line-of-sight query per expanded vertex,
                        verifying the parent that was optimistically assumed at generation.</li>
                    <li>Repair on failure: adopt the cheapest settled grid neighbor. The vertex's
                        cost rises to the honest value before it is expanded, so nothing built on
                        it inherits the optimistic error.</li>
                    <li>Generation is check-free: always path 2, straight to the grandparent.
                        This is where the savings come from — unexpanded nodes never get
                        verified.</li>
                </ol>}
                ko={<ol>
                    <li>연기된 검사. 확장되는 vertex 마다 정확히 한 번의 line-of-sight 질의로,
                        생성 때 낙관적으로 가정했던 부모를 확인한다.</li>
                    <li>실패 시 수리. 이미 settle 된 grid 이웃 중 가장 싼 것을 부모로 삼는다.
                        vertex 의 비용이 확장 전에 정직한 값으로 올라가므로, 그 위에 쌓이는
                        것들이 낙관적 오류를 물려받지 않는다.</li>
                    <li>생성은 검사가 없다. 항상 path 2로 조부모에 직결한다. 절약이 나오는
                        지점이 여기다. 확장되지 않는 노드는 끝내 확인받지 않는다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The rubble field maximizes generated edges, which is exactly what Lazy Theta*
                    saves on. Toggle between the two planners: near-identical red paths, but the
                    LOS-check counter drops severalfold. Draw extra rubble and watch the gap
                    widen.
                </p>}
                ko={<p>
                    잔해 지형은 생성되는 edge 수를 극대화하는데, Lazy Theta* 가 아끼는 것이
                    바로 그것이다. 두 planner 를 토글해 보라. 빨간 경로는 사실상 같지만 LOS 검사
                    카운터는 몇 배로 떨어진다. 잔해를 더 그리면 격차가 더 벌어진다.
                </p>}
            />
            <LazyThetaSandbox/>
            <TraceReplay algo="lazy_theta_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's Lazy Theta* demo — occasional repair events fire when an optimistic parent fails verification",
                "저장소 Lazy Theta* demo 의 실제 trace. 낙관적 부모가 확인에 실패하는 순간 수리 이벤트가 간간이 발생한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation mirrors Theta* except for the deferred check —{" "}
                    <code>set_vertex</code> at the top of the pop, optimistic path 2 in the
                    relaxation. Embedded below in full.
                </p>}
                ko={<p>
                    구현은 연기된 검사를 빼면 Theta*와 같은 뼈대다. pop 머리의{" "}
                    <code>set_vertex</code>, relaxation 의 낙관적 path 2. 전체를 아래에 embed
                    했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/lazy_theta_star.py",
                            code: lazyPy,
                            href: `${REPO}/python/navigation/global_planning/search/lazy_theta_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/lazy_theta_star.cpp",
                            code: lazyCpp,
                            href: `${REPO}/cpp/src/global_planning/search/lazy_theta_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Lazy Theta* implementation, embedded from the repository sources",
                    "Lazy Theta* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    A. Nash, S. Koenig, C. Tovey,{" "}
                    <a href="https://doi.org/10.1609/aaai.v24i1.7566" target="_blank"
                       rel="noopener noreferrer">
                        <em>Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in
                            3D</em>
                    </a>,
                    AAAI Conference on Artificial Intelligence, 2010.
                </li>
                <li>
                    K. Daniel, A. Nash, S. Koenig, A. Felner,{" "}
                    <a href="https://doi.org/10.1613/jair.2994" target="_blank"
                       rel="noopener noreferrer">
                        <em>Theta*: Any-Angle Path Planning on Grids</em>
                    </a>,
                    Journal of Artificial Intelligence Research, 2010.
                </li>
            </ol>
        </>
    )
}

export default LazyThetaStar
