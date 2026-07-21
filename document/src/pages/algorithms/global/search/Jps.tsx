import {T, useTr} from "../../../../libs/i18n";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import JpsSandbox from "../../../../components/panels/global/jps/JpsSandbox";
import jpsPy from "../../../../../../python/navigation/global_planning/search/jps.py?raw";
import jpsCpp from "../../../../../../cpp/src/global_planning/search/jps.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const Jps = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    On an open grid, most of what A* expands is waste. Between two points there
                    are exponentially many optimal staircase paths — swap the order of the same
                    moves and the cost is identical — and A* dutifully explores representatives
                    of all of them. Jump Point Search (Harabor &amp; Grastien, 2011) breaks that
                    symmetry with pure geometry: no preprocessing, no memory, same optimal paths,
                    often an order of magnitude fewer expansions.
                </p>}
                ko={<p>
                    열린 grid 에서 A* 확장의 대부분은 낭비다. 두 점 사이에는 같은 이동들의
                    순서만 바꾼, 비용이 똑같은 계단 경로가 지수적으로 많고, A* 는 그 대표들을
                    성실하게 전부 탐색한다. Jump Point Search(Harabor &amp; Grastien, 2011)는 그
                    대칭을 순수 기하만으로 깨뜨린다. 전처리도, 추가 메모리도 없이 같은 최적
                    경로를 흔히 자릿수 단위로 적은 확장으로 찾는다.
                </p>}
            />

            <h2>{t("Symmetry Is the Enemy", "대칭이 적이다")}</h2>
            <T
                en={<>
                    <p>
                        The idea: commit to a <em>canonical ordering</em> of equivalent paths —
                        say, diagonal moves before straight ones — and refuse to expand any node
                        that some other node on the same canonical path can reach equally cheaply.
                        What remains? A node matters only where a canonical path is <em>forced</em>{" "}
                        to branch, and on a uniform grid that happens only next to obstacle
                        corners. Those cells are the <strong>jump points</strong>.
                    </p>
                    <p>
                        Concretely, instead of pushing eight neighbors, JPS <em>scans</em> each
                        allowed direction in a straight line, skipping cell after cell, and stops
                        only at the goal or at a cell with a <em>forced neighbor</em> — this
                        repository's grid forbids corner-cutting, so a forced neighbor appears
                        when an obstacle sits diagonally <em>behind</em> the direction of travel,
                        opening a side cell reachable only by turning here. A diagonal scan
                        delegates: after each diagonal step it probes both orthogonal directions,
                        and becomes a jump point itself if either probe finds one.
                    </p>
                </>}
                ko={<>
                    <p>
                        아이디어는 동치 경로들에 <em>정준 순서</em>를 부여하는 것이다. 예컨대
                        대각 이동을 직선 이동보다 먼저 하기로 정하고, 같은 정준 경로 위의 다른
                        노드가 똑같이 싸게 도달할 수 있는 노드는 확장 자체를 거부한다. 그러면
                        남는 것은? 정준 경로가 <em>어쩔 수 없이</em> 갈라지는 지점뿐이고, 균일
                        grid 에서 그런 일은 장애물 모서리 옆에서만 생긴다. 그 셀들이{" "}
                        <strong>jump point</strong> 다.
                    </p>
                    <p>
                        구체적으로 JPS 는 이웃 여덟을 큐에 넣는 대신, 허용된 각 방향으로 셀을
                        건너뛰며 직선 <em>스캔</em>을 하고, goal 이나 <em>forced neighbor</em> 가
                        있는 셀에서만 멈춘다. 이 저장소의 grid 는 corner-cutting 을 금지하므로,
                        forced neighbor 는 진행 방향의 대각 <em>뒤</em>에 장애물이 있어 여기서
                        꺾어야만 닿는 옆 칸이 열릴 때 생긴다. 대각 스캔은 위임한다. 대각 한 걸음
                        마다 두 직교 방향을 찔러 보고, 어느 쪽이든 jump point 를 찾으면 자신이
                        jump point 가 된다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Exactly A*'s optimal costs</strong> — pruning removes only
                        symmetric duplicates, never the cheapest route (Harabor &amp; Grastien
                        2011 prove path preservation).</li>
                    <li><strong>No preprocessing, no extra memory</strong> — everything is derived
                        from a single-cell occupancy oracle at query time, so it works on maps
                        that change between queries.</li>
                    <li><strong>Expansions collapse</strong>; the scans still touch cells, but a
                        cell probe is far cheaper than a queue insertion. Worst case (dense
                        clutter) degrades toward plain A*.</li>
                    <li><strong>Uniform-cost 8-connected grids only</strong> — the symmetry
                        argument dies with weighted cells or arbitrary graphs.</li>
                </ul>}
                ko={<ul>
                    <li><strong>비용은 A*의 최적과 정확히 같다.</strong> 가지치기가 제거하는
                        것은 대칭 중복뿐이며 가장 싼 경로는 절대 잃지 않는다 (Harabor &amp;
                        Grastien 2011의 경로 보존 증명).</li>
                    <li><strong>전처리도 추가 메모리도 없다.</strong> 전부 질의 시점의 단일 셀
                        점유 oracle 에서 유도되므로, 질의 사이에 바뀌는 맵에서도 동작한다.</li>
                    <li><strong>확장 수가 무너지듯 준다.</strong> 스캔이 셀을 훑긴 하지만 셀
                        probe 는 큐 삽입보다 훨씬 싸다. 최악의 경우(빽빽한 장애물)에는 A* 수준으로
                        퇴화한다.</li>
                    <li><strong>균일 비용 8-connected grid 전용</strong>이다. 셀 가중치나 일반
                        그래프에서는 대칭 논증 자체가 성립하지 않는다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The outer loop is A* verbatim; only the successor generator changes — and the
                    start, having no incoming direction, scans all eight:
                </p>}
                ko={<p>
                    바깥 루프는 A* 그대로다. successor 생성기만 달라진다. 들어온 방향이 없는
                    시작 노드는 여덟 방향을 모두 스캔한다:
                </p>}
            />
            <Pseudocode code={`successors(u):
    for each direction d allowed by how u was reached:        # 1
        jp ← scan(u, d)                                       # 2
        if jp exists:  yield (jp, octile(u, jp))

scan(u, d):
    step from u along d while the move is legal:
        if cell = goal:  return cell                          # 3
        if straight d and an obstacle sits diagonally behind, # 4
           opening a free side cell:  return cell
        if diagonal d and (scan(cell, d_horizontal) or        # 5
                           scan(cell, d_vertical)):  return cell
    return none                                               # dead end`}/>
            <T
                en={<ol>
                    <li>Direction pruning: after a straight move, only the continuation (plus
                        forced branches) is allowed; after a diagonal, the continuation and its
                        two orthogonal legs. This is what enforces the canonical ordering.</li>
                    <li>The scan replaces up to hundreds of queue operations with a cache-friendly
                        straight walk.</li>
                    <li>The goal always terminates a scan — it must be reachable as a jump
                        target.</li>
                    <li>The forced-neighbor rule (no-corner-cutting variant): turning is only
                        ever necessary just past an obstacle corner.</li>
                    <li>Diagonal scans recurse one level into straight probes; a diagonal cell
                        that "sees" a jump point orthogonally is where the path must branch.</li>
                </ol>}
                ko={<ol>
                    <li>방향 가지치기. 직선 이동 뒤에는 그 연장(과 forced 분기)만, 대각 이동
                        뒤에는 연장과 두 직교 성분만 허용한다. 정준 순서를 강제하는 장치가
                        이것이다.</li>
                    <li>스캔 하나가 수백 번의 큐 연산을 캐시 친화적인 직선 순회로 대체한다.</li>
                    <li>goal 은 항상 스캔을 멈춘다. jump 대상으로 도달 가능해야 하기 때문이다.</li>
                    <li>forced-neighbor 규칙(no-corner-cutting 변형). 꺾어야 하는 순간은 장애물
                        모서리를 막 지난 직후뿐이다.</li>
                    <li>대각 스캔은 직선 probe 로 한 단계 위임한다. 직교 방향으로 jump point 가
                        "보이는" 대각 셀이 곧 경로가 갈라져야 하는 지점이다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The hall is symmetry heaven. Toggle the A* shadow: the gray region is every
                    cell A* expands, the indigo dots are JPS's expansions, and the long straight
                    tree edges are jumps landing directly on obstacle corners. Add clutter and
                    watch the two counts converge — symmetry is exactly what dense maps lack.
                </p>}
                ko={<p>
                    홀은 대칭의 천국이다. A* 그림자를 켜 보라. 회색 영역이 A* 가 확장하는 모든
                    셀이고, 남색 점이 JPS 의 확장이며, 길게 뻗은 트리 선이 장애물 모서리에 바로
                    내려앉는 jump 다. 장애물을 더 그리면 두 수치가 가까워진다. 빽빽한 맵에
                    없는 것이 바로 대칭이다.
                </p>}
            />
            <JpsSandbox/>
            <TraceReplay algo="jps" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's JPS demo — a handful of expansions where A* needed hundreds",
                "저장소 JPS demo 의 실제 trace. A* 가 수백 번 확장하던 곳을 몇 번으로 끝낸다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation is the A* loop with <code>scan</code> and the direction
                    pruning — embedded below in full. Note the successor edges carry octile
                    costs of whole jumps, not unit steps.
                </p>}
                ko={<p>
                    구현은 A* 루프에 <code>scan</code> 과 방향 가지치기를 더한 것이다. 전체를
                    아래에 embed 했다. successor edge 가 단위 스텝이 아니라 jump 전체의 octile
                    비용을 실어 나르는 점을 눈여겨보라.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/jps.py",
                            code: jpsPy,
                            href: `${REPO}/python/navigation/global_planning/search/jps.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/jps.cpp",
                            code: jpsCpp,
                            href: `${REPO}/cpp/src/global_planning/search/jps.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete JPS implementation, embedded from the repository sources",
                    "JPS 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    D. Harabor, A. Grastien,{" "}
                    <a href="https://doi.org/10.1609/aaai.v25i1.7994" target="_blank"
                       rel="noopener noreferrer">
                        <em>Online Graph Pruning for Pathfinding on Grid Maps</em>
                    </a>,
                    AAAI Conference on Artificial Intelligence, 2011.
                </li>
                <li>
                    D. Harabor, A. Grastien,{" "}
                    <a href="https://cdn.aaai.org/ojs/13593/13593-52-17111-1-2-20201228.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Improving Jump Point Search</em>
                    </a>,
                    International Conference on Automated Planning and Scheduling (ICAPS), 2014.
                </li>
            </ol>
        </>
    )
}

export default Jps
