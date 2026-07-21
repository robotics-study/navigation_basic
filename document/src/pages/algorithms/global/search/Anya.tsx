import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import CornerTurn from "../../../../components/panels/global/anya/CornerTurn";
import AnyaSandbox from "../../../../components/panels/global/anya/AnyaSandbox";
import {runAnyaFull} from "../../../../libs/algorithms/anya";
import anyaPy from "../../../../../../python/navigation/global_planning/search/anya.py?raw";
import anyaCpp from "../../../../../../cpp/src/global_planning/search/anya.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Anya = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Theta* approximates, Visibility A* perfects the approximation — and both
                    still miss the true shortest path, because they only ever turn at cell
                    centres. Anya (Harabor, Grastien, Öz &amp; Aksakalli, 2016) removes that last
                    restriction: turning points live on <em>grid corners</em>, the lattice
                    points where obstacle boundaries actually bend, and the search sweeps
                    row intervals of visibility instead of enumerating cells. The result is the
                    exact Euclidean shortest any-angle path — the destination this whole
                    any-angle lineage has been walking toward.
                </p>}
                ko={<p>
                    Theta*는 근사하고, Visibility A*는 그 근사를 완성한다. 그래도 둘 다 참
                    최단 경로는 놓친다. 꺾는 지점이 언제나 셀 중심이기 때문이다. Anya(Harabor,
                    Grastien, Öz &amp; Aksakalli, 2016)는 그 마지막 제약을 없앤다. turning
                    point를 장애물 경계가 실제로 꺾이는 격자점, 곧 <em>grid corner</em> 위에
                    두고, 셀을 열거하는 대신 행 단위 가시 interval을 sweep 한다. 결과는 정확한
                    유클리드 최단 any-angle 경로다. 이 any-angle 계보가 걸어온 종착지다.
                </p>}
            />

            <h2>{t("Turning at Corners", "모서리에서 꺾는다")}</h2>
            <T
                en={<>
                    <p>
                        Pull a string tight between start and goal around the obstacles. The
                        taut string is the shortest path, and it bends only where something
                        physically stops it from straightening: at <em>convex obstacle
                        corners</em>. On a grid map those corners are lattice points — integer{" "}
                        <InlineMath math="(x, y)"/> vertices where free and blocked cells meet.
                        No cell centre sits on a corner, which is exactly why every cell-centre
                        planner leaves a sliver of length on the table.
                    </p>
                    <p>
                        So the right vertex set is: the start, the goal, and the obstacle
                        corners. Anya searches this set with plain A*, ordered by
                    </p>
                    <BlockMath math="f(r) \;=\; g(r) + \lVert r - \text{goal} \rVert"/>
                    <Terms items={[
                        ["r", <><strong>the new node type</strong>: a root — an obstacle corner (or the start), not a cell</>],
                        ["g(r)", <>exact Euclidean length of the best taut start→<InlineMath math="r"/> polyline found so far</>],
                        ["\\lVert r - \\text{goal} \\rVert", "straight-line distance to the goal: admissible, since no path is shorter than the straight line"],
                    ]}/>
                    <p>
                        What makes it Anya rather than a plain corner-graph search is{" "}
                        <em>how successors are generated</em>: instead of testing every corner
                        pair for visibility, an expansion sweeps the root's view row by row as
                        contiguous <em>intervals</em>, projecting each interval to the next row
                        and re-splitting it at obstacle walls. Corners caught in the sweep
                        become successors; everything else is never touched.
                    </p>
                </>}
                ko={<>
                    <p>
                        시작과 목표 사이에 실을 걸고 장애물에 감아 팽팽히 당겨 보라. 팽팽한
                        실이 곧 최단 경로이고, 실이 꺾이는 곳은 펴지는 것을 물리적으로 막는
                        지점, 곧 <em>볼록 장애물 모서리</em>뿐이다. grid 맵에서 그 모서리는
                        free 셀과 blocked 셀이 만나는 정수 <InlineMath math="(x, y)"/> 격자점이다.
                        어떤 셀 중심도 모서리 위에 있지 않다. 셀 중심 planner 들이 길이를 조금씩
                        흘리는 이유가 정확히 이것이다.
                    </p>
                    <p>
                        따라서 올바른 정점 집합은 시작, 목표, 그리고 장애물 모서리들이다.
                        Anya는 이 집합을 평범한 A*로 탐색한다. 정렬 기준은
                    </p>
                    <BlockMath math="f(r) \;=\; g(r) + \lVert r - \text{goal} \rVert"/>
                    <Terms items={[
                        ["r", <><strong>새로운 노드 타입</strong>: root. 셀이 아니라 장애물 모서리(또는 시작점)다</>],
                        ["g(r)", <>지금까지 찾은 최선의 taut 시작→<InlineMath math="r"/> 폴리라인의 정확한 유클리드 길이</>],
                        ["\\lVert r - \\text{goal} \\rVert", "목표까지의 직선거리. 직선보다 짧은 경로는 없으므로 admissible 하다"],
                    ]}/>
                    <p>
                        이것을 단순한 corner-graph 탐색이 아니라 Anya로 만드는 것은{" "}
                        <em>successor 생성 방식</em>이다. 모든 모서리 쌍의 가시성을 검사하는
                        대신, 확장은 root의 시야를 행 단위의 연속 <em>interval</em>로 sweep
                        한다. interval을 다음 행으로 투영하고 장애물 벽에서 다시 쪼갠다.
                        sweep에 걸린 모서리만 successor가 되고, 나머지는 건드리지도 않는다.
                    </p>
                </>}
            />
            <CornerTurn/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Truly optimal</strong>: returns the exact Euclidean shortest
                        any-angle path (proof sketch below) — the first planner in this section
                        with that guarantee. Anya ≤ Visibility A* ≤ Theta* on every instance.</li>
                    <li><strong>Complete</strong>: corners of the start's free component are
                        finite; unreachable goals are detected when the frontier empties.</li>
                    <li><strong>Cost</strong>: expansions are very few (corners only), but each
                        one runs an interval sweep across the map. The trade is the same as
                        Visibility A*'s, pushed further: almost no queue work, all geometry.</li>
                    <li><strong>No tuning knob</strong>: the only parameter is a float
                        tolerance for corner/pinch tests. Optimality does not depend on it.</li>
                    <li>This repository's variant makes corners the search nodes and uses the
                        interval sweep for successor generation; the paper's Anya goes one step
                        further and makes the (interval, root) pairs themselves the nodes,
                        deferring even corner enumeration.</li>
                </ul>}
                ko={<ul>
                    <li><strong>참 최적</strong>: 정확한 유클리드 최단 any-angle 경로를
                        돌려준다 (아래 증명 스케치). 이 섹션에서 그 보장을 가진 첫 planner다.
                        모든 문제에서 Anya ≤ Visibility A* ≤ Theta*.</li>
                    <li><strong>완전</strong>: 시작 성분의 모서리는 유한하고, 도달 불가능한
                        목표는 frontier가 비면 검출된다.</li>
                    <li><strong>비용</strong>: 확장 수는 아주 적지만(모서리뿐), 확장마다 맵을
                        가로지르는 interval sweep이 돈다. Visibility A*의 거래를 한 발 더
                        밀어붙인 셈이다. 큐 작업은 거의 없고, 전부 기하다.</li>
                    <li><strong>튜닝 손잡이가 없다</strong>: 유일한 파라미터는 corner/pinch
                        판정용 float 허용오차다. 최적성은 그 값에 의존하지 않는다.</li>
                    <li>이 저장소의 변형은 모서리를 탐색 노드로 삼고 interval sweep 을
                        successor 생성에 쓴다. 논문의 Anya는 한 걸음 더 나아가 (interval,
                        root) 쌍 자체를 노드로 삼아 모서리 열거까지 미룬다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    A* over corner roots. The interesting work is in the successor sweep — a
                    beam of visibility pushed row by row away from the root:
                </p>}
                ko={<p>
                    corner root 위의 A*다. 재미있는 부분은 successor sweep, 곧 root에서 행
                    단위로 밀려 나가는 가시성 beam이다:
                </p>}
            />
            <Pseudocode code={`g[start] ← 0;  push start with key h(start)
while OPEN is not empty:
    r ← pop_min(OPEN);  skip if settled
    if f(r) ≥ best goal cost found:  return that path            # 1
    settle r
    if segment r → goal is clear:  update best goal cost         # 2
    for each sweep direction (up, down):
        beam ← visible intervals of the adjacent row             # 3
        while beam is not empty:
            collect obstacle corners inside beam as successors   # 4
            project beam to the next row; split at walls         # 5
    also walk r's own row to its flat corner successors          # 6
    for each successor corner c:
        relax g[c] ← min(g[c], g[r] + euclidean(r, c))`}/>
            <T
                en={<ol>
                    <li>The goal is not a corner, so the loop stops when no queued root could
                        possibly beat the best complete path found — the standard A* stopping
                        rule adapted to a goal reached by a final straight leg.</li>
                    <li>Any settled corner that sees the goal closes a candidate path with one
                        straight segment.</li>
                    <li>An interval is a maximal visible run of a row; the first beam is the
                        row just above (or below) the root.</li>
                    <li>Only corners inside the lit region become successors — the sweep is
                        the visibility test, no per-pair line-of-sight query needed.</li>
                    <li>Projection follows the rays root→interval-endpoints to the next row;
                        walls split the projected interval, shadows shrink it, and an empty
                        beam ends the sweep.</li>
                    <li>Corners along the root's own row are reachable by a flat segment and
                        would be missed by the row-by-row cone — they are walked directly.</li>
                </ol>}
                ko={<ol>
                    <li>목표는 모서리가 아니므로, 큐의 어떤 root도 이미 찾은 최선의 완성
                        경로를 이길 수 없을 때 루프가 멈춘다. 마지막 직선 구간으로 도달하는
                        목표에 맞춘 표준 A* 종료 규칙이다.</li>
                    <li>settle 된 모서리가 목표를 볼 수 있으면 직선 구간 하나로 후보 경로가
                        닫힌다.</li>
                    <li>interval은 행의 최대 가시 run이다. 첫 beam은 root 바로 위(또는
                        아래) 행이다.</li>
                    <li>빛이 닿는 영역 안의 모서리만 successor가 된다. sweep 자체가 가시성
                        검사라서 쌍마다 line-of-sight를 따로 물을 필요가 없다.</li>
                    <li>투영은 root→interval 끝점 광선을 따라 다음 행으로 간다. 벽이 투영된
                        interval을 쪼개고, 그림자가 줄이며, beam이 비면 sweep이 끝난다.</li>
                    <li>root 자기 행의 모서리는 수평 구간으로 닿는데 행 단위 cone에는 안
                        걸린다. 그래서 따로 걸어서 찾는다.</li>
                </ol>}
            />
            <Proof title={t("Why corners suffice (taut-string argument)", "모서리로 충분한 이유 (taut string 논증)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Let <InlineMath math="\pi"/> be a shortest
                            path and <InlineMath math="b"/> an interior bend point of it that is{" "}
                            <em>not</em> a convex obstacle corner. Take points{" "}
                            <InlineMath math="u, v"/> on <InlineMath math="\pi"/> just before and
                            after <InlineMath math="b"/>.
                        </p>
                        <BlockMath math="\lVert u - v \rVert \;\overset{\triangle}{<}\; \lVert u - b \rVert + \lVert b - v \rVert \quad \text{and} \quad \overline{uv} \text{ is collision-free for } u, v \text{ close enough}"/>
                        <Terms items={[
                            ["\\pi,\\ b", <>a shortest path and one of its bend points; <InlineMath math="b"/> assumed <em>not</em> to be an obstacle corner</>],
                            ["u,\\ v", <>points of <InlineMath math="\pi"/> just before / after <InlineMath math="b"/></>],
                            ["\\overline{uv}", <>the straight shortcut replacing <InlineMath math="u \to b \to v"/>: nothing blocks it near a non-corner bend</>],
                        ]}/>
                        <p>
                            Replacing <InlineMath math="u \to b \to v"/> with{" "}
                            <InlineMath math="\overline{uv}"/> shortens <InlineMath math="\pi"/> —
                            contradiction. So every bend of a shortest path is a convex obstacle
                            corner, and searching start ∪ corners ∪ goal loses nothing.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 최단 경로 <InlineMath math="\pi"/>의 내부
                            꺾임점 <InlineMath math="b"/>가 볼록 장애물 모서리가 <em>아니라고</em>{" "}
                            하자. <InlineMath math="\pi"/> 위에서 <InlineMath math="b"/> 직전과
                            직후의 점 <InlineMath math="u, v"/>를 잡으면:
                        </p>
                        <BlockMath math="\lVert u - v \rVert \;\overset{\triangle}{<}\; \lVert u - b \rVert + \lVert b - v \rVert \quad \text{이고} \quad u, v \text{가 충분히 가까우면 } \overline{uv} \text{는 충돌이 없다}"/>
                        <Terms items={[
                            ["\\pi,\\ b", <>최단 경로와 그 꺾임점. <InlineMath math="b"/>는 장애물 모서리가 <em>아니라고</em> 가정</>],
                            ["u,\\ v", <><InlineMath math="\pi"/> 위에서 <InlineMath math="b"/> 직전 / 직후의 점</>],
                            ["\\overline{uv}", <><InlineMath math="u \to b \to v"/>를 대체하는 직선 지름길. 모서리가 아닌 꺾임 근처에는 막는 것이 없다</>],
                        ]}/>
                        <p>
                            <InlineMath math="u \to b \to v"/>를 <InlineMath math="\overline{uv}"/>로
                            바꾸면 <InlineMath math="\pi"/>가 짧아진다. 모순. 따라서 최단 경로의
                            모든 꺾임은 볼록 장애물 모서리이고, 시작 ∪ 모서리 ∪ 목표만 탐색해도
                            잃는 것이 없다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    First the anatomy of a single turn: corner versus cell centre, half a cell
                    apart. Then the sandbox — the red path bends exactly on the block's lattice
                    corners, and the readout pits all three any-angle planners against each
                    other on the same problem. The replay below is the repository demo on the
                    benchmark maps: a handful of corner expansions where grid planners touched
                    hundreds of cells.
                </p>}
                ko={<p>
                    먼저 턴 하나의 해부도부터. 모서리와 셀 중심은 반 셀 차이다. 다음은
                    sandbox다. 빨간 경로가 블록의 격자 모서리에서 정확히 꺾이고, readout 은
                    같은 문제에서 any-angle planner 셋을 맞붙인다. 아래 replay는 벤치마크 맵
                    위의 저장소 demo다. grid planner 들이 수백 셀을 만지던 곳에서 모서리 확장
                    몇 번이면 끝난다.
                </p>}
            />
            <AnyaSandbox/>
            <TraceReplay algo="anya" maps={["open01", "maze01"]}
                         truePathOf={(map, start, goal, params) => runAnyaFull({
                             map, start, goal,
                             vertexEpsilon: typeof params?.vertex_epsilon === "number"
                                 ? params.vertex_epsilon : 1e-9,
                         }).geometry}
                         label={t(
                             "Real traces from the repository's Anya demo — expansions are obstacle corners, so the closed set is a handful of dots",
                             "저장소 Anya demo의 실제 trace. 확장이 장애물 모서리뿐이라 closed 집합이 점 몇 개로 끝난다",
                         )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The corner-level geometry (segment clearance, interval projection,
                    splitting) lives beside the A* loop; occupancy is observed only through the
                    capability's reachable component. Embedded below in full.
                </p>}
                ko={<p>
                    corner 수준의 기하(선분 통과 검사, interval 투영, 분할)가 A* 루프 옆에
                    있고, occupancy는 capability의 도달 가능 성분으로만 관찰한다. 전체를
                    아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/anya.py",
                            code: anyaPy,
                            href: `${REPO}/python/navigation/global_planning/search/anya.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/anya.cpp",
                            code: anyaCpp,
                            href: `${REPO}/cpp/src/global_planning/search/anya.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Anya implementation, embedded from the repository sources",
                    "Anya 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    D. Harabor, A. Grastien, D. Öz, V. Aksakalli,{" "}
                    <a href="https://doi.org/10.1613/jair.5007" target="_blank"
                       rel="noopener noreferrer">
                        <em>Optimal Any-Angle Pathfinding In Practice</em>
                    </a>,
                    Journal of Artificial Intelligence Research, 2016.
                </li>
                <li>
                    D. Harabor, A. Grastien,{" "}
                    <a href="https://doi.org/10.1609/icaps.v23i1.13568" target="_blank"
                       rel="noopener noreferrer">
                        <em>An Optimal Any-Angle Pathfinding Algorithm</em>
                    </a>,
                    International Conference on Automated Planning and Scheduling, 2013.
                </li>
                <li>
                    T. Lozano-Pérez, M. A. Wesley,{" "}
                    <a href="https://doi.org/10.1145/359156.359164" target="_blank"
                       rel="noopener noreferrer">
                        <em>An Algorithm for Planning Collision-Free Paths Among Polyhedral
                            Obstacles</em>
                    </a>,
                    Communications of the ACM, 1979.
                </li>
            </ol>
        </>
    )
}

export default Anya
