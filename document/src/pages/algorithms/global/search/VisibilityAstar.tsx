import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import VisibilityFan from "../../../../components/panels/global/visibility_astar/VisibilityFan";
import VisibilitySandbox from "../../../../components/panels/global/visibility_astar/VisibilitySandbox";
import visPy from "../../../../../../python/navigation/global_planning/search/visibility_astar.py?raw";
import visCpp from "../../../../../../cpp/src/global_planning/search/visibility_astar.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const VisibilityAstar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Theta* sneaks straight lines into a grid search, one shortcut at a time.
                    Visibility A* asks the blunt follow-up: why keep the grid adjacency at all?
                    Make <em>line of sight itself</em> the successor relation — every cell you
                    can see is a neighbor at straight-line cost — and plain A* returns the
                    shortest polyline through cell centres, the graph Theta* was only
                    approximating. The idea is as old as motion planning: it is the visibility
                    graph of Lozano-Pérez &amp; Wesley (1979), rebuilt on cell centres.
                </p>}
                ko={<p>
                    Theta*는 grid 탐색에 직선을 한 번에 하나씩 끼워 넣는다. Visibility A*는
                    그 다음 질문을 정면으로 던진다. 애초에 grid 인접을 왜 유지하는가?{" "}
                    <em>line of sight 자체</em>를 successor 관계로 삼으면, 보이는 모든 셀이
                    직선거리 비용의 이웃이 되고, 평범한 A*가 셀 중심을 지나는 최단 폴리라인을
                    돌려준다. Theta*가 근사하던 바로 그 그래프다. 발상 자체는 motion planning
                    만큼 오래됐다. Lozano-Pérez &amp; Wesley(1979)의 visibility graph를 셀
                    중심 위에 다시 세운 것이다.
                </p>}
            />

            <h2>{t("From Shortcut to Successor", "지름길에서 successor로")}</h2>
            <T
                en={<>
                    <p>
                        Fix the <em>cell-centre visibility graph</em>{" "}
                        <InlineMath math="G_{\text{vis}} = (V, E)"/>: vertices are the free cells
                        reachable from the start, and{" "}
                        <InlineMath math="(u, v) \in E"/> whenever the straight segment between
                        the two cell centres crosses only free cells, weighted by the Euclidean
                        distance <InlineMath math="\lVert u - v \rVert"/>. Theta* never builds
                        this graph. It walks the grid and, at each relaxation, tests a single
                        shortcut to one candidate parent. Whether the taut turns of the best
                        route survive depends on which parent chain the search happens to commit
                        to.
                    </p>
                    <p>
                        Visibility A* searches <InlineMath math="G_{\text{vis}}"/> directly.
                        Expanding <InlineMath math="s"/> relaxes <em>every</em> cell visible
                        from it, so the frontier leaps across the map in straight lines instead
                        of diffusing cell by cell. Expansions collapse to a handful; the price
                        moves into the line-of-sight tests each expansion fires. The
                        implementation walks the visible set row by row as maximal contiguous
                        runs (intervals), which is also exactly what the trace emits for the
                        replayer.
                    </p>
                </>}
                ko={<>
                    <p>
                        <em>셀 중심 visibility graph</em>{" "}
                        <InlineMath math="G_{\text{vis}} = (V, E)"/>를 정의하자. 정점은 시작에서
                        도달 가능한 free 셀이고, 두 셀 중심을 잇는 선분이 free 셀만 지나면{" "}
                        <InlineMath math="(u, v) \in E"/>이며 가중치는 유클리드 거리{" "}
                        <InlineMath math="\lVert u - v \rVert"/>다. Theta*는 이 그래프를 만들지
                        않는다. grid를 걸으면서 relaxation마다 후보 부모 하나로의 지름길을
                        시험할 뿐이다. 최적 경로의 taut 턴이 살아남는지는 탐색이 어느 부모
                        사슬에 올라탔는지에 달려 있다.
                    </p>
                    <p>
                        Visibility A*는 <InlineMath math="G_{\text{vis}}"/>를 직접 탐색한다.{" "}
                        <InlineMath math="s"/>를 확장하면 거기서 보이는 <em>모든</em> 셀이
                        relax 되므로, frontier가 셀 단위로 번지는 대신 직선으로 맵을 건너뛴다.
                        확장 수는 몇 번으로 줄고, 비용은 확장마다 쏘는 line-of-sight 검사로
                        옮겨 간다. 구현은 가시 집합을 행별 최대 연속 구간(interval)으로
                        훑는데, trace가 replayer에 내보내는 단위도 정확히 이것이다.
                    </p>
                </>}
            />
            <VisibilityFan/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Optimal on the cell-centre visibility graph</strong>: the
                        Euclidean heuristic is admissible and consistent there, so A* returns
                        the shortest cell-centre polyline. Weighting it by{" "}
                        <InlineMath math="w > 1"/> trades that guarantee for speed
                        (Pohl, 1970).</li>
                    <li><strong>Never worse than Theta*</strong> on the same instance: any
                        Theta* output is itself a path in{" "}
                        <InlineMath math="G_{\text{vis}}"/> (proof below) — and the sandbox
                        shows cases where it is strictly better.</li>
                    <li><strong>Not the true Euclidean optimum</strong>: turning points are
                        quantized to cell centres. The genuinely shortest route may turn at an
                        obstacle corner no cell centre lands on.</li>
                    <li><strong>Cost</strong>: expansions are few, but each one scans the
                        component with a LOS test per cell — <InlineMath math="O(|V|)"/> checks
                        of <InlineMath math="O(\text{segment length})"/> each. The LOS counter
                        in the sandbox readout is the honest price tag.</li>
                </ul>}
                ko={<ul>
                    <li><strong>셀 중심 visibility graph 위에서 최적</strong>: 유클리드
                        heuristic이 이 그래프에서 admissible + consistent 하므로 A*가 최단
                        셀 중심 폴리라인을 돌려준다. <InlineMath math="w > 1"/>로 가중하면 그
                        보장을 속도와 맞바꾼다 (Pohl, 1970).</li>
                    <li>같은 문제에서 <strong>Theta*보다 나쁠 수 없다</strong>: Theta*의 출력은
                        그 자체로 <InlineMath math="G_{\text{vis}}"/>의 경로다 (아래 논증).
                        sandbox는 엄밀히 더 짧아지는 사례를 보여 준다.</li>
                    <li><strong>참 유클리드 최적은 아니다</strong>: 턴 지점이 셀 중심으로
                        양자화된다. 진짜 최단 경로는 어떤 셀 중심에도 놓이지 않는 장애물
                        모서리에서 꺾일 수 있다.</li>
                    <li><strong>비용</strong>: 확장은 몇 번 안 되지만, 확장마다 성분 전체를
                        셀당 LOS 검사로 훑는다. 검사 <InlineMath math="O(|V|)"/>회에 각각{" "}
                        <InlineMath math="O(\text{선분 길이})"/>다. sandbox readout의 LOS
                        카운터가 이 정직한 가격표다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Plain A* with an unusual successor loop. The candidate vertex set is
                    discovered once, as the start's connected free component over grid
                    neighbors; each expansion then projects its visibility onto that component
                    row by row:
                </p>}
                ko={<p>
                    successor 루프만 특이한 평범한 A*다. 후보 정점 집합은 시작의 연결 free
                    성분으로 한 번 발견해 두고, 이후 확장마다 자신의 가시 영역을 그 성분에
                    행 단위로 투영한다:
                </p>}
            />
            <Pseudocode code={`component ← free cells reachable from start via grid neighbors   # 1
g[start] ← 0;  parent[start] ← start;  push start with key h(start)
while OPEN is not empty:
    s ← pop_min(OPEN);  skip if closed;  close s
    if s = goal:  return follow parent links from goal            # 2
    for each row of component (ascending):
        for each maximal visible run [lo, hi] in that row:        # 3
            for each cell c in [lo, hi] not closed:
                candidate ← g[s] + euclidean(s, c)                # 4
                if candidate < g[c]:
                    g[c] ← candidate;  parent[c] ← s              # 5
                    push c with key g[c] + h(c)`}/>
            <T
                en={<ol>
                    <li>Restricting vertices to the start's component loses nothing: every
                        feasible path stays inside it, and a goal outside it is unreachable
                        before the search even starts.</li>
                    <li>Reconstruction follows parent links, and consecutive parents may sit
                        anywhere on the map: each link is one straight leg of the polyline.</li>
                    <li>An interval is a maximal run of row-adjacent free cells all visible
                        from <InlineMath math="s"/>. Scanning runs instead of single cells
                        keeps the inner loop simple and gives the replayer its fan-shaped
                        drawing unit.</li>
                    <li>The edge cost is the straight-line distance to{" "}
                        <InlineMath math="s"/> itself, never through intermediate cells: this
                        is a real visibility-graph edge, not a shortcut bolted onto a grid
                        step.</li>
                    <li>Standard relaxation. Since every visible cell was already offered the
                        straight edge from <InlineMath math="s"/>, no Theta*-style parent
                        repair is ever needed.</li>
                </ol>}
                ko={<ol>
                    <li>정점을 시작의 성분으로 제한해도 잃는 것이 없다. 실행 가능한 모든
                        경로는 그 안에 머물고, 성분 밖의 goal은 탐색 전에 이미 도달
                        불가능이다.</li>
                    <li>재구성은 부모 링크를 따라간다. 연속한 부모가 맵 어디에든 있을 수
                        있고, 링크 하나가 폴리라인의 직선 구간 하나다.</li>
                    <li>interval은 행에서 연속이고 전부 <InlineMath math="s"/>에서 보이는
                        free 셀의 최대 run이다. 셀 하나씩이 아니라 run 단위로 훑으면 안쪽
                        루프가 단순해지고, replayer가 부채꼴로 그릴 단위도 이것이 된다.</li>
                    <li>edge 비용은 중간 셀을 거치지 않은 <InlineMath math="s"/>까지의
                        직선거리다. grid 스텝에 덧댄 지름길이 아니라 진짜 visibility-graph
                        edge다.</li>
                    <li>표준 relaxation이다. 보이는 셀마다 이미 <InlineMath math="s"/>에서의
                        직선 edge를 제안받았으므로, Theta* 식의 부모 수리가 필요할 일이
                        없다.</li>
                </ol>}
            />
            <Proof title={t("Lemma (never worse than Theta*)", "보조정리 (Theta*보다 나쁠 수 없다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Let Theta* return the polyline{" "}
                            <InlineMath math="v_0, v_1, \dots, v_k"/> with cost{" "}
                            <InlineMath math="C_\Theta = \sum_i \lVert v_i - v_{i+1} \rVert"/>.
                            Every <InlineMath math="v_i"/> is a cell centre, and every
                            consecutive pair has line of sight (each parent link Theta* keeps is
                            LOS-verified).
                        </p>
                        <BlockMath math="(v_i, v_{i+1}) \in E \;\;\forall i \;\Rightarrow\; v_0 \dots v_k \text{ is a path in } G_{\text{vis}} \;\Rightarrow\; \mathrm{opt}(G_{\text{vis}}) \le C_\Theta"/>
                        <Terms items={[
                            ["v_0, \\dots, v_k", "the vertices of the Theta* polyline, all cell centres, consecutive pairs mutually visible"],
                            ["E", <>edge set of the cell-centre visibility graph <InlineMath math="G_{\\text{vis}}"/>: pairs of cells with line of sight, weighted by straight-line distance</>],
                            ["C_\\Theta", "total cost of the Theta* path"],
                            ["\\mathrm{opt}(G_{\\text{vis}})", <>cost of the shortest path in <InlineMath math="G_{\\text{vis}}"/> — what Visibility A* returns</>],
                        ]}/>
                        <p>
                            A* with a consistent heuristic returns{" "}
                            <InlineMath math="\mathrm{opt}(G_{\text{vis}})"/>, hence a cost{" "}
                            <InlineMath math="\le C_\Theta"/>.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> Theta*가 폴리라인{" "}
                            <InlineMath math="v_0, v_1, \dots, v_k"/>를 비용{" "}
                            <InlineMath math="C_\Theta = \sum_i \lVert v_i - v_{i+1} \rVert"/>로
                            돌려줬다고 하자. 모든 <InlineMath math="v_i"/>는 셀 중심이고, 연속한
                            쌍마다 line of sight가 있다 (Theta*가 유지하는 부모 링크는 전부
                            LOS 검증을 거친다).
                        </p>
                        <BlockMath math="(v_i, v_{i+1}) \in E \;\;\forall i \;\Rightarrow\; v_0 \dots v_k \text{ 는 } G_{\text{vis}} \text{ 의 경로} \;\Rightarrow\; \mathrm{opt}(G_{\text{vis}}) \le C_\Theta"/>
                        <Terms items={[
                            ["v_0, \\dots, v_k", "Theta* 폴리라인의 꼭짓점들. 전부 셀 중심이고, 연속한 쌍은 서로 보인다"],
                            ["E", <>셀 중심 visibility graph <InlineMath math="G_{\\text{vis}}"/>의 간선 집합. line of sight가 있는 셀 쌍이고 가중치는 직선거리</>],
                            ["C_\\Theta", "Theta* 경로의 총비용"],
                            ["\\mathrm{opt}(G_{\\text{vis}})", <><InlineMath math="G_{\\text{vis}}"/> 최단 경로의 비용. Visibility A*가 돌려주는 값</>],
                        ]}/>
                        <p>
                            consistent heuristic을 쓴 A*는{" "}
                            <InlineMath math="\mathrm{opt}(G_{\text{vis}})"/>를 돌려주므로 비용이{" "}
                            <InlineMath math="C_\Theta"/> 이하다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("Not the True Optimum", "참 최적은 아니다")}</h2>
            <T
                en={<>
                    <p>
                        The classical visibility graph places vertices at <em>obstacle
                        corners</em>, because a shortest path in the plane only ever turns
                        there. Cell centres are a stand-in for those corners, off by up to half
                        a diagonal per turn, so the cell-centre optimum can exceed the true
                        one. The gap is small and bounded by the grid resolution, but it is
                        structural: no amount of extra search fixes it, because the right
                        turning point is simply not in the vertex set.
                    </p>
                    <p>
                        Closing that gap without leaving the grid is exactly the interval
                        trick of Anya, the next algorithm in this lineage: it searches over
                        contiguous <em>row intervals</em> instead of cells, letting turning
                        points slide continuously along rows — and recovers the true Euclidean
                        optimum.
                    </p>
                </>}
                ko={<>
                    <p>
                        고전적인 visibility graph는 정점을 <em>장애물 모서리</em>에 둔다.
                        평면의 최단 경로는 오직 거기서만 꺾이기 때문이다. 셀 중심은 그
                        모서리의 대역이라 턴마다 최대 반 대각선만큼 어긋나고, 셀 중심 최적이
                        참 최적을 넘어설 수 있다. 그 차이는 작고 grid 해상도로 유계지만
                        구조적이다. 올바른 턴 지점이 정점 집합에 아예 없으므로 탐색을 더
                        해도 고쳐지지 않는다.
                    </p>
                    <p>
                        grid를 떠나지 않고 이 간극을 닫는 것이 이 계보의 다음 알고리즘인
                        Anya의 interval 트릭이다. 셀 대신 행의 연속 <em>interval</em> 위를
                        탐색해 턴 지점이 행을 따라 연속적으로 미끄러지게 하고, 참 유클리드
                        최적을 회복한다.
                    </p>
                </>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs the live engine on staggered slabs built so the best
                    route must wrap both inner corners: Visibility A* finds it in a handful of
                    expansions while the dashed Theta* overlay misses one taut turn and pays
                    for it. The replay below is the repository demo on the benchmark maps —
                    watch single expansions claim whole swaths of the map.
                </p>}
                ko={<p>
                    sandbox는 엇갈린 슬래브 위에서 라이브 엔진을 돌린다. 최적 경로가 두
                    안쪽 모서리를 모두 감아야 하는 배치라, Visibility A*는 확장 몇 번 만에
                    그 경로를 찾고 점선 Theta* 겹치기는 taut 턴 하나를 놓쳐 값을 치른다.
                    아래 replay는 벤치마크 맵 위의 저장소 demo다. 확장 한 번이 맵의 넓은
                    영역을 통째로 쓸어 가는 것을 보라.
                </p>}
            />
            <VisibilitySandbox/>
            <TraceReplay algo="visibility_astar" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's Visibility A* demo — each expansion relaxes every cell it can see, so the closed set stays tiny",
                "저장소 Visibility A* demo의 실제 trace. 확장마다 보이는 모든 셀이 relax 되므로 closed 집합이 아주 작게 유지된다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The successor loop is the interval projection over the start's component;
                    everything else is the A* you have seen since the third page of this
                    section. Embedded below in full.
                </p>}
                ko={<p>
                    successor 루프가 시작 성분 위의 interval 투영이고, 나머지는 이 섹션
                    세 번째 페이지부터 봐 온 그 A*다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/visibility_astar.py",
                            code: visPy,
                            href: `${REPO}/python/navigation/global_planning/search/visibility_astar.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/visibility_astar.cpp",
                            code: visCpp,
                            href: `${REPO}/cpp/src/global_planning/search/visibility_astar.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Visibility A* implementation, embedded from the repository sources",
                    "Visibility A* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    T. Lozano-Pérez, M. A. Wesley,{" "}
                    <a href="https://doi.org/10.1145/359156.359164" target="_blank"
                       rel="noopener noreferrer">
                        <em>An Algorithm for Planning Collision-Free Paths Among Polyhedral
                            Obstacles</em>
                    </a>,
                    Communications of the ACM, 1979.
                </li>
                <li>
                    K. Daniel, A. Nash, S. Koenig, A. Felner,{" "}
                    <a href="https://doi.org/10.1613/jair.2994" target="_blank"
                       rel="noopener noreferrer">
                        <em>Theta*: Any-Angle Path Planning on Grids</em>
                    </a>,
                    Journal of Artificial Intelligence Research, 2010.
                </li>
                <li>
                    I. Pohl,{" "}
                    <a href="https://doi.org/10.1016/0004-3702(70)90007-X" target="_blank"
                       rel="noopener noreferrer">
                        <em>Heuristic Search Viewed as Path Finding in a Graph</em>
                    </a>,
                    Artificial Intelligence, 1970.
                </li>
            </ol>
        </>
    )
}

export default VisibilityAstar
