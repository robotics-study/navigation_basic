import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import ThetaStarSandbox from "../../../../components/panels/global/theta_star/ThetaStarSandbox";
import SupercoverLos from "../../../../components/panels/global/theta_star/SupercoverLos";
import thetaPy from "../../../../../../python/navigation/global_planning/search/theta_star.py?raw";
import thetaCpp from "../../../../../../cpp/src/global_planning/search/theta_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const ThetaStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every planner so far returns paths glued to grid edges: horizontal, vertical,
                    45°. Real robots do not turn in eight directions. Theta* (Nash, Daniel,
                    Koenig &amp; Felner, 2007) keeps A*'s search but lets a path segment run{" "}
                    <em>straight through</em> the grid whenever there is line of sight — the
                    founding algorithm of any-angle planning.
                </p>}
                ko={<p>
                    지금까지의 planner 는 전부 grid edge 에 붙은 경로를 돌려준다. 수평, 수직,
                    45°. 실제 로봇은 여덟 방향으로만 돌지 않는다. Theta*(Nash, Daniel, Koenig
                    &amp; Felner, 2007)는 A*의 탐색을 유지하되, line of sight 가 있으면 경로
                    구간이 grid 를 <em>곧장 가로지르게</em> 한다. any-angle planning 의 시조다.
                </p>}
            />

            <h2>{t("Grids Lie About Distance", "격자는 거리를 왜곡한다")}</h2>
            <T
                en={<>
                    <p>
                        An 8-connected grid path can only move in multiples of 45°, so its length
                        overshoots the true shortest path by up to{" "}
                        <InlineMath math="\approx 8\%"/>{" "}
                        (<InlineMath math="2\cos(\pi/8)/(1+\cos(\pi/4)) - 1"/> in the worst
                        direction) — and worse, it <em>looks</em> wrong: long zigzags where a
                        straight line obviously fits. Smoothing the A* path afterwards helps but
                        cannot fix it: the smoother only sees the one path A* happened to return,
                        which may thread the wrong side of an obstacle entirely.
                    </p>
                    <p>
                        Theta*'s idea is to smooth <em>during</em> the search. When expanding{" "}
                        <InlineMath math="s"/>, each neighbor gets to try two parents:
                    </p>
                    <BlockMath math="\text{path 1: } g(s) + c(s, s') \qquad \text{path 2: } g(\text{parent}(s)) + \lVert \text{parent}(s) - s' \rVert \;\;\text{if line-of-sight}"/>
                    <p>
                        Path 2 skips <InlineMath math="s"/> and connects the neighbor straight to
                        the expanded node's parent. Chains of such shortcuts collapse whole zigzag
                        stretches into single straight segments whose endpoints hug obstacle
                        corners.
                    </p>
                </>}
                ko={<>
                    <p>
                        8-connected grid 경로는 45°의 배수로만 움직일 수 있어, 실제 최단 경로보다
                        최대 <InlineMath math="\approx 8\%"/>{" "}
                        (최악 방향에서 <InlineMath math="2\cos(\pi/8)/(1+\cos(\pi/4)) - 1"/>) 길다.
                        더 나쁜 것은 <em>보기에도</em> 틀렸다는 점이다. 직선이 명백히 들어가는
                        자리에 긴 지그재그가 놓인다. A* 경로를 사후에 smoothing 하면 낫긴 하지만
                        해결은 아니다. smoother 는 A*가 우연히 돌려준 그 경로 하나만 보는데, 그
                        경로가 장애물의 반대쪽을 돌아갔을 수도 있다.
                    </p>
                    <p>
                        Theta*의 아이디어는 탐색 <em>도중에</em> smoothing 하는 것이다.{" "}
                        <InlineMath math="s"/>를 확장할 때 각 이웃은 부모 후보 둘을 시험한다:
                    </p>
                    <BlockMath math="\text{path 1: } g(s) + c(s, s') \qquad \text{path 2: } g(\text{parent}(s)) + \lVert \text{parent}(s) - s' \rVert \;\;\text{(line-of-sight 일 때)}"/>
                    <p>
                        path 2는 <InlineMath math="s"/>를 건너뛰고 이웃을 확장 노드의 부모에
                        직선으로 잇는다. 이 지름길이 연쇄되면 지그재그 구간 전체가 장애물
                        모서리를 스치는 긴 직선 구간 하나로 접힌다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Any-angle</strong>: path vertices are grid cells, but segments run
                        at arbitrary angles between them.</li>
                    <li><strong>Shorter than grid-optimal</strong> in practice (the sandbox below
                        shows the gap), but <strong>not guaranteed optimal</strong> among all
                        continuous paths — Daniel et al. (2010) give counterexamples; measured
                        suboptimality is a fraction of a percent.</li>
                    <li><strong>Cost</strong>: one A*-shaped search, plus up to one line-of-sight
                        check per generated edge — the checks, not the queue, dominate runtime on
                        large maps (Lazy Theta*'s opening).</li>
                    <li>Heuristic is straight-line Euclidean, consistent with any-angle{" "}
                        <InlineMath math="g"/> values.</li>
                </ul>}
                ko={<ul>
                    <li><strong>Any-angle</strong>: 경로의 꼭짓점은 grid 셀이지만, 그 사이 구간은
                        임의 각도의 직선이다.</li>
                    <li>실전에서 <strong>grid-최적보다 짧다</strong>(아래 sandbox 가 그 차이를
                        보여 준다). 다만 모든 연속 경로 중 <strong>최적이라는 보장은 없다</strong>.
                        Daniel et al.(2010)에 반례가 있고, 실측 준최적성은 1% 미만 수준이다.</li>
                    <li><strong>비용</strong>: A* 꼴 탐색 한 번에, 생성되는 edge 마다 최대 한
                        번의 line-of-sight 검사가 붙는다. 큰 맵에서는 큐가 아니라 이 검사가
                        runtime 을 지배한다 (Lazy Theta* 가 파고드는 지점).</li>
                    <li>heuristic 은 직선 Euclidean 으로, any-angle{" "}
                        <InlineMath math="g"/> 값과 consistent 하다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Structurally this is A* — OPEN/CLOSED, pop-min, relaxation — with each node
                    remembering its parent and the relaxation offered a second, straight-line
                    option (the start is its own parent):
                </p>}
                ko={<p>
                    구조는 A* 그대로다. OPEN/CLOSED, pop-min, relaxation. 각 노드가 부모를
                    기억하고, relaxation 에 직선 선택지가 하나 추가될 뿐이다 (시작 노드의 부모는
                    자기 자신):
                </p>}
            />
            <Pseudocode code={`g[start] ← 0;  parent[start] ← start;  push start with key h(start)
while OPEN is not empty:
    s ← pop_min(OPEN);  move s to CLOSED
    if s = goal:  return follow parent links from goal            # 1
    p ← parent[s]
    for each neighbor s' of s not in CLOSED:
        if line_of_sight(p, s'):                                  # 2
            candidate ← g[p] + euclidean(p, s');   via ← p        # 3
        else:
            candidate ← g[s] + c(s, s');           via ← s        # 4
        if candidate < g[s']:
            g[s'] ← candidate;  parent[s'] ← via
            push s' with key g[s'] + h(s')`}/>
            <T
                en={<ol>
                    <li>Reconstruction follows parent links — but unlike A*, consecutive parents
                        may be many cells apart: each link is a straight segment.</li>
                    <li>One line-of-sight query per generated edge, from the{" "}
                        <em>grandparent</em> <InlineMath math="p"/>, not from{" "}
                        <InlineMath math="s"/>.</li>
                    <li>Path 2: connect straight to the grandparent. When line of sight holds
                        this is never worse than path 1 (proof below), so it is taken without
                        comparing.</li>
                    <li>Path 1: the ordinary grid step — the fallback that keeps the search
                        complete when walls block the view.</li>
                </ol>}
                ko={<ol>
                    <li>재구성은 부모 링크를 따라간다. A*와 달리 연속한 부모가 여러 칸 떨어져
                        있을 수 있고, 각 링크가 곧 직선 구간이다.</li>
                    <li>line-of-sight 질의는 생성되는 edge 마다 한 번,{" "}
                        <InlineMath math="s"/>가 아니라 <em>조부모</em>{" "}
                        <InlineMath math="p"/>에서 쏜다.</li>
                    <li>path 2: 조부모에 직선으로 잇는다. line of sight 가 성립하면 path 1보다
                        나쁠 수 없으므로(아래 논증) 비교 없이 채택한다.</li>
                    <li>path 1: 평범한 grid 스텝. 벽이 시야를 막을 때 탐색의 완전성을 지키는
                        fallback 이다.</li>
                </ol>}
            />
            <Proof title={t("Lemma (the shortcut never hurts)", "보조정리 (지름길은 손해가 없다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Invariant: if{" "}
                            <InlineMath math="\text{parent}(s) = p"/>, then{" "}
                            <InlineMath math="g(s) = g(p) + \lVert p - s \rVert"/> (every parent
                            link is a straight segment). With line of sight from{" "}
                            <InlineMath math="p"/> to <InlineMath math="s'"/>:
                        </p>
                        <BlockMath math="g(p) + \lVert p - s' \rVert \;\overset{\triangle}{\le}\; g(p) + \lVert p - s \rVert + \lVert s - s' \rVert \;=\; g(s) + \lVert s - s' \rVert \;\le\; g(s) + c(s, s')"/>
                        <p>
                            Path 2 is never worse than path 1, so preferring it is safe.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 불변식:{" "}
                            <InlineMath math="\text{parent}(s) = p"/>면{" "}
                            <InlineMath math="g(s) = g(p) + \lVert p - s \rVert"/>다 (모든 부모
                            링크는 직선 구간). <InlineMath math="p"/>에서{" "}
                            <InlineMath math="s'"/>로 line of sight 가 있으면:
                        </p>
                        <BlockMath math="g(p) + \lVert p - s' \rVert \;\overset{\triangle}{\le}\; g(p) + \lVert p - s \rVert + \lVert s - s' \rVert \;=\; g(s) + \lVert s - s' \rVert \;\le\; g(s) + c(s, s')"/>
                        <p>
                            path 2는 path 1보다 나쁠 수 없고, 따라서 비교 없이 택해도 안전하다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    First the primitive: the figure below is the exact line-of-sight test this
                    site's engines and the repository share — a supercover walk over every cell
                    the segment crosses. Then the planner: in the pillar sandbox, toggle the A*
                    overlay and compare the dashed 45°-locked path against Theta*'s straight
                    segments, with both costs in the readout.
                </p>}
                ko={<p>
                    먼저 primitive 부터. 아래 figure 는 이 사이트의 엔진과 저장소가 공유하는
                    바로 그 line-of-sight 검사다. 선분이 지나는 모든 셀을 도는 supercover 순회다.
                    다음은 planner 다. 기둥 sandbox 에서 A* 겹치기를 켜고, 45°에 갇힌 점선
                    경로와 Theta*의 직선 구간을 비교해 보라. 두 비용이 readout 에 함께 나온다.
                </p>}
            />
            <SupercoverLos/>
            <ThetaStarSandbox/>
            <TraceReplay algo="theta_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's Theta* demo — watch parent links jump many cells as shortcuts form",
                "저장소 Theta* demo 의 실제 trace. 지름길이 생기며 부모 링크가 여러 칸을 건너뛰는 것이 보인다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation does not ride the shared best-first core: any-angle needs
                    the Euclidean heuristic and the two-path relaxation. Embedded below in full.
                </p>}
                ko={<p>
                    구현은 공유 best-first 코어를 쓰지 않는다. any-angle 에는 Euclidean
                    heuristic 과 두 갈래 relaxation 이 필요하기 때문이다. 전체를 아래에 embed
                    했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/theta_star.py",
                            code: thetaPy,
                            href: `${REPO}/python/navigation/global_planning/search/theta_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/theta_star.cpp",
                            code: thetaCpp,
                            href: `${REPO}/cpp/src/global_planning/search/theta_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Theta* implementation, embedded from the repository sources",
                    "Theta* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    A. Nash, K. Daniel, S. Koenig, A. Felner,{" "}
                    <a href="https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>Theta*: Any-Angle Path Planning on Grids</em>
                    </a>,
                    AAAI Conference on Artificial Intelligence, 2007.
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
                    J. Amanatides, A. Woo,{" "}
                    <a href="http://www.cse.yorku.ca/~amana/research/grid.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>A Fast Voxel Traversal Algorithm for Ray Tracing</em>
                    </a>,
                    Eurographics, 1987.
                </li>
            </ol>
        </>
    )
}

export default ThetaStar
