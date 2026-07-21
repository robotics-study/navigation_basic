import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import RrtStarSandbox from "../../../../components/panels/global/rrt_star/RrtStarSandbox";
import rrtStarPy from "../../../../../../python/navigation/global_planning/sampling/rrt_star.py?raw";
import rrtStarCpp from "../../../../../../cpp/src/global_planning/sampling/rrt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const RrtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    RRT finds a path and freezes: whatever jagged route the first lucky
                    branch took is the answer forever. RRT* (Karaman &amp; Frazzoli, 2011)
                    keeps the same growth loop but refuses to commit — every new node first{" "}
                    <em>shops</em> for the cheapest parent in its neighborhood, then offers
                    itself as a cheaper parent to everyone nearby. Two local edits, applied
                    relentlessly, give a global guarantee: the tree's paths converge to the
                    optimum almost surely. This is the page where sampling planners stop
                    being merely feasible and start being optimal.
                </p>}
                ko={<p>
                    RRT는 경로를 찾으면 얼어붙는다. 운 좋은 첫 가지가 지나간 삐죽한 길이
                    영원히 답이다. RRT*(Karaman &amp; Frazzoli, 2011)는 같은 성장 루프를
                    유지하되 확정을 거부한다. 새 노드마다 먼저 근방에서 가장 싼 부모를{" "}
                    <em>골라잡고</em>, 그다음 자신이 더 싼 부모가 되어 주변에 자신을
                    제안한다. 국소 수선 둘을 집요하게 반복한 결과가 전역 보장이다. 트리의
                    경로가 거의 확실히 최적으로 수렴한다. sampling planner가 "되기만 하는"
                    수준을 벗어나 최적이 되는 페이지다.
                </p>}
            />

            <h2>{t("Choose Parent, Then Rewire", "부모를 고르고, 다시 배선한다")}</h2>
            <T
                en={<>
                    <p>
                        After the usual sample–nearest–steer step produces{" "}
                        <InlineMath math="q_{\text{new}}"/>, RRT* looks at every tree node
                        within a neighborhood radius and runs two symmetric passes:
                    </p>
                    <BlockMath math="\text{parent}(q_{\text{new}}) = \arg\min_{v \in N} \bigl(\mathrm{cost}(v) + \lVert v - q_{\text{new}} \rVert\bigr) \qquad \mathrm{cost}(u) \leftarrow \mathrm{cost}(q_{\text{new}}) + \lVert q_{\text{new}} - u \rVert \;\;\text{if cheaper}"/>
                    <Terms items={[
                        ["q_{\\text{new}}", "the freshly steered node being inserted"],
                        ["N", <>the neighborhood: tree nodes within the near radius of <InlineMath math="q_{\text{new}}"/> (feasible motions only)</>],
                        ["\\mathrm{cost}(v)", <>accumulated path cost from the start to <InlineMath math="v"/> through the tree</>],
                        ["u \\in N", <><strong>the new move</strong> (rewire): any neighbor that gets cheaper by routing through <InlineMath math="q_{\text{new}}"/> is re-parented to it — and the saving propagates down its whole subtree</>],
                    ]}/>
                    <p>
                        Choose-parent makes the new node arrive as cheaply as possible;
                        rewire lets its arrival repair history. Neither pass looks beyond
                        the neighborhood, yet their accumulation straightens whole routes —
                        watch the sandbox path pull taut as the budget grows.
                    </p>
                </>}
                ko={<>
                    <p>
                        평소의 sample–nearest–steer가 <InlineMath math="q_{\text{new}}"/>를
                        만들고 나면, RRT*은 근방 반경 안의 트리 노드 전부를 보고 대칭적인
                        패스 둘을 돌린다:
                    </p>
                    <BlockMath math="\text{parent}(q_{\text{new}}) = \arg\min_{v \in N} \bigl(\mathrm{cost}(v) + \lVert v - q_{\text{new}} \rVert\bigr) \qquad \mathrm{cost}(u) \leftarrow \mathrm{cost}(q_{\text{new}}) + \lVert q_{\text{new}} - u \rVert \;\;\text{(더 싸지면)}"/>
                    <Terms items={[
                        ["q_{\\text{new}}", "방금 steer 되어 삽입되는 새 노드"],
                        ["N", <>근방. <InlineMath math="q_{\text{new}}"/>의 near 반경 안 트리 노드들 (이동이 실행 가능한 것만)</>],
                        ["\\mathrm{cost}(v)", <>트리를 따라 시작→<InlineMath math="v"/>까지 누적된 경로 비용</>],
                        ["u \\in N", <><strong>새로 추가된 수</strong> (rewire): <InlineMath math="q_{\text{new}}"/>를 거치는 편이 싸지는 이웃은 그쪽으로 부모를 갈아탄다. 절약분은 그 부분 트리 전체에 전파된다</>],
                    ]}/>
                    <p>
                        choose-parent는 새 노드를 최대한 싸게 도착시키고, rewire는 그
                        도착이 과거를 수리하게 한다. 두 패스 모두 근방 밖을 보지 않지만,
                        그 누적이 경로 전체를 편다. 예산을 키울수록 sandbox의 경로가
                        팽팽해지는 것을 보라.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Asymptotically optimal</strong>: path cost converges to the
                        optimum almost surely — provided the near radius shrinks no faster
                        than the RGG rate (or stays a sufficiently large constant, as this
                        repository defaults to on its small maps).</li>
                    <li><strong>Anytime</strong>: the incumbent goal path improves whenever
                        a cheaper connection appears; the player's solution counter shows
                        each published improvement.</li>
                    <li><strong>Probabilistically complete</strong>, exactly as RRT.</li>
                    <li><strong>Cost</strong>: per iteration, one nearest query plus a
                        near-set scan and up to <InlineMath math="|N|"/> extra collision
                        checks for choose-parent and rewire — several times RRT's work per
                        sample, traded for path quality.</li>
                    <li><strong>Goal handling</strong>: the goal never enters the tree (it
                        must not become a rewire candidate); only its best parent and cost
                        are tracked.</li>
                </ul>}
                ko={<ul>
                    <li><strong>점근 최적</strong>: near 반경이 RGG 속도보다 빨리 줄지
                        않는 한 (또는 이 저장소의 작은 맵 기본값처럼 충분히 큰 상수인 한)
                        경로 비용이 거의 확실히 최적으로 수렴한다.</li>
                    <li><strong>Anytime</strong>: 더 싼 연결이 나타날 때마다 goal 경로가
                        갱신된다. 플레이어의 해 카운터가 발표된 개선을 하나씩 보여 준다.</li>
                    <li>RRT와 똑같이 <strong>확률적 완전</strong>.</li>
                    <li><strong>비용</strong>: 반복마다 nearest 한 번에 near 집합 스캔,
                        그리고 choose-parent와 rewire 몫으로 최대 <InlineMath math="|N|"/>
                        회의 추가 충돌 검사가 붙는다. 표본당 일이 RRT의 몇 배지만 경로
                        품질과 맞바꾼 값이다.</li>
                    <li><strong>goal 처리</strong>: goal은 트리에 넣지 않는다 (rewire
                        후보가 되면 안 된다). 최선 부모와 비용만 추적한다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    RRT's loop with two inserted passes and an incumbent instead of an
                    early return:
                </p>}
                ko={<p>
                    RRT의 루프에 패스 둘이 끼어들고, 조기 반환 대신 현직 해를 유지한다:
                </p>}
            />
            <Pseudocode code={`tree ← {start};  best ← ∞
repeat max_iterations times:
    q_rand ← goal with probability p, else uniform sample
    q_new ← steer(nearest(q_rand), q_rand, η);  skip if blocked
    N ← tree nodes within near radius of q_new                # 1
    parent ← argmin over feasible v ∈ N of cost(v) + ‖v−q_new‖  # 2
    add q_new under parent
    for u ∈ N:                                                # 3
        if cost(q_new) + ‖q_new−u‖ < cost(u) and motion free:
            re-parent u under q_new;  propagate to u's subtree
    if q_new reaches goal region and beats best:              # 4
        best ← that cost;  publish improved path`}/>
            <T
                en={<ol>
                    <li>The neighborhood is the interaction range of both passes — fixed
                        radius here, or the shrinking RGG radius from the PRM* page.</li>
                    <li>Choose-parent: the nearest node found the sample, but a farther
                        neighbor may offer a cheaper route through itself.</li>
                    <li>Rewire: the mirror image. Skipping the propagation to descendant
                        costs is the classic implementation bug — stale subtree costs
                        silently break the optimality invariant.</li>
                    <li>No early return: the loop runs its full budget, and reaching the
                        goal merely updates the incumbent.</li>
                </ol>}
                ko={<ol>
                    <li>근방이 두 패스의 상호작용 범위다. 여기서는 고정 반경이고, PRM*
                        페이지의 줄어드는 RGG 반경을 쓸 수도 있다.</li>
                    <li>choose-parent: 표본을 찾은 것은 nearest 지만, 더 먼 이웃이 자기를
                        거치는 더 싼 길을 제안할 수 있다.</li>
                    <li>rewire: 그 거울상이다. 후손 비용 전파를 빼먹는 것이 고전적인 구현
                        버그다. 낡은 부분 트리 비용이 최적성 불변식을 조용히 깨뜨린다.</li>
                    <li>조기 반환이 없다. 루프는 예산을 다 쓰고, goal 도달은 현직 해를
                        갱신할 뿐이다.</li>
                </ol>}
            />
            <Proof title={t("Why rewiring must reach the subtree", "rewire가 부분 트리까지 내려가야 하는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Re-parenting <InlineMath math="u"/>{" "}
                            changes its cost by{" "}
                            <InlineMath math="\Delta = \mathrm{cost}_{\text{new}}(u) - \mathrm{cost}_{\text{old}}(u) < 0"/>.
                            Every descendant <InlineMath math="w"/> reaches the start
                            through <InlineMath math="u"/>, so its true cost is
                        </p>
                        <BlockMath math="\mathrm{cost}(w) \;=\; \mathrm{cost}(u) + \mathrm{len}(u \leadsto w) \;\Rightarrow\; \mathrm{cost}(w) \text{ shifts by the same } \Delta"/>
                        <Terms items={[
                            ["u", "the neighbor that just got a cheaper parent"],
                            ["w", <>any descendant of <InlineMath math="u"/> in the tree</>],
                            ["\\mathrm{len}(u \\leadsto w)", <>tree-path length from <InlineMath math="u"/> down to <InlineMath math="w"/> — unchanged by the re-parenting</>],
                            ["\\Delta", "the (negative) cost change at u"],
                        ]}/>
                        <p>
                            Leave the descendants un-updated and every later choose-parent
                            or rewire comparison against them uses an overestimate —
                            cheaper routes get rejected, and the convergence argument no
                            longer applies. The repository's tree therefore stores children
                            lists purely so this propagation can walk the subtree.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> <InlineMath math="u"/>의 부모를 갈아타면
                            비용이{" "}
                            <InlineMath math="\Delta = \mathrm{cost}_{\text{new}}(u) - \mathrm{cost}_{\text{old}}(u) < 0"/>만큼
                            변한다. 모든 후손 <InlineMath math="w"/>는{" "}
                            <InlineMath math="u"/>를 거쳐 시작에 닿으므로
                        </p>
                        <BlockMath math="\mathrm{cost}(w) \;=\; \mathrm{cost}(u) + \mathrm{len}(u \leadsto w) \;\Rightarrow\; \mathrm{cost}(w) \text{도 같은 } \Delta \text{만큼 이동}"/>
                        <Terms items={[
                            ["u", "방금 더 싼 부모를 얻은 이웃"],
                            ["w", <>트리에서 <InlineMath math="u"/>의 아무 후손</>],
                            ["\\mathrm{len}(u \\leadsto w)", <><InlineMath math="u"/>에서 <InlineMath math="w"/>까지의 트리 경로 길이. 부모 교체로 변하지 않는다</>],
                            ["\\Delta", "u에서의 (음수) 비용 변화"],
                        ]}/>
                        <p>
                            후손을 갱신하지 않으면 이후의 모든 choose-parent·rewire 비교가
                            과대평가된 값을 쓴다. 더 싼 길이 기각되고, 수렴 논증이 더는
                            성립하지 않는다. 저장소의 트리가 children 목록을 들고 있는
                            이유가 오직 이 전파를 위해서다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox is nearly empty on purpose: the optimum is a taut diagonal
                    grazing one block, so convergence is visible to the eye. Raise the
                    budget from 800 to 5000 iterations and the red path tightens toward
                    the straight-line bound while the dashed RRT answer never moves; the
                    solution counter ticks with each published improvement. The replay
                    below is the repository demo — rewiring continuously reshapes the tree
                    even after the first path appears.
                </p>}
                ko={<p>
                    sandbox는 일부러 거의 비어 있다. 최적이 블록 하나를 스치는 팽팽한
                    대각선이라 수렴이 눈에 보인다. 예산을 800에서 5000 반복으로 올리면
                    빨간 경로가 직선 하한 쪽으로 조여지는데, 점선 RRT의 답은 꿈쩍하지
                    않는다. 해 카운터가 발표된 개선을 하나씩 센다. 아래 replay는 저장소
                    demo다. 첫 경로가 나온 뒤에도 rewire가 트리를 끊임없이 고쳐 짓는다.
                </p>}
            />
            <RrtStarSandbox/>
            <TraceReplay algo="rrt_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's RRT* demo — the tree keeps rewiring after the first solution, and the published path improves in steps",
                "저장소 RRT* demo의 실제 trace. 첫 해 이후에도 트리가 계속 rewire 되고, 발표되는 경로가 계단식으로 좋아진다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    Choose-parent and rewire live in the shared sampling module (Informed
                    RRT* and Fast-RRT reuse them unchanged); the planner file is the loop
                    plus incumbent tracking. Embedded below in full.
                </p>}
                ko={<p>
                    choose-parent와 rewire는 공유 sampling 모듈에 있다 (Informed RRT*과
                    Fast-RRT가 그대로 재사용한다). planner 파일은 루프에 현직 해 추적을
                    더한 것이다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/rrt_star.py",
                            code: rrtStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/rrt_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/rrt_star.cpp",
                            code: rrtStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/rrt_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete RRT* implementation, embedded from the repository sources",
                    "RRT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank"
                       rel="noopener noreferrer">
                        <em>Sampling-based Algorithms for Optimal Motion Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2011.
                </li>
                <li>
                    S. M. LaValle,{" "}
                    <a href="https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Rapidly-Exploring Random Trees: A New Tool for Path Planning</em>
                    </a>,
                    Technical Report TR 98-11, Iowa State University, 1998.
                </li>
            </ol>
        </>
    )
}

export default RrtStar
