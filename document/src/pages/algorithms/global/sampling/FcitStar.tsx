import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import FcitStarSandbox from "../../../../components/panels/global/fcit_star/FcitStarSandbox";
import fcitStarPy from "../../../../../../python/navigation/global_planning/sampling/fcit_star.py?raw";
import fcitStarCpp from "../../../../../../cpp/src/global_planning/sampling/fcit_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const FcitStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    BIT* and AIT* both bound the number of candidate edges by wiring each sample
                    only to neighbors within a shrinking radius{" "}
                    <InlineMath math="r_n"/>. That radius exists to keep collision-checking
                    affordable, but modern collision checks are cheap, and the radius has a cost of
                    its own: two far-apart samples can never be joined directly even when the
                    straight motion between them is free. FCIT* (Wilson et al., 2025)
                    drops the radius. It searches the <em>fully connected</em> graph over the
                    current informed batch — every sample paired with every other — and keeps
                    everything else about AIT*: an obstacle-aware reverse heuristic, a lazily
                    validated forward search, and a persistent set of edges found blocked. Trading
                    a denser candidate graph for the freedom to take long direct shortcuts, it
                    reaches the taut path a radius graph can only approach.
                </p>}
                ko={<p>
                    BIT*와 AIT*는 각 표본을 줄어드는 반경 <InlineMath math="r_n"/> 안의 이웃에만
                    이어 후보 간선 수를 묶는다. 이 반경은 충돌 검사를 감당할 만큼 유지하려고
                    있지만, 요즘 충돌 검사는 싸고 반경은 나름의 대가를 치른다. 멀리 떨어진 두
                    표본은 그 사이 직선 이동이 비어 있어도 직접 이어질 수 없다. FCIT*(Wilson et al.,
                    2025)는 그 반경을 버린다. 현재 informed 배치 위의 <em>완전 연결</em>
                    그래프, 곧 모든 표본을 서로 짝지은 그래프를 탐색하고, AIT*의 나머지는 그대로
                    가져온다. 장애물을 아는 역방향 heuristic, lazy하게 검증하는 전방 탐색, 막힌
                    간선을 쌓는 영속 집합이다. 더 빽빽한 후보 그래프를 긴 직행 지름길을 쓸 자유와
                    맞바꿔, 반경 그래프가 다가가기만 하는 팽팽한 경로에 곧장 닿는다.
                </p>}
            />

            <h2>{t("From a Radius Graph to a Complete Graph", "반경 그래프에서 완전 그래프로")}</h2>
            <T
                en={<>
                    <p>
                        The asymptotically optimal batch planners all share one construction: over
                        the current samples <InlineMath math="V"/> they build a random geometric
                        graph, joining a pair only when it lies within the radius{" "}
                        <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/>. Shrinking that radius
                        with <InlineMath math="n"/> keeps the edge count near-linear, which is what
                        made bounding collision checks tractable in 2015. FCIT* observes that this
                        bound is no longer the binding constraint and replaces the radius graph
                        with the complete graph,
                    </p>
                    <BlockMath math="E \;=\; \bigl\{\, (u, v) : u, v \in V,\; u \neq v \,\bigr\} \;\setminus\; E_{\text{inv}}."/>
                    <Terms items={[
                        ["V", <>the accumulated sample set, index 0 the start and index 1 the goal</>],
                        ["E_{\\text{inv}}", <>the persistent set of edges already found in collision by the forward search, carried across every batch</>],
                        ["E", <>the candidate edge set, <strong>the new term</strong>: every pair of samples except those already known to be blocked — no radius at all</>],
                    ]}/>
                    <p>
                        Everything downstream is AIT* unchanged. A reverse Dijkstra from the goal
                        over <InlineMath math="E"/> supplies an obstacle-aware cost-to-go{" "}
                        <InlineMath math="\hat h"/>, and a forward A* keyed on{" "}
                        <InlineMath math="g + \hat h"/> extends the tree, collision-checking each
                        edge only when it would improve a vertex. Every motion found blocked is
                        added to <InlineMath math="E_{\text{inv}}"/>, so the next batch's reverse
                        search routes around it — the same adaptive feedback loop AIT* introduced.
                    </p>
                    <p>
                        The one structural change carries the whole idea. Because{" "}
                        <InlineMath math="E"/> contains every within-radius pair <em>and</em> every
                        longer pair, a radius graph is a subgraph of it. So any path the radius
                        planners could assemble is still available to FCIT*, which can additionally
                        connect two distant samples in a single edge where a radius graph would
                        need a chain of intermediate ones. In the demo that single
                        edge is the whole solution: the start joins the goal directly, and the
                        first batch already returns the exact straight-line optimum.
                    </p>
                    <p>
                        This repository implements the faithful core and states its simplifications
                        plainly. The reverse search is recomputed from scratch each batch rather
                        than repaired incrementally, and the forward{" "}
                        <InlineMath math="g"/>, parents, and open heap are likewise rebuilt per
                        batch — only <InlineMath math="c_{\text{best}}"/> and{" "}
                        <InlineMath math="E_{\text{inv}}"/> persist. The sample budget is kept
                        modest because the complete graph has <InlineMath math="O(n^2)"/> edges;
                        the paper develops machinery to keep eager all-pairs evaluation cheap at
                        scale that this implementation does not reproduce.
                    </p>
                </>}
                ko={<>
                    <p>
                        점근 최적 batch planner들은 한 가지 구성을 공유한다. 현재 표본{" "}
                        <InlineMath math="V"/> 위에 random geometric graph를 세우되, 두 점이 반경{" "}
                        <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/> 안에 있을 때만 잇는
                        것이다. 이 반경을 <InlineMath math="n"/>에 따라 줄이면 간선 수가 거의
                        선형으로 유지되고, 그것이 2015년에 충돌 검사를 감당 가능하게 만든 열쇠였다.
                        FCIT*는 이 한계가 더는 병목이 아님을 보고, 반경 그래프를 완전 그래프로
                        바꾼다.
                    </p>
                    <BlockMath math="E \;=\; \bigl\{\, (u, v) : u, v \in V,\; u \neq v \,\bigr\} \;\setminus\; E_{\text{inv}}."/>
                    <Terms items={[
                        ["V", <>누적된 표본 집합. index 0이 start, index 1이 goal이다</>],
                        ["E_{\\text{inv}}", <>전방 탐색이 이미 충돌로 밝힌 간선의 영속 집합. 모든 배치를 넘어 유지된다</>],
                        ["E", <>후보 간선 집합. <strong>새로 추가된 항</strong>이며, 이미 막힌 것으로 아는 간선만 뺀 모든 표본 쌍이다. 반경이 전혀 없다</>],
                    ]}/>
                    <p>
                        그 아래는 AIT* 그대로다. goal로부터 <InlineMath math="E"/> 위로 도는 역방향
                        Dijkstra가 장애물을 아는 cost-to-go <InlineMath math="\hat h"/>를 주고,{" "}
                        <InlineMath math="g + \hat h"/>로 키를 잡은 전방 A*가 트리를 넓히며, 정점을
                        개선할 간선만 그 순간 충돌 검사한다. 막힌 이동은 모두{" "}
                        <InlineMath math="E_{\text{inv}}"/>에 더해져, 다음 배치의 역방향 탐색이 그것을
                        우회한다. AIT*가 도입한 적응형 피드백 루프와 같다.
                    </p>
                    <p>
                        단 하나의 구조 변경이 아이디어 전체를 나른다.{" "}
                        <InlineMath math="E"/>가 반경 안의 모든 쌍에 <em>더해</em> 더 먼 모든 쌍까지
                        담으므로, 반경 그래프는 그것의 부분그래프다. 그래서 반경 planner가 짤 수 있던
                        어떤 경로든 FCIT*에서도 여전히 쓸 수 있고, FCIT*는 반경 그래프라면 중간 표본
                        사슬이 필요했을 먼 두 표본을 한 간선으로 더 이을 수 있다. demo에서는
                        그 한 간선이 곧 해 전체다. start가 goal에 직접 이어지고, 첫 배치가 이미 정확한
                        직선 최적을 반환한다.
                    </p>
                    <p>
                        이 저장소는 충실한 코어를 구현하고 단순화를 분명히 밝힌다. 역방향 탐색은
                        배치마다 증분 수리 대신 처음부터 다시 계산하고, 전방{" "}
                        <InlineMath math="g"/>와 parent와 open heap도 배치마다 다시 세운다.{" "}
                        <InlineMath math="c_{\text{best}}"/>와 <InlineMath math="E_{\text{inv}}"/>만
                        남는다. 완전 그래프는 <InlineMath math="O(n^2)"/> 간선이라 표본 예산을 작게
                        유지한다. 논문은 all-pairs 평가를 규모에서도 싸게 유지하는 장치를 전개하지만
                        이 구현은 그것을 재현하지 않는다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Probabilistically complete, anytime, asymptotically optimal</strong>:
                        the complete graph is a superset of the RGG, so every convergence guarantee
                        the radius planners enjoy still holds, and the returned cost is
                        non-increasing across batches.</li>
                    <li><strong>No worse than a radius graph at equal samples</strong>: over the
                        same sample set the complete graph offers a superset of the paths, so
                        FCIT*'s best solution never costs more than a radius planner's — the
                        property the demo measures against AIT*.</li>
                    <li><strong>Takes long shortcuts</strong>: two distant samples can be joined by
                        one edge when the motion is free, so the taut path is found without
                        accumulating the intermediate samples a radius chain would require.</li>
                    <li><strong>Obstacle-aware and adaptive</strong>: inherited from AIT*, the
                        reverse heuristic bends around walls and sharpens as invalid edges
                        accumulate in <InlineMath math="E_{\text{inv}}"/> across batches.</li>
                    <li><strong>Admissible</strong>: the reverse search runs over a graph that
                        still contains every collision-free edge, so <InlineMath math="\hat h"/>
                        never overestimates and the forward A* stays optimal on the validated
                        graph.</li>
                    <li><strong>Cost</strong>: the complete adjacency is{" "}
                        <InlineMath math="O(n^2)"/> edges, versus{" "}
                        <InlineMath math="\Theta(n \log n)"/> for the radius graph — the price of
                        dropping the radius. Per batch this repository rebuilds the adjacency, the
                        reverse Dijkstra, and the forward A* from scratch rather than reusing the
                        previous batch's work, which is why the sample budget stays small.</li>
                </ul>}
                ko={<ul>
                    <li><strong>확률적 완전, anytime, 점근 최적</strong>: 완전 그래프는 RGG의
                        상위집합이므로, 반경 planner가 누리는 수렴 보장이 그대로 성립하고 반환 비용은
                        배치를 거치며 비증가한다.</li>
                    <li><strong>같은 표본에서 반경 그래프보다 나쁘지 않다</strong>: 같은 표본 집합
                        위에서 완전 그래프가 경로의 상위집합을 제공하므로, FCIT*의 최선 해는 반경
                        planner보다 비싸지 않다. demo가 AIT*와 견주어 재는 성질이다.</li>
                    <li><strong>긴 지름길을 쓴다</strong>: 이동이 비어 있으면 먼 두 표본을 한 간선으로
                        이을 수 있어, 반경 사슬이라면 필요했을 중간 표본을 쌓지 않고도 팽팽한 경로를
                        찾는다.</li>
                    <li><strong>장애물 인지와 적응</strong>: AIT*에서 물려받아, 역방향 heuristic이
                        벽을 돌아가고 배치를 거쳐 무효 간선이{" "}
                        <InlineMath math="E_{\text{inv}}"/>에 쌓일수록 날카로워진다.</li>
                    <li><strong>Admissible</strong>: 역방향 탐색은 충돌 없는 간선을 모두 담은 그래프
                        위에서 돌아 <InlineMath math="\hat h"/>가 절대 과대평가하지 않고, 전방 A*가
                        검증된 그래프 위에서 최적을 유지한다.</li>
                    <li><strong>비용</strong>: 완전 인접은 <InlineMath math="O(n^2)"/> 간선으로, 반경
                        그래프의 <InlineMath math="\Theta(n \log n)"/>과 대비된다. 반경을 버린 값이다.
                        이 저장소는 배치마다 인접과 역방향 Dijkstra와 전방 A*를 이전 배치의 일을
                        재사용하지 않고 처음부터 다시 세우므로, 표본 예산을 작게 둔다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Only three things persist across batches: the growing sample array, the
                    incumbent cost <InlineMath math="c_{\text{best}}"/> that sets the informed
                    ellipse, and the invalid-edge set <InlineMath math="E_{\text{inv}}"/>. Each
                    batch draws samples, builds the complete adjacency minus{" "}
                    <InlineMath math="E_{\text{inv}}"/>, runs the reverse Dijkstra for the
                    heuristic, then runs the forward A* that both extends the tree and validates
                    edges. The forward search's discoveries feed the next batch's reverse search.
                </p>}
                ko={<p>
                    배치를 넘어 남는 것은 셋뿐이다. 자라는 표본 배열, informed 타원을 정하는 현직
                    비용 <InlineMath math="c_{\text{best}}"/>, 무효 간선 집합{" "}
                    <InlineMath math="E_{\text{inv}}"/>다. 배치마다 표본을 뽑고,{" "}
                    <InlineMath math="E_{\text{inv}}"/>만 뺀 완전 인접을 세우고, heuristic을 위한
                    역방향 Dijkstra를 돌린 뒤, 트리를 넓히며 간선을 검증하는 전방 A*를 돌린다. 전방
                    탐색의 발견이 다음 배치의 역방향 탐색으로 되먹여진다.
                </p>}
            />
            <Pseudocode code={`V ← [start, goal];  E_inv ← ∅;  c_best ← ∞
for each batch:
    V ← V + m samples (informed ellipse once c_best < ∞)   # 1
    build complete adjacency over V, minus E_inv           # 2
    ĥ ← reverse Dijkstra from goal over that adjacency      # 3
    forward A* from start, key g[v] + ĥ[v]:                # 4
        pop cheapest v; stop this batch when v = goal
        for each other sample x with g[v]+‖v−x‖ < g[x]:     # 5
            if (v, x) ∈ E_inv: continue
            if motion v → x collides:                       # 6
                add (v, x) to E_inv;  continue
            relax g[x] through v
            if x = goal and improved: c_best ← g[goal]      # 7
return best path to goal across all batches`}/>
            <T
                en={<ol>
                    <li>Grow the batch. Before a solution exists the samples are uniform. After,
                        they come from the informed ellipse with foci start and goal and transverse
                        diameter <InlineMath math="c_{\text{best}}"/> (Gammell et al., 2014), the
                        same focusing BIT* and AIT* use.</li>
                    <li>Build the candidate graph. Every pair of samples is a candidate edge,
                        minus those already in <InlineMath math="E_{\text{inv}}"/>. There is no
                        radius and no distance threshold — this single line is the whole departure
                        from AIT*.</li>
                    <li>The reverse search is the heuristic. Dijkstra from the goal over that
                        complete adjacency, with <em>no</em> collision checks.{" "}
                        <InlineMath math="\hat h(x)"/> is the graph cost-to-go, already routed
                        around whatever obstacles have been discovered so far.</li>
                    <li>The forward A*, keyed on <InlineMath math="g + \hat h"/>. It settles each
                        vertex once, in increasing key order, and stops the batch as soon as the
                        goal is settled with its final cost for this batch's graph.</li>
                    <li>A candidate is considered only when routing through{" "}
                        <InlineMath math="v"/> would strictly lower{" "}
                        <InlineMath math="x"/>'s cost-to-come. Edges already known invalid are
                        skipped, the accumulated feedback that keeps the heuristic adaptive.</li>
                    <li>The one lazy validation. The forward search collision-checks each improving
                        edge; an invalid edge is added to the persistent set so the next batch's
                        reverse search routes around it, and <InlineMath math="x"/> waits for a
                        later edge.</li>
                    <li>Reaching the goal with a cheaper cost updates the incumbent, which shrinks
                        the informed ellipse for the next batch's samples. This is the only place{" "}
                        <InlineMath math="c_{\text{best}}"/> moves, and it only moves down.</li>
                </ol>}
                ko={<ol>
                    <li>배치를 키운다. 해가 생기기 전엔 표본이 균일하다. 생긴 뒤엔 시작·goal을
                        초점으로 하고 횡축 지름이 <InlineMath math="c_{\text{best}}"/>인 informed
                        타원에서 나온다(Gammell et al., 2014). BIT*와 AIT*가 쓰는 것과 같은
                        집중이다.</li>
                    <li>후보 그래프를 세운다. 모든 표본 쌍이 후보 간선이고, 이미{" "}
                        <InlineMath math="E_{\text{inv}}"/>에 있는 것만 뺀다. 반경도 거리 임계도 없다.
                        이 한 줄이 AIT*로부터의 유일한 이탈이다.</li>
                    <li>역방향 탐색이 곧 heuristic이다. goal로부터의 Dijkstra를 그 완전 인접 위에서
                        충돌 검사 <em>없이</em> 돌린다. <InlineMath math="\hat h(x)"/>는 지금까지
                        발견된 장애물을 이미 우회한 그래프 cost-to-go다.</li>
                    <li>전방 A*를 <InlineMath math="g + \hat h"/>로 키를 잡는다. 각 정점을 증가 키
                        순으로 한 번씩 확정하고, 이번 배치 그래프에서 goal이 최종 비용으로 확정되는
                        즉시 배치를 끝낸다.</li>
                    <li>후보는 <InlineMath math="v"/>를 거치는 것이{" "}
                        <InlineMath math="x"/>의 cost-to-come을 엄격히 낮출 때만 고려한다. 이미 무효로
                        아는 간선은 건너뛴다. heuristic을 적응형으로 유지하는 누적 피드백이다.</li>
                    <li>단 한 번의 lazy 검증이다. 전방 탐색은 개선하는 간선마다 충돌 검사한다. 무효
                        간선은 영속 집합에 더해져 다음 배치의 역방향 탐색이 그것을 우회하고,{" "}
                        <InlineMath math="x"/>는 뒤의 간선을 기다린다.</li>
                    <li>더 싼 비용으로 goal에 닿으면 현직 해가 갱신되고, 다음 배치 표본을 위한
                        informed 타원이 줄어든다. <InlineMath math="c_{\text{best}}"/>가 움직이는
                        유일한 지점이고, 내려가기만 한다.</li>
                </ol>}
            />
            <Proof title={t(
                "Why FCIT* is never worse than a radius graph at equal samples",
                "같은 표본에서 FCIT*가 반경 그래프보다 나쁘지 않은 이유",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Over one fixed sample set, the cost FCIT*
                            returns is no larger than the cost any radius-graph planner returns from
                            the same samples.
                        </p>
                        <p>
                            <strong>Assumptions.</strong> Let <InlineMath math="V"/> be the samples
                            and let a path be admissible when all its edges are collision-free. The
                            radius planner may use only edges with{" "}
                            <InlineMath math="\lVert u - v \rVert \le r_n"/>; FCIT* may use any
                            pair. Since <InlineMath math="r_n < \infty"/>, every within-radius edge
                            is also an FCIT* edge, so the radius graph is a subgraph of the complete
                            graph and
                        </p>
                        <BlockMath math="\Pi_r(V) \;\subseteq\; \Pi_c(V)."/>
                        <Terms items={[
                            ["\\Pi_r(V)", <>set of collision-free start-to-goal paths using only edges within radius <InlineMath math="r_n"/></>],
                            ["\\Pi_c(V)", <>set of collision-free start-to-goal paths over the complete graph, <strong>the new term</strong>, which contains every radius path and additionally every path using a longer direct edge</>],
                        ]}/>
                        <p>
                            because keeping more edges can only add paths, never remove one.
                            Minimizing a cost over a superset can only lower the minimum, so
                        </p>
                        <BlockMath math="c_c(V) \;=\; \min_{\pi \in \Pi_c(V)} \operatorname{cost}(\pi) \;\le\; \min_{\pi \in \Pi_r(V)} \operatorname{cost}(\pi) \;=\; c_r(V)."/>
                        <Terms items={[
                            ["c_c(V)", <>the best path cost FCIT* can return over the complete graph on samples <InlineMath math="V"/></>],
                            ["c_r(V)", <>the best path cost a radius-graph planner can return over the same samples</>],
                            ["\\operatorname{cost}(\\pi)", <>summed Euclidean length of the edges of path <InlineMath math="\pi"/></>],
                        ]}/>
                        <p>
                            <strong>Conclusion.</strong> <InlineMath math="c_c \le c_r"/> at every
                            sample count. The gap is exactly the shortcuts — direct edges longer
                            than <InlineMath math="r_n"/> — that the complete graph admits and the
                            radius graph forbids. When start and goal sit in one
                            connected free region, so the single edge between them is collision-free
                            and <InlineMath math="c_c"/> equals the straight-line distance from the
                            first batch, while <InlineMath math="c_r"/> must chain intermediate
                            samples and only descends toward it as <InlineMath math="n"/> grows.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 하나의 고정된 표본 집합 위에서 FCIT*가 반환하는
                            비용은, 같은 표본으로 어떤 반경 그래프 planner가 반환하는 비용보다 크지
                            않다.
                        </p>
                        <p>
                            <strong>가정.</strong> <InlineMath math="V"/>를 표본이라 하고, 간선이 모두
                            충돌 없을 때 그 경로를 admissible이라 하자. 반경 planner는{" "}
                            <InlineMath math="\lVert u - v \rVert \le r_n"/>인 간선만 쓸 수 있고,
                            FCIT*는 어떤 쌍이든 쓸 수 있다. <InlineMath math="r_n < \infty"/>이므로
                            반경 안의 간선은 모두 FCIT*의 간선이기도 하다. 곧 반경 그래프는 완전
                            그래프의 부분그래프이고
                        </p>
                        <BlockMath math="\Pi_r(V) \;\subseteq\; \Pi_c(V)."/>
                        <Terms items={[
                            ["\\Pi_r(V)", <>반경 <InlineMath math="r_n"/> 안의 간선만 쓴 충돌 없는 start–goal 경로 집합</>],
                            ["\\Pi_c(V)", <>완전 그래프 위 충돌 없는 start–goal 경로 집합. <strong>새로 추가된 항</strong>이며, 모든 반경 경로에 더해 더 긴 직행 간선을 쓴 경로까지 담는다</>],
                        ]}/>
                        <p>
                            간선을 더 남기는 것은 경로를 더할 뿐 지울 수 없기 때문이다. 비용을
                            상위집합 위에서 최소화하면 최소값은 낮아질 수만 있으므로
                        </p>
                        <BlockMath math="c_c(V) \;=\; \min_{\pi \in \Pi_c(V)} \operatorname{cost}(\pi) \;\le\; \min_{\pi \in \Pi_r(V)} \operatorname{cost}(\pi) \;=\; c_r(V)."/>
                        <Terms items={[
                            ["c_c(V)", <>표본 <InlineMath math="V"/> 위 완전 그래프에서 FCIT*가 반환할 수 있는 최선 경로 비용</>],
                            ["c_r(V)", <>같은 표본 위 반경 그래프 planner가 반환할 수 있는 최선 경로 비용</>],
                            ["\\operatorname{cost}(\\pi)", <>경로 <InlineMath math="\pi"/> 간선들의 유클리드 길이 합</>],
                        ]}/>
                        <p>
                            <strong>결론.</strong> 어느 표본 수에서든{" "}
                            <InlineMath math="c_c \le c_r"/>이다. 그 격차는 완전 그래프가 허용하고
                            반경 그래프가 막는 지름길, 곧 <InlineMath math="r_n"/>보다 긴 직행
                            간선이다. start와 goal이 한 연결된 자유 영역에 있으면 둘
                            사이의 단 한 간선이 충돌 없고, <InlineMath math="c_c"/>는 첫 배치부터
                            직선거리와 같다. 반면 <InlineMath math="c_r"/>는 중간 표본을 사슬로
                            이어야 하고 <InlineMath math="n"/>이 커져야 그쪽으로 내려간다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs FCIT* between staggered blocks, where the optimum threads the
                    straight start-to-goal line. FCIT*'s fully connected graph contains that direct
                    edge, so the first batch already returns the exact optimum and every later batch
                    holds it. Alongside, AIT* runs on the same sample budget — its shrinking radius
                    cannot span the gap in one edge, so it connects a bent chain and only approaches
                    the straight line as batches accumulate. Paint a wall to block the direct line
                    and watch FCIT* take the longest free shortcuts around it. The replay below is
                    the repository demo on the benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 엇갈린 블록 사이에서 FCIT*를 돌린다. 최적 경로는 블록 사이 긴 자유 구간을 꿰는 직선들이다.
                    FCIT*의 완전 연결 그래프는 그 직행 간선을 담으므로, 첫 배치가 이미 정확한 최적을
                    반환하고 이후 배치는 그것을 유지한다. 옆에서는 같은 표본 예산의 AIT*가 도는데,
                    줄어드는 반경으로는 그 거리를 한 간선에 잇지 못해 꺾인 사슬로 잇고 배치가 쌓여야
                    직선에 다가간다. 벽을 칠해 직선을 막으면 FCIT*가 그 둘레로 가장 긴 자유 지름길을
                    잡는 것을 볼 수 있다. 아래 replay는 벤치마크 맵 위의 저장소 demo다.
                </p>}
            />
            <FcitStarSandbox/>
            <TraceReplay algo="fcit_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's FCIT* demo: each batch draws informed samples, a reverse search over the fully connected graph builds the heuristic, and the forward search connects long direct edges while validating them lazily",
                "저장소 FCIT* demo의 실제 trace. 배치마다 informed 표본을 뽑고, 완전 연결 그래프 위 역방향 탐색이 heuristic을 세우며, 전방 탐색이 긴 직행 간선을 lazy하게 검증하면서 잇는다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The informed sampler is shared with the other batch planners; FCIT* itself is
                    the complete adjacency, the reverse Dijkstra, and the forward A* with its lazy
                    validation and the persistent invalid-edge set. Embedded below in full.
                </p>}
                ko={<p>
                    informed 표본기는 다른 batch planner들과 공유한다. FCIT* 자체는 완전 인접과
                    역방향 Dijkstra, 그리고 lazy 검증 및 영속 무효 간선 집합을 갖는 전방 A*다. 전체를
                    아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/fcit_star.py",
                            code: fcitStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/fcit_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/fcit_star.cpp",
                            code: fcitStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/fcit_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete FCIT* implementation, embedded from the repository sources",
                    "FCIT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    T. S. Wilson, W. Thomason, Z. Kingston, L. E. Kavraki, J. D. Gammell,{" "}
                    <a href="https://arxiv.org/abs/2411.17902" target="_blank"
                       rel="noopener noreferrer">
                        <em>Nearest-Neighbourless Asymptotically Optimal Motion Planning with Fully
                            Connected Informed Trees (FCIT*)</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2025.
                </li>
                <li>
                    M. P. Strub, J. D. Gammell,{" "}
                    <a href="https://doi.org/10.1109/ICRA40945.2020.9197338" target="_blank"
                       rel="noopener noreferrer">
                        <em>Adaptively Informed Trees (AIT*): Fast Asymptotically Optimal Path
                            Planning through Adaptive Heuristics</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2020.
                </li>
                <li>
                    J. D. Gammell, T. D. Barfoot, S. S. Srinivasa,{" "}
                    <a href="https://doi.org/10.1177/0278364919890396" target="_blank"
                       rel="noopener noreferrer">
                        <em>Batch Informed Trees (BIT*): Informed Asymptotically Optimal Anytime
                            Search</em>
                    </a>,
                    The International Journal of Robotics Research, 2020.
                </li>
                <li>
                    J. D. Gammell, S. S. Srinivasa, T. D. Barfoot,{" "}
                    <a href="https://doi.org/10.1109/IROS.2014.6942976" target="_blank"
                       rel="noopener noreferrer">
                        <em>Informed RRT*: Optimal Sampling-based Path Planning Focused via
                            Direct Sampling of an Admissible Ellipsoidal Heuristic</em>
                    </a>,
                    IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2014.
                </li>
            </ol>
        </>
    )
}

export default FcitStar
