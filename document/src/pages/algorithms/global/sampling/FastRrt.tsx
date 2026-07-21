import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import FastRrtSandbox from "../../../../components/panels/global/fast_rrt/FastRrtSandbox";
import fastRrtPy from "../../../../../../python/navigation/global_planning/sampling/fast_rrt.py?raw";
import fastRrtCpp from "../../../../../../cpp/src/global_planning/sampling/fast_rrt.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const FastRrt = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    RRT* converges to the optimum, but it spends its budget everywhere:
                    it keeps sampling regions the tree already blankets, and a blocked
                    straight extension simply throws the iteration away. Fast-RRT (Wu et
                    al., 2021) keeps the RRT* tree and its two rewiring passes, then bolts
                    on three accelerators that attack exactly that waste. It samples only
                    where the tree has not reached, it rescues a blocked extension with a
                    random probe instead of discarding it, and once any feasible path
                    exists it shortcuts the path taut with the triangle inequality. Same
                    optimality target, far less motion spent to approach it.
                </p>}
                ko={<p>
                    RRT*은 최적으로 수렴하지만 예산을 사방에 흩뿌린다. 트리가 이미 덮은
                    영역을 계속 표본으로 뽑고, 직선 확장이 막히면 그 반복을 그냥 버린다.
                    Fast-RRT(Wu et al., 2021)는 RRT* 트리와 두 rewire 패스를 그대로 두고,
                    바로 그 낭비를 겨냥한 가속기 셋을 덧댄다. 트리가 아직 닿지 않은 곳만
                    표본으로 뽑고, 막힌 확장은 버리는 대신 무작위 탐침으로 살려 내며,
                    실행 가능한 경로가 하나라도 생기면 삼각 부등식으로 그 경로를 팽팽하게
                    당긴다. 최적이라는 목표는 같되, 거기 다가가는 데 드는 움직임이 훨씬
                    적다.
                </p>}
            />

            <h2>{t("Three Accelerations on the RRT* Tree", "RRT* 트리에 얹는 가속 셋")}</h2>
            <T
                en={<>
                    <p>
                        The backbone is unchanged RRT*: sample, steer toward the sample from
                        the nearest node, choose the cheapest feasible parent in the
                        neighborhood, and rewire neighbors that get cheaper through the new
                        node. Fast-RRT inserts three changes.
                    </p>
                    <p>
                        <strong>Fast-Sampling.</strong> A free draw is rejected while it
                        lands inside any existing node's reached radius, so samples
                        concentrate on unreached space:
                    </p>
                    <BlockMath math="\text{accept } q \iff q = \text{goal (biased draw)} \;\;\text{or}\;\; \min_{v \in \text{tree}} \lVert q - v \rVert > r_{\text{reached}}"/>
                    <Terms items={[
                        ["q", "the candidate random sample being tested for acceptance"],
                        ["v", "a node already in the search tree"],
                        ["\\lVert q - v \\rVert", "Euclidean distance from the sample to that node"],
                        ["r_{\\text{reached}}", <><strong>the new term</strong>: the reached radius. A free sample within this distance of any node is deemed already explored and redrawn (up to a bounded number of retries)</>],
                        ["\\text{goal (biased draw)}", <>with probability <InlineMath math="p"/> the sample is the goal itself, and that draw is always accepted so the tree can still reach the goal</>],
                    ]}/>
                    <p>
                        <strong>Random Steering.</strong> When the straight step toward the
                        sample is blocked, rather than discard the iteration, probe random
                        directions and take the first collision-free one:
                    </p>
                    <BlockMath math="q_{\text{new}} = q_{\text{near}} + \eta\,(\cos\theta,\ \sin\theta), \qquad \theta \sim \mathrm{Uniform}(0, 2\pi)"/>
                    <Terms items={[
                        ["q_{\\text{new}}", "the candidate new node produced by a random probe"],
                        ["q_{\\text{near}}", <>the tree node nearest to the sample, from which the extension is attempted</>],
                        ["\\eta", "the step size: the fixed extension distance of one steer"],
                        ["\\theta", <><strong>the new term</strong>: a random heading drawn uniformly in <InlineMath math="[0, 2\pi)"/>, retried up to a bounded count until the step is collision-free</>],
                    ]}/>
                    <p>
                        <strong>Fast-Optimal.</strong> Once a node reaches the goal region,
                        the extracted path is shortcut by the triangle inequality: from each
                        kept waypoint, jump to the farthest later waypoint still reachable by
                        a collision-free straight segment. The cheapest shortcut path found
                        so far is the incumbent, improved anytime just as in RRT*.
                    </p>
                </>}
                ko={<>
                    <p>
                        골격은 바뀌지 않은 RRT*이다. 표본을 뽑고, nearest 노드에서 표본
                        쪽으로 steer 하고, 근방에서 가장 싼 실행 가능한 부모를 고르고, 새
                        노드를 거쳐 싸지는 이웃을 rewire 한다. Fast-RRT는 여기에 변경 셋을
                        끼워 넣는다.
                    </p>
                    <p>
                        <strong>Fast-Sampling.</strong> 자유 표본은 기존 노드의 reached
                        반경 안에 떨어지는 동안 거부되어, 표본이 트리가 아직 닿지 않은
                        공간에 몰린다.
                    </p>
                    <BlockMath math="\text{accept } q \iff q = \text{goal (biased draw)} \;\;\text{or}\;\; \min_{v \in \text{tree}} \lVert q - v \rVert > r_{\text{reached}}"/>
                    <Terms items={[
                        ["q", "받아들일지 검사 중인 후보 무작위 표본"],
                        ["v", "이미 탐색 트리에 있는 노드"],
                        ["\\lVert q - v \\rVert", "표본에서 그 노드까지의 유클리드 거리"],
                        ["r_{\\text{reached}}", <><strong>새로 추가된 항</strong>: reached 반경. 어느 노드로부터든 이 거리 안에 든 자유 표본은 이미 탐사된 것으로 보고 다시 뽑는다 (재시도 횟수는 상한이 있다)</>],
                        ["\\text{goal (biased draw)}", <>확률 <InlineMath math="p"/>로 표본이 goal 자체가 되며, 이 draw는 트리가 goal에 닿을 수 있도록 항상 받아들인다</>],
                    ]}/>
                    <p>
                        <strong>Random Steering.</strong> 표본 쪽 직선 스텝이 막히면 반복을
                        버리는 대신 무작위 방향을 탐침해 첫 충돌 없는 방향을 취한다.
                    </p>
                    <BlockMath math="q_{\text{new}} = q_{\text{near}} + \eta\,(\cos\theta,\ \sin\theta), \qquad \theta \sim \mathrm{Uniform}(0, 2\pi)"/>
                    <Terms items={[
                        ["q_{\\text{new}}", "무작위 탐침이 만든 후보 새 노드"],
                        ["q_{\\text{near}}", <>표본에 가장 가까운 트리 노드. 확장을 시도하는 출발점이다</>],
                        ["\\eta", "step size. steer 한 번의 고정 확장 거리"],
                        ["\\theta", <><strong>새로 추가된 항</strong>: <InlineMath math="[0, 2\pi)"/>에서 균일하게 뽑는 무작위 방향. 스텝이 충돌 없을 때까지 상한 횟수만큼 재시도한다</>],
                    ]}/>
                    <p>
                        <strong>Fast-Optimal.</strong> 노드가 goal 영역에 닿으면 추출된
                        경로를 삼각 부등식으로 지름길화한다. 유지된 각 waypoint에서, 충돌
                        없는 직선 구간으로 아직 닿는 가장 먼 이후 waypoint로 뛴다. 지금까지
                        찾은 가장 싼 지름길 경로가 현직 해이고, RRT*처럼 anytime 으로
                        개선된다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Feasibility, not a new optimality proof</strong>: the three
                        accelerators change <em>where</em> and <em>how fast</em> the tree
                        explores, not the RRT* rewiring that carries the asymptotic
                        guarantee. Fast-Sampling and Random Steering are heuristics that cut
                        search-time variance, especially through narrow passages.</li>
                    <li><strong>Divergence from the paper</strong>: Wu et al. re-initialise a
                        plain RRT per outer iteration and their Fast-Optimal <em>fuses
                        multiple paths</em> at crossing points; this repository keeps one
                        persistent RRT* tree and replaces the fusion with a single-path
                        triangle-inequality shortcut — same intent, simpler mechanism.</li>
                    <li><strong>Anytime</strong>: the incumbent is the cheapest shortcut path
                        seen so far and only improves; the loop always runs its full
                        budget.</li>
                    <li><strong>Probabilistically complete</strong>, as RRT — biased goal
                        draws are always accepted, so the goal stays reachable.</li>
                    <li><strong>Shortcut is a projection down</strong>: replacing a subpath
                        by a straight chord never lengthens it, so Fast-Optimal never makes a
                        path worse (proof below).</li>
                    <li><strong>Cost</strong>: per iteration, RRT*'s nearest query, near-set
                        scan, and up to <InlineMath math="|N|"/> collision checks, plus the
                        Fast-Sampling rejection loop and, when blocked, the Random Steering
                        probes — both bounded by the retry cap.</li>
                    <li><strong>Goal handling</strong>: the goal never enters the tree, so it
                        is never a nearest, near, or rewire candidate; only the incumbent
                        path and its cost are tracked.</li>
                </ul>}
                ko={<ul>
                    <li><strong>가속이지 새 최적성 증명이 아니다</strong>: 가속기 셋은
                        트리가 <em>어디를</em>, <em>얼마나 빨리</em> 탐색하는지를 바꿀 뿐,
                        점근 보장을 지는 RRT* rewire는 건드리지 않는다. Fast-Sampling 과
                        Random Steering은 search-time variance를 줄이는 휴리스틱이고,
                        특히 좁은 통로에서 효과가 크다.</li>
                    <li><strong>논문과의 편차</strong>: Wu et al.은 외곽 루프마다 plain RRT를
                        재초기화하고 Fast-Optimal이 <em>여러 경로를 교차점에서 융합</em>하지만,
                        이 저장소는 지속되는 RRT* 트리 하나에 단일 경로 삼각부등식 shortcut을
                        얹는다. 의도는 같고 기전은 더 단순하다.</li>
                    <li><strong>Anytime</strong>: 현직 해는 지금까지 본 가장 싼 지름길
                        경로이고 개선만 된다. 루프는 언제나 예산을 다 쓴다.</li>
                    <li>RRT처럼 <strong>확률적 완전</strong>. goal biased draw는 항상
                        받아들여지므로 goal이 도달 가능하게 유지된다.</li>
                    <li><strong>지름길은 아래로의 사영</strong>: 부분 경로를 직선 현으로
                        바꾸면 길이가 늘지 않으므로, Fast-Optimal은 경로를 더 나쁘게 만들지
                        않는다 (아래 증명).</li>
                    <li><strong>비용</strong>: 반복마다 RRT*의 nearest 질의, near 집합 스캔,
                        최대 <InlineMath math="|N|"/>회 충돌 검사에, Fast-Sampling 거부 루프,
                        그리고 막혔을 때 Random Steering 탐침이 더해진다. 둘 다 재시도
                        상한으로 묶인다.</li>
                    <li><strong>goal 처리</strong>: goal은 트리에 들어가지 않으므로
                        nearest·near·rewire 후보가 되지 않는다. 현직 경로와 그 비용만
                        추적한다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The RRT* loop with sampling and extension replaced by their Fast-RRT
                    variants, and the goal test followed by a shortcut prune:
                </p>}
                ko={<p>
                    RRT* 루프에서 sampling과 extension을 Fast-RRT 변형으로 갈아 끼우고,
                    goal 검사 뒤에 지름길 pruning을 붙인다:
                </p>}
            />
            <Pseudocode code={`tree ← {start};  best ← ∞
repeat max_iterations times:
    q_rand ← goal w.p. p, else resample until outside every node's r_reached   # 1
    q_new  ← steer(nearest(q_rand), q_rand, η)                                 # 2
    if q_new blocked: q_new ← first collision-free random probe from q_near    # 3
    if still blocked: continue
    N ← tree nodes within near radius of q_new
    parent ← argmin over feasible v ∈ N of cost(v) + ‖v−q_new‖;  add q_new     # 4
    for u ∈ N: if routing through q_new is cheaper, re-parent u;  propagate    # 5
    if q_new reaches goal region:                                             # 6
        path ← shortcut(path_to(q_new) + goal)                                # 7
        if cost(path) < best: best ← cost(path);  publish path`}/>
            <T
                en={<ol>
                    <li>Fast-Sampling: keep the goal-biased coin, but a free sample is
                        redrawn while it sits within <InlineMath math="r_{\text{reached}}"/>
                        of any node, up to a bounded retry count. The tree stops re-covering
                        ground it already owns.</li>
                    <li>The ordinary steer toward the sample from the nearest node.</li>
                    <li>Random Steering: only if the straight step is blocked. Fixed-length
                        steps in random directions are tried until one is collision-free —
                        this is what threads narrow gaps that a single straight extension
                        keeps hitting.</li>
                    <li>Choose-parent, unchanged from RRT*: the nearest node found the
                        sample, but a farther neighbor may offer a cheaper route through
                        itself.</li>
                    <li>Rewire, unchanged from RRT*: a neighbor that gets cheaper through the
                        new node is re-parented, and the saving must propagate to its whole
                        subtree or the optimality invariant breaks.</li>
                    <li>No early return: reaching the goal only updates the incumbent, so the
                        loop keeps improving to its full budget.</li>
                    <li>Fast-Optimal: shortcut the extracted path, then keep it only if it
                        beats the best cost so far. Because the shortcut is a greedy jump to
                        the farthest reachable waypoint, one pass suffices.</li>
                </ol>}
                ko={<ol>
                    <li>Fast-Sampling: goal biased 동전은 그대로 두되, 자유 표본은 어느
                        노드로부터든 <InlineMath math="r_{\text{reached}}"/> 안에 있는 동안
                        상한 횟수까지 다시 뽑는다. 트리가 이미 가진 땅을 다시 덮는 일을
                        멈춘다.</li>
                    <li>nearest 노드에서 표본 쪽으로 하는 평범한 steer다.</li>
                    <li>Random Steering: 직선 스텝이 막힌 경우에만. 무작위 방향으로 고정
                        길이 스텝을 충돌 없는 것이 나올 때까지 시도한다. 직선 확장 하나로는
                        계속 부딪히는 좁은 gap을 꿰는 것이 바로 이 단계다.</li>
                    <li>choose-parent, RRT*에서 그대로다. 표본을 찾은 것은 nearest 지만, 더
                        먼 이웃이 자기를 거치는 더 싼 길을 제안할 수 있다.</li>
                    <li>rewire, RRT*에서 그대로다. 새 노드를 거쳐 싸지는 이웃은 부모를
                        갈아타고, 절약분은 그 부분 트리 전체에 전파되어야 최적성 불변식이
                        깨지지 않는다.</li>
                    <li>조기 반환이 없다. goal 도달은 현직 해를 갱신할 뿐이라, 루프는 예산을
                        다 쓸 때까지 계속 개선한다.</li>
                    <li>Fast-Optimal: 추출된 경로를 지름길화한 뒤, 지금까지의 best 비용을
                        이겼을 때만 채택한다. 지름길이 닿는 가장 먼 waypoint로 뛰는 greedy
                        방식이라 한 번의 pass로 충분하다.</li>
                </ol>}
            />
            <Proof title={t("Why the shortcut never makes a path worse", "지름길이 경로를 나빠지게 하지 않는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> A shortcut replaces a run of waypoints{" "}
                            <InlineMath math="p_i, p_{i+1}, \dots, p_j"/> with the single
                            straight chord <InlineMath math="p_i \to p_j"/>, and it does so
                            only when that chord is collision-free. Its length compares to the
                            polyline it removes by the triangle inequality, applied along the
                            run:
                        </p>
                        <BlockMath math="\lVert p_i - p_j \rVert \;\le\; \sum_{k=i}^{j-1} \lVert p_k - p_{k+1} \rVert"/>
                        <Terms items={[
                            ["p_i,\\ p_j", "the kept endpoints of the shortcut: the waypoint jumped from and the farthest later waypoint still reachable by a straight collision-free segment"],
                            ["p_k, p_{k+1}", <>consecutive waypoints of the original polyline between <InlineMath math="p_i"/> and <InlineMath math="p_j"/></>],
                            ["\\lVert p_i - p_j \\rVert", "length of the replacement chord"],
                            ["\\sum_{k=i}^{j-1} \\lVert p_k - p_{k+1}\\rVert", "total length of the original subpath being replaced"],
                        ]}/>
                        <p>
                            Each greedy pass replaces disjoint runs, and summing the
                            inequality over them gives total length after{" "}
                            <InlineMath math="\le"/> total length before. So every accepted
                            shortcut leaves the path no longer than it was, and the incumbent
                            filter keeps only a strictly cheaper result — the pruning can
                            never regress. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 지름길은 waypoint들의 한 구간{" "}
                            <InlineMath math="p_i, p_{i+1}, \dots, p_j"/>를 하나의 직선 현{" "}
                            <InlineMath math="p_i \to p_j"/>로 바꾸며, 그 현이 충돌 없을
                            때에만 그렇게 한다. 그 길이는 제거되는 폴리라인과 삼각 부등식을
                            구간을 따라 적용해 비교된다.
                        </p>
                        <BlockMath math="\lVert p_i - p_j \rVert \;\le\; \sum_{k=i}^{j-1} \lVert p_k - p_{k+1} \rVert"/>
                        <Terms items={[
                            ["p_i,\\ p_j", "지름길의 유지된 양 끝점. 뛰어 나가는 waypoint와, 직선 충돌 없는 구간으로 아직 닿는 가장 먼 이후 waypoint"],
                            ["p_k, p_{k+1}", <><InlineMath math="p_i"/>와 <InlineMath math="p_j"/> 사이 원래 폴리라인의 연속한 waypoint</>],
                            ["\\lVert p_i - p_j \\rVert", "대체하는 현의 길이"],
                            ["\\sum_{k=i}^{j-1} \\lVert p_k - p_{k+1}\\rVert", "대체되는 원래 부분 경로의 총 길이"],
                        ]}/>
                        <p>
                            greedy pass는 겹치지 않는 구간들을 대체하고, 그 구간들에 대해
                            부등식을 합하면 대체 후 총 길이가 대체 전 총 길이{" "}
                            <InlineMath math="\le"/>가 된다. 따라서 채택된 지름길은 경로를
                            이전보다 길게 만들지 않고, 현직 필터는 엄격히 더 싼 결과만
                            남긴다. pruning은 결코 뒷걸음질하지 않는다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox seals a vertical wall except for a single-cell gap, with a
                    diagonal start and goal so the tree has to find the opening and bend
                    through it. Fast-Sampling spreads the tree evenly instead of clumping,
                    and Random Steering threads the gap the straight extension keeps
                    hitting, so the red Fast-RRT path stays smooth and reaches the goal
                    reliably; the dashed same-seed RRT answer is jagged and often misses the
                    passage entirely. Press regrow to reseed and watch how much more
                    consistently Fast-RRT gets through. The replay below is the repository
                    demo on the benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 세로 벽을 한 칸 gap만 남기고 막고, start와 goal을 대각으로
                    두어 트리가 통로를 찾아 꺾어 들어가게 한다. Fast-Sampling이 트리를
                    뭉치지 않고 고루 펴고, Random Steering이 직선 확장으로는 계속 부딪히는
                    gap을 꿰므로, 빨간 Fast-RRT 경로는 매끈하고 goal에 안정적으로 닿는다.
                    같은 seed의 점선 RRT 답은 삐죽하고 통로를 통째로 놓칠 때가 많다. 다시
                    성장 버튼으로 seed를 바꿔, Fast-RRT가 얼마나 더 꾸준히 통과하는지 보라.
                    아래 replay는 벤치마크 맵에서 돌린 저장소 demo다.
                </p>}
            />
            <FastRrtSandbox/>
            <TraceReplay algo="fast_rrt" maps={["maze01", "open01"]} label={t(
                "Real traces from the repository's Fast-RRT demo — fast-sampling keeps the tree spreading into unreached space, and each published path is already shortcut taut",
                "저장소 Fast-RRT demo의 실제 trace. fast-sampling이 트리를 미탐사 공간으로 계속 펴 나가고, 발표되는 경로는 이미 지름길로 팽팽하게 당겨져 있다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The RRT* tree, choose-parent, and rewire come from the shared sampling
                    module; the Fast-RRT file adds only Fast-Sampling, Random Steering, and
                    the greedy shortcut prune. Both language ports run the same operations in
                    the same order, so the browser engine above matches the recorded traces
                    exactly. Embedded below in full.
                </p>}
                ko={<p>
                    RRT* 트리와 choose-parent, rewire는 공유 sampling 모듈에서 온다.
                    Fast-RRT 파일이 더하는 것은 Fast-Sampling, Random Steering, 그리고 greedy
                    지름길 pruning 뿐이다. 두 언어 포트가 같은 연산을 같은 순서로 돌리므로,
                    위 브라우저 엔진이 기록된 trace와 정확히 일치한다. 전체를 아래에 embed
                    했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/fast_rrt.py",
                            code: fastRrtPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/fast_rrt.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/fast_rrt.cpp",
                            code: fastRrtCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/fast_rrt.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Fast-RRT implementation, embedded from the repository sources",
                    "Fast-RRT 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    Z. Wu, Z. Meng, W. Zhao, Z. Wu,{" "}
                    <a href="https://doi.org/10.3390/app112411777" target="_blank"
                       rel="noopener noreferrer">
                        <em>Fast-RRT: A RRT-Based Optimal Path Finding Method</em>
                    </a>,
                    Applied Sciences, 11(24):11777, 2021.
                </li>
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

export default FastRrt
