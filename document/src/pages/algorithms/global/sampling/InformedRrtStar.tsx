import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import InformedRrtStarSandbox from "../../../../components/panels/global/informed_rrt_star/InformedRrtStarSandbox";
import informedRrtStarPy from "../../../../../../python/navigation/global_planning/sampling/informed_rrt_star.py?raw";
import informedRrtStarCpp from "../../../../../../cpp/src/global_planning/sampling/informed_rrt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const InformedRrtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    RRT* never stops sampling the whole map. Long after a solution exists,
                    it keeps throwing points into far corners the current path could never
                    benefit from, and each wasted sample is an iteration that did nothing for
                    the answer. Informed RRT* (Gammell, Srinivasa &amp; Barfoot, 2014) changes
                    exactly one line: once a path of cost <InlineMath math="c_{\text{best}}"/>{" "}
                    exists, it draws only from the ellipse of states that could still beat it.
                    The tree machinery is untouched — same choose-parent, same rewire, same
                    almost-sure convergence — but every post-solution sample now lands where
                    it can actually help, so the cost tightens far faster.
                </p>}
                ko={<p>
                    RRT*은 맵 전체 표본 추출을 멈추지 않는다. 해가 나온 뒤로도 한참을,
                    현재 경로가 도움받을 수 없는 먼 구석에 점을 계속 던진다. 낭비된 표본
                    하나하나가 답에 아무 기여도 못 한 반복이다. Informed RRT*(Gammell,
                    Srinivasa &amp; Barfoot, 2014)은 딱 한 줄을 바꾼다. 비용{" "}
                    <InlineMath math="c_{\text{best}}"/>의 경로가 하나 생기면, 그것을 아직
                    이길 수 있는 상태들의 타원 안에서만 표본을 뽑는다. 트리 기계는 그대로다.
                    choose-parent도, rewire도, 거의 확실한 수렴도 같다. 다만 해 발견 이후의
                    표본이 이제 실제로 도움이 되는 곳에만 떨어지므로 비용이 훨씬 빨리
                    조여진다.
                </p>}
            />

            <h2>{t("From Everywhere to the Ellipse", "전 공간에서 타원으로")}</h2>
            <T
                en={<>
                    <p>
                        Suppose a solution of cost <InlineMath math="c_{\text{best}}"/> is in
                        hand. Take any state <InlineMath math="x"/> and ask the cheapest
                        possible cost of a start-to-goal path forced through it. Each leg is
                        at least its straight-line length, so that cost is bounded below by
                    </p>
                    <BlockMath math="f(x) = \lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert"/>
                    <Terms items={[
                        ["x", "any candidate state in the plane"],
                        ["x_{\\text{start}}", "the start state (one focus of the ellipse)"],
                        ["x_{\\text{goal}}", "the goal state (the other focus)"],
                        ["f(x)", <><strong>the new term</strong>: the straight-line lower bound on any start→goal path routed through <InlineMath math="x"/> — cost through <InlineMath math="x"/> can only exceed it</>],
                    ]}/>
                    <p>
                        If <InlineMath math="f(x) > c_{\text{best}}"/> then no path through{" "}
                        <InlineMath math="x"/> can undercut the incumbent, so a sample landing
                        there is pure waste. The states that might still help are exactly
                    </p>
                    <BlockMath math="\mathcal{X}_f = \bigl\{\, x : \lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert \le c_{\text{best}} \,\bigr\}"/>
                    <Terms items={[
                        ["\\mathcal{X}_f", <><strong>the new term</strong>: the informed set — states whose lower bound <InlineMath math="f(x)"/> still clears the incumbent</>],
                        ["x_{\\text{start}}", "the start state (focus)"],
                        ["x_{\\text{goal}}", "the goal state (focus)"],
                        ["c_{\\text{best}}", "the cost of the current best solution (the incumbent)"],
                    ]}/>
                    <p>
                        The defining property of an ellipse is that the sum of distances to
                        its two foci is constant, so <InlineMath math="\mathcal{X}_f"/> is a
                        filled ellipse with foci <InlineMath math="x_{\text{start}}"/> and{" "}
                        <InlineMath math="x_{\text{goal}}"/>. Its transverse diameter is the
                        incumbent cost itself, and its width follows from the focal distance:
                    </p>
                    <BlockMath math="a = \frac{c_{\text{best}}}{2}, \qquad b = \frac{\sqrt{c_{\text{best}}^2 - c_{\min}^2}}{2}, \qquad c_{\min} = \lVert x_{\text{goal}} - x_{\text{start}} \rVert"/>
                    <Terms items={[
                        ["a", "semi-major axis, half the transverse diameter — set by the incumbent cost"],
                        ["b", "semi-minor axis, half the conjugate diameter — how far the ellipse bulges off the straight line"],
                        ["c_{\\text{best}}", "the cost of the current best solution"],
                        ["c_{\\min}", <><strong>the new term</strong>: the straight-line distance between the two foci — the theoretical lower bound on any solution</>],
                    ]}/>
                    <p>
                        As <InlineMath math="c_{\text{best}}"/> drops toward{" "}
                        <InlineMath math="c_{\min}"/>, the ellipse collapses onto the segment
                        between start and goal, and sampling focuses ever tighter. That
                        feedback — a better path shrinks the region, a smaller region finds
                        better paths — is the whole method.
                    </p>
                </>}
                ko={<>
                    <p>
                        비용 <InlineMath math="c_{\text{best}}"/>의 해가 손에 있다고 하자.
                        임의의 상태 <InlineMath math="x"/>를 하나 잡고, 그 점을 반드시
                        지나는 시작→목표 경로의 최소 가능 비용을 묻는다. 각 구간은 적어도
                        직선 길이만큼이므로 그 비용은 아래 값이 하한이다.
                    </p>
                    <BlockMath math="f(x) = \lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert"/>
                    <Terms items={[
                        ["x", "평면 위 임의의 후보 상태"],
                        ["x_{\\text{start}}", "시작 상태 (타원의 한 초점)"],
                        ["x_{\\text{goal}}", "목표 상태 (다른 초점)"],
                        ["f(x)", <><strong>새로 추가된 항</strong>: <InlineMath math="x"/>를 지나는 시작→목표 경로의 직선 하한. <InlineMath math="x"/>를 지나는 실제 비용은 이보다 클 수만 있다</>],
                    ]}/>
                    <p>
                        <InlineMath math="f(x) > c_{\text{best}}"/>이면{" "}
                        <InlineMath math="x"/>를 지나는 어떤 경로도 현직 해를 깎을 수 없으니,
                        거기 떨어진 표본은 순전한 낭비다. 아직 도움이 될 수 있는 상태는
                        정확히 다음 집합이다.
                    </p>
                    <BlockMath math="\mathcal{X}_f = \bigl\{\, x : \lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert \le c_{\text{best}} \,\bigr\}"/>
                    <Terms items={[
                        ["\\mathcal{X}_f", <><strong>새로 추가된 항</strong>: informed 집합. 하한 <InlineMath math="f(x)"/>가 아직 현직 해를 통과하는 상태들</>],
                        ["x_{\\text{start}}", "시작 상태 (초점)"],
                        ["x_{\\text{goal}}", "목표 상태 (초점)"],
                        ["c_{\\text{best}}", "현재 best 해의 비용 (현직 해)"],
                    ]}/>
                    <p>
                        타원의 정의는 두 초점까지의 거리 합이 일정하다는 것이므로,{" "}
                        <InlineMath math="\mathcal{X}_f"/>는 <InlineMath math="x_{\text{start}}"/>와{" "}
                        <InlineMath math="x_{\text{goal}}"/>을 초점으로 하는 채워진 타원이다.
                        횡축 지름은 현직 해의 비용 그 자체이고, 폭은 초점 간 거리로 정해진다.
                    </p>
                    <BlockMath math="a = \frac{c_{\text{best}}}{2}, \qquad b = \frac{\sqrt{c_{\text{best}}^2 - c_{\min}^2}}{2}, \qquad c_{\min} = \lVert x_{\text{goal}} - x_{\text{start}} \rVert"/>
                    <Terms items={[
                        ["a", "장반경. 횡축 지름의 절반으로, 현직 해 비용이 정한다"],
                        ["b", "단반경. 켤레 지름의 절반으로, 타원이 직선에서 얼마나 부풀어 나오는지다"],
                        ["c_{\\text{best}}", "현재 best 해의 비용"],
                        ["c_{\\min}", <><strong>새로 추가된 항</strong>: 두 초점 사이의 직선 거리. 어떤 해든 넘을 수 없는 이론 하한</>],
                    ]}/>
                    <p>
                        <InlineMath math="c_{\text{best}}"/>이 <InlineMath math="c_{\min}"/>
                        쪽으로 내려갈수록 타원은 시작과 목표를 잇는 선분으로 접히고, 표본은
                        점점 더 좁게 몰린다. 더 나은 경로가 영역을 줄이고 더 좁은 영역이 더
                        나은 경로를 찾는 이 되먹임이 방법의 전부다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Same guarantees as RRT*</strong>: asymptotically optimal and
                        probabilistically complete. The informed set always contains the
                        optimal path, so restricting samples to it never removes the solution
                        the convergence argument needs.</li>
                    <li><strong>Faster convergence</strong>: after the first solution every
                        sample is spent inside the shrinking ellipse instead of the whole
                        space, so the incumbent tightens with far fewer iterations — most
                        pronounced on large, open maps where the ellipse is a small fraction
                        of the free area.</li>
                    <li><strong>Anytime</strong>: the incumbent improves whenever a cheaper
                        connection appears, exactly as RRT*.</li>
                    <li><strong>Cost</strong>: identical per-iteration work to RRT* (one
                        nearest query, a near-set scan, choose-parent and rewire). Direct
                        ellipse sampling is <InlineMath math="O(1)"/>, so the focusing is free
                        — no rejection loop.</li>
                    <li><strong>Before the first solution</strong>: behaves as plain RRT* with
                        goal-biased uniform sampling. The ellipse has no meaning until a
                        finite <InlineMath math="c_{\text{best}}"/> exists.</li>
                </ul>}
                ko={<ul>
                    <li><strong>RRT*과 같은 보장</strong>: 점근 최적이고 확률적 완전이다.
                        informed 집합은 항상 최적 경로를 품으므로, 표본을 그 안으로 제한해도
                        수렴 논증에 필요한 해가 사라지지 않는다.</li>
                    <li><strong>더 빠른 수렴</strong>: 첫 해 이후 모든 표본이 전 공간이 아니라
                        줄어드는 타원 안에서 쓰인다. 그래서 현직 해가 훨씬 적은 반복으로
                        조여진다. 타원이 free 공간의 작은 일부인 넓은 개방 맵에서 효과가 가장
                        크다.</li>
                    <li><strong>Anytime</strong>: 더 싼 연결이 나타날 때마다 현직 해가
                        갱신된다. RRT*과 똑같다.</li>
                    <li><strong>비용</strong>: 반복당 작업량이 RRT*과 동일하다 (nearest 한 번,
                        near 집합 스캔, choose-parent, rewire). 타원 직접 표본은{" "}
                        <InlineMath math="O(1)"/>이라 초점화에 추가 비용이 없다. rejection
                        루프가 없다.</li>
                    <li><strong>첫 해 이전</strong>: goal-bias 균일 표본을 쓰는 평범한 RRT*로
                        동작한다. 유한한 <InlineMath math="c_{\text{best}}"/>이 생기기 전엔
                        타원이 의미를 갖지 않는다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The RRT* loop verbatim, with a single fork at the sampling line and a
                    direct map from the unit disk into the ellipse (no rejection):
                </p>}
                ko={<p>
                    RRT* 루프를 그대로 두고, 표본 추출 줄에서만 갈래를 하나 두어 단위 원판을
                    타원으로 직접 옮긴다 (rejection 없음):
                </p>}
            />
            <Pseudocode code={`tree ← {start};  best ← ∞
repeat max_iterations times:
    if best < ∞:                                             # 1
        (angle, radius) ← uniform disk draw
        q_rand ← rotate & scale disk point into ellipse(start, goal, best)
    else:
        q_rand ← goal with probability p, else uniform sample
    q_new ← steer(nearest(q_rand), q_rand, η);  skip if blocked
    N ← tree nodes within near radius of q_new
    parent ← argmin over feasible v ∈ N of cost(v) + ‖v−q_new‖   # 2
    add q_new under parent
    for u ∈ N: reroute u through q_new if cheaper, propagate       # 3
    if q_new reaches goal region and beats best:                  # 4
        best ← that cost;  publish improved path`}/>
            <T
                en={<ol>
                    <li>The one new branch. Once a finite <InlineMath math="c_{\text{best}}"/>{" "}
                        exists, a uniform point in the unit disk is scaled by the semi-axes{" "}
                        <InlineMath math="(a, b)"/> and rotated to align with the start–goal
                        axis, landing uniformly inside the ellipse. The draw order — angle
                        first, then radius as <InlineMath math="\sqrt{\text{uniform}}"/> for
                        area-uniformity — must match the reference exactly for the sample
                        stream to be reproducible.</li>
                    <li>Choose-parent, unchanged from RRT*: a farther neighbor may still offer
                        a cheaper route through itself than the nearest node did.</li>
                    <li>Rewire, unchanged: reroute neighbors through <InlineMath math="q_{\text{new}}"/>{" "}
                        when cheaper, and push the saving down each subtree.</li>
                    <li>No early return. Reaching the goal only updates the incumbent, which
                        in turn shrinks the ellipse the next iteration samples from.</li>
                </ol>}
                ko={<ol>
                    <li>새로 생긴 유일한 갈래다. 유한한 <InlineMath math="c_{\text{best}}"/>
                        이 생기면 단위 원판 안 균일 점을 반축{" "}
                        <InlineMath math="(a, b)"/>로 늘이고 start–goal 축에 맞춰 회전해
                        타원 안에 균일하게 떨어뜨린다. draw 순서는 각도가 먼저, 그다음 면적
                        균일을 위한 <InlineMath math="\sqrt{\text{uniform}}"/> 반지름이다. 이
                        순서가 레퍼런스와 정확히 같아야 표본열이 재현된다.</li>
                    <li>choose-parent. RRT*과 그대로다. 더 먼 이웃이 nearest 노드보다 자기를
                        거치는 더 싼 길을 제안할 수 있다.</li>
                    <li>rewire. 그대로다. <InlineMath math="q_{\text{new}}"/>를 거치는 편이
                        싸지는 이웃을 재배선하고 절약분을 각 부분 트리로 내려보낸다.</li>
                    <li>조기 반환이 없다. goal 도달은 현직 해를 갱신할 뿐이고, 그 갱신이
                        다음 반복이 표본을 뽑을 타원을 다시 줄인다.</li>
                </ol>}
            />
            <Proof title={t("Why samples outside the ellipse cannot help", "타원 밖 표본이 도움 될 수 없는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> An incumbent of cost{" "}
                            <InlineMath math="c_{\text{best}}"/> exists. Let{" "}
                            <InlineMath math="x"/> be any state and{" "}
                            <InlineMath math="\sigma"/> any feasible start→goal path passing
                            through it. Since every leg costs at least its Euclidean length,
                        </p>
                        <BlockMath math="\mathrm{cost}(\sigma) \;\ge\; \lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert \;=\; f(x)"/>
                        <Terms items={[
                            ["\\sigma", <>any feasible path from start to goal through <InlineMath math="x"/></>],
                            ["x", "the state under test"],
                            ["x_{\\text{start}}, x_{\\text{goal}}", "start and goal (the ellipse foci)"],
                            ["f(x)", <>the straight-line lower bound <InlineMath math="\lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert"/></>],
                        ]}/>
                        <p>
                            <strong>Contradiction.</strong> Suppose <InlineMath math="x"/>{" "}
                            lies outside the ellipse, i.e.{" "}
                            <InlineMath math="f(x) > c_{\text{best}}"/>. Chaining with the
                            bound above,
                        </p>
                        <BlockMath math="\mathrm{cost}(\sigma) \;\ge\; f(x) \;>\; c_{\text{best}}"/>
                        <Terms items={[
                            ["\\mathrm{cost}(\\sigma)", <>cost of any path through <InlineMath math="x"/></>],
                            ["f(x)", "the straight-line lower bound at x"],
                            ["c_{\\text{best}}", "the incumbent cost"],
                        ]}/>
                        <p>
                            So every path through such an <InlineMath math="x"/> is strictly
                            worse than the incumbent — it can never become the new best.
                            Uniform sampling outside <InlineMath math="\mathcal{X}_f"/> spends
                            iterations that provably cannot improve the answer, while the
                            optimal path lies entirely inside <InlineMath math="\mathcal{X}_f"/>{" "}
                            (its own cost is <InlineMath math="\le c_{\text{best}}"/>).
                            Restricting draws to the ellipse discards only the useless region.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 비용{" "}
                            <InlineMath math="c_{\text{best}}"/>의 현직 해가 있다.{" "}
                            <InlineMath math="x"/>를 임의의 상태로,{" "}
                            <InlineMath math="\sigma"/>를 그 점을 지나는 임의의 실행 가능한
                            시작→목표 경로로 두자. 모든 구간이 적어도 유클리드 길이만큼
                            드므로
                        </p>
                        <BlockMath math="\mathrm{cost}(\sigma) \;\ge\; \lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert \;=\; f(x)"/>
                        <Terms items={[
                            ["\\sigma", <>시작에서 목표까지 <InlineMath math="x"/>를 지나는 임의의 실행 가능 경로</>],
                            ["x", "검사 대상 상태"],
                            ["x_{\\text{start}}, x_{\\text{goal}}", "시작과 목표 (타원 초점)"],
                            ["f(x)", <>직선 하한 <InlineMath math="\lVert x - x_{\text{start}} \rVert + \lVert x - x_{\text{goal}} \rVert"/></>],
                        ]}/>
                        <p>
                            <strong>모순.</strong> <InlineMath math="x"/>가 타원 밖에 있다고,
                            즉 <InlineMath math="f(x) > c_{\text{best}}"/>이라 하자. 위 하한과
                            엮으면
                        </p>
                        <BlockMath math="\mathrm{cost}(\sigma) \;\ge\; f(x) \;>\; c_{\text{best}}"/>
                        <Terms items={[
                            ["\\mathrm{cost}(\\sigma)", <><InlineMath math="x"/>를 지나는 임의 경로의 비용</>],
                            ["f(x)", "x에서의 직선 하한"],
                            ["c_{\\text{best}}", "현직 해의 비용"],
                        ]}/>
                        <p>
                            그러므로 그런 <InlineMath math="x"/>를 지나는 모든 경로는 현직
                            해보다 엄격히 나쁘고, 새 best가 될 수 없다.{" "}
                            <InlineMath math="\mathcal{X}_f"/> 밖의 균일 표본은 답을 개선할 수
                            없음이 증명된 반복을 소모한다. 반면 최적 경로는 전부{" "}
                            <InlineMath math="\mathcal{X}_f"/> 안에 있다 (그 비용이{" "}
                            <InlineMath math="\le c_{\text{best}}"/>이므로). 표본을 타원으로
                            제한하는 것은 쓸모없는 영역만 버리는 셈이다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox opens on a wide map whose diagonal is partly blocked, so the
                    optimum bends slightly off the straight line and leaves room to converge.
                    Watch the samples: they scatter across the whole map until the first path
                    appears, then snap into a diagonal band — the informed ellipse — and stay
                    there. The red Informed path settles below the dashed RRT* path grown from
                    the same seed and budget, because RRT* keeps wasting samples in the empty
                    corners. Raise the budget or press regrow to see the gap hold.
                </p>}
                ko={<p>
                    sandbox는 대각선이 일부 막힌 넓은 맵에서 열린다. 최적이 직선에서 살짝
                    꺾여 수렴 여지가 남는다. 표본을 보라. 첫 경로가 나오기 전엔 맵 전체에
                    흩어지다가, 경로가 나오는 순간 대각선 띠 안으로, 곧 informed 타원 안으로
                    딱 접혀 거기 머문다. 빨간 Informed 경로는 같은 seed·예산으로 키운 점선
                    RRT* 경로 아래로 내려앉는다. RRT*은 빈 구석에 표본을 계속 버리기
                    때문이다. 예산을 올리거나 다시 성장을 눌러 그 격차가 유지되는지 보라.
                </p>}
            />
            <InformedRrtStarSandbox/>
            <TraceReplay algo="informed_rrt_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's Informed RRT* demo — after the first solution the samples concentrate in the start–goal ellipse and the published path improves in steps",
                "저장소 Informed RRT* demo의 실제 trace. 첫 해 이후 표본이 start–goal 타원에 몰리고, 발표되는 경로가 계단식으로 좋아진다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The planner file is the RRT* loop with the sampling line forked to the
                    ellipse. Choose-parent, rewire, and the ellipse sampler all live in the
                    shared sampling module — the same <code>informed_sample</code> is reused
                    by the other informed planners — so this file adds only the incumbent-gated
                    switch between uniform and ellipse draws. Embedded below in full.
                </p>}
                ko={<p>
                    planner 파일은 표본 추출 줄만 타원으로 가른 RRT* 루프다. choose-parent,
                    rewire, 타원 표본기는 모두 공유 sampling 모듈에 있다. 같은{" "}
                    <code>informed_sample</code>을 다른 informed planner들이 재사용한다. 그래서
                    이 파일은 현직 해 유무로 균일 표본과 타원 표본을 가르는 스위치만 더한다.
                    전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/informed_rrt_star.py",
                            code: informedRrtStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/informed_rrt_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/informed_rrt_star.cpp",
                            code: informedRrtStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/informed_rrt_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Informed RRT* implementation, embedded from the repository sources",
                    "Informed RRT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    J. D. Gammell, S. S. Srinivasa, T. D. Barfoot,{" "}
                    <a href="https://doi.org/10.1109/IROS.2014.6942976" target="_blank"
                       rel="noopener noreferrer">
                        <em>Informed RRT*: Optimal Sampling-based Path Planning Focused via
                        Direct Sampling of an Admissible Ellipsoidal Heuristic</em>
                    </a>,
                    IEEE/RSJ International Conference on Intelligent Robots and Systems
                    (IROS), 2014.
                </li>
                <li>
                    J. D. Gammell, T. D. Barfoot, S. S. Srinivasa,{" "}
                    <a href="https://doi.org/10.1109/TRO.2018.2830331" target="_blank"
                       rel="noopener noreferrer">
                        <em>Informed Sampling for Asymptotically Optimal Path Planning</em>
                    </a>,
                    IEEE Transactions on Robotics, 2018.
                </li>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank"
                       rel="noopener noreferrer">
                        <em>Sampling-based Algorithms for Optimal Motion Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2011.
                </li>
            </ol>
        </>
    )
}

export default InformedRrtStar
