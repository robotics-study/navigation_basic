import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import KinodynamicSandbox from "../../../../components/panels/global/kinodynamic_rrt_star/KinodynamicSandbox";
import kinoPy from "../../../../../../python/navigation/global_planning/sampling/kinodynamic_rrt_star.py?raw";
import kinoCpp from "../../../../../../cpp/src/global_planning/sampling/kinodynamic_rrt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const KinodynamicRrtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every RRT* so far drew a <em>straight</em> edge between two states and
                    charged its Euclidean length. That is only honest for a robot that can
                    move in any direction at will. A body with momentum cannot: a point mass
                    steered by acceleration keeps its velocity, so the trajectory that reaches
                    a target state is a curve, and its true cost is control effort, not
                    distance. Kinodynamic RRT* (Webb &amp; van den Berg, 2013) keeps the whole
                    RRT* skeleton and swaps out just two pieces. The straight edge becomes the
                    exact trajectory of a fixed-final-state, free-final-time optimal controller
                    for a controllable linear system, and the Euclidean metric becomes that
                    controller&apos;s arrival cost. The tree then grows and straightens in the
                    cost geometry the dynamics induce, not in Euclidean space.
                </p>}
                ko={<p>
                    지금까지의 RRT*은 모두 두 상태 사이에 <em>직선</em> 간선을 긋고 그
                    유클리드 길이를 비용으로 매겼다. 이는 아무 방향으로나 마음대로 움직이는
                    로봇에게만 정직하다. 관성을 가진 물체는 그럴 수 없다. 가속도로 조종되는
                    점질량은 속도를 유지하므로, 목표 상태에 닿는 궤적은 곡선이고 그 진짜
                    비용은 거리가 아니라 제어 노력이다. Kinodynamic RRT*(Webb &amp; van den
                    Berg, 2013)은 RRT*의 골격을 그대로 두고 딱 두 조각만 갈아 끼운다. 직선
                    간선은 제어 가능한 선형계의 고정 최종상태·자유 최종시간 최적 제어기의
                    정확한 궤적이 되고, 유클리드 척도는 그 제어기의 도달 비용이 된다. 그러면
                    트리는 유클리드가 아니라 동역학이 유도하는 비용 기하에서 자라고 곧게
                    펴진다.
                </p>}
            />

            <h2>{t("From Straight Edges to Optimal Trajectories", "직선 간선에서 최적 궤적으로")}</h2>
            <T
                en={<>
                    <p>
                        The repository runs the planner on a 2D double integrator: the state is
                        a position and a velocity <InlineMath math="(x, y, v_x, v_y)"/>, the
                        control is acceleration, and the running cost trades time against effort
                        as <InlineMath math="J = \int_0^\tau \bigl(1 + r\,u^\top u\bigr)\,dt"/>.
                        The map never sees the velocity. It answers only whether the{" "}
                        <InlineMath math="(x, y)"/> projection of the trajectory is collision
                        free, the same <InlineMath math="\text{SamplingSpace}"/> the earlier
                        RRT* pages use. Connecting two states means solving for the arrival
                        time. For a fixed final time <InlineMath math="t"/> the minimum-effort
                        cost decouples per axis into
                    </p>
                    <BlockMath math="c(t) = t + r\left(\frac{C_3}{t^3} + \frac{C_2}{t^2} + \frac{C_1}{t}\right)"/>
                    <Terms items={[
                        ["c(t)", "the cost of arriving at the target state exactly at time t, using the minimum-effort control that does so"],
                        ["t", "the arrival time, the single free variable that is optimized away"],
                        ["r", <>the effort penalty (the config&apos;s <code>control_weight</code>): larger <InlineMath math="r"/> charges control more, so trajectories smooth out and slow down</>],
                        ["C_1", <><strong>the new term</strong>: <InlineMath math="\sum_{\text{axes}} 4\,(v_0^2 + v_0 v_1 + v_1^2)"/>, the velocity-mismatch coefficient</>],
                        ["C_2", <><strong>the new term</strong>: <InlineMath math="\sum_{\text{axes}} -12\,a\,(v_0 + v_1)"/>, the position-velocity coupling coefficient</>],
                        ["C_3", <><strong>the new term</strong>: <InlineMath math="\sum_{\text{axes}} 12\,a^2"/>, the position-gap coefficient</>],
                        ["a", <>the per-axis position gap <InlineMath math="p_1 - p_0"/> between the two states</>],
                        ["v_0, v_1", "the per-axis start and end velocities of the two states"],
                    ]}/>
                    <p>
                        The first term rewards arriving sooner and the bracket punishes the
                        effort that haste demands, so <InlineMath math="c(t)"/> blows up at both
                        ends and dips to a single minimum in between. Setting{" "}
                        <InlineMath math="c'(t) = 0"/> and clearing the denominator gives a
                        depressed quartic whose positive real root is the optimal arrival time:
                    </p>
                    <BlockMath math="t^4 - r\,C_1\,t^2 - 2r\,C_2\,t - 3r\,C_3 = 0 \qquad\Longrightarrow\qquad \tau^\ast = \arg\min_{t > 0} c(t)"/>
                    <Terms items={[
                        ["t", "the arrival time being solved for; the quartic is c'(t) = 0 cleared by t⁴"],
                        ["r", "the effort penalty weighting control cost against time"],
                        ["C_1, C_2, C_3", <>the same velocity-mismatch, coupling, and position-gap coefficients from <InlineMath math="c(t)"/> above</>],
                        ["\\tau^\\ast", <><strong>the new term</strong>: the optimal arrival time, the positive real root of the quartic that minimizes <InlineMath math="c(t)"/></>],
                    ]}/>
                    <p>
                        The connection cost <InlineMath math="c(\tau^\ast)"/> plays the role that
                        Euclidean distance played in geometric RRT*. It is the nearest-neighbour
                        metric and it is the choose-parent and rewire cost. The trajectory
                        realising it is the unique cubic that meets both endpoints&apos;
                        positions and velocities, the minimum-<InlineMath math="\int\lVert u\rVert^2"/>{" "}
                        Hermite interpolant, and that curve is what the collision checker walks.
                    </p>
                </>}
                ko={<>
                    <p>
                        저장소는 2D double integrator 위에서 planner를 돌린다. 상태는 위치와
                        속도 <InlineMath math="(x, y, v_x, v_y)"/>, control은 가속도, 진행 비용은
                        시간과 노력을 맞바꾸는{" "}
                        <InlineMath math="J = \int_0^\tau \bigl(1 + r\,u^\top u\bigr)\,dt"/>이다.
                        맵은 속도를 결코 보지 않는다. 궤적의 <InlineMath math="(x, y)"/> 투영이
                        충돌 없는지만 답하며, 앞 RRT* 페이지가 쓰는 그{" "}
                        <InlineMath math="\text{SamplingSpace}"/>와 같다. 두 상태를 잇는다는 것은
                        도달 시간을 푸는 것이다. 고정 최종시간 <InlineMath math="t"/>에 대해
                        최소 노력 비용은 축별로 분해되어
                    </p>
                    <BlockMath math="c(t) = t + r\left(\frac{C_3}{t^3} + \frac{C_2}{t^2} + \frac{C_1}{t}\right)"/>
                    <Terms items={[
                        ["c(t)", "정확히 시간 t에 목표 상태에 도달하는 최소 노력 제어의 비용"],
                        ["t", "도달 시간. 최적화로 소거되는 유일한 자유 변수"],
                        ["r", <>제어 페널티(config의 <code>control_weight</code>). <InlineMath math="r"/>이 클수록 제어에 비용을 더 매겨 궤적이 완만하고 느려진다</>],
                        ["C_1", <><strong>새로 추가된 항</strong>: <InlineMath math="\sum_{\text{axes}} 4\,(v_0^2 + v_0 v_1 + v_1^2)"/>, 속도 불일치 계수</>],
                        ["C_2", <><strong>새로 추가된 항</strong>: <InlineMath math="\sum_{\text{axes}} -12\,a\,(v_0 + v_1)"/>, 위치·속도 결합 계수</>],
                        ["C_3", <><strong>새로 추가된 항</strong>: <InlineMath math="\sum_{\text{axes}} 12\,a^2"/>, 위치 격차 계수</>],
                        ["a", <>두 상태의 축별 위치 격차 <InlineMath math="p_1 - p_0"/></>],
                        ["v_0, v_1", "두 상태의 축별 시작·끝 속도"],
                    ]}/>
                    <p>
                        첫 항은 빨리 도착할수록 보상하고 괄호는 그 조급함이 요구하는 노력을
                        벌한다. 그래서 <InlineMath math="c(t)"/>는 양 끝에서 발산하고 그
                        사이에서 최솟값 하나로 내려간다. <InlineMath math="c'(t) = 0"/>으로
                        놓고 분모를 없애면 depressed quartic이 되고, 그 양의 실근이 최적 도달
                        시간이다:
                    </p>
                    <BlockMath math="t^4 - r\,C_1\,t^2 - 2r\,C_2\,t - 3r\,C_3 = 0 \qquad\Longrightarrow\qquad \tau^\ast = \arg\min_{t > 0} c(t)"/>
                    <Terms items={[
                        ["t", "풀이 대상 도달 시간. quartic은 c'(t) = 0을 t⁴로 소거한 것이다"],
                        ["r", "시간 대비 제어 비용을 저울질하는 제어 페널티"],
                        ["C_1, C_2, C_3", <>위의 <InlineMath math="c(t)"/>와 같은 속도 불일치·결합·위치 격차 계수</>],
                        ["\\tau^\\ast", <><strong>새로 추가된 항</strong>: 최적 도달 시간. <InlineMath math="c(t)"/>를 최소화하는 quartic의 양의 실근</>],
                    ]}/>
                    <p>
                        연결 비용 <InlineMath math="c(\tau^\ast)"/>은 기하 RRT*에서 유클리드
                        거리가 하던 역할을 맡는다. nearest 척도이자 choose-parent·rewire
                        비용이다. 그것을 실현하는 궤적은 양 끝의 위치와 속도를 모두 만족하는
                        유일한 3차, 곧 최소-<InlineMath math="\int\lVert u\rVert^2"/> Hermite
                        보간이고, 그 곡선이 충돌 검사기가 따라 걷는 대상이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Exact steering, in closed form</strong>: for a controllable
                        linear system the fixed-final-state controller reaches any target state
                        exactly, so <InlineMath math="x_{\text{new}}"/> is the sample itself, with
                        no <InlineMath math="\eta"/>-step truncation. SST gave steering up;
                        kinodynamic RRT* keeps it and pays for it with a quartic solve.</li>
                    <li><strong>Dynamically feasible edges</strong>: every edge is a real
                        double-integrator trajectory, so the returned path is executable by a
                        body with momentum, unlike a straight-line geometric path.</li>
                    <li><strong>Asymptotically optimal</strong>: with the optimal-control cost as
                        its metric, the RRT* choose-parent and rewire machinery converges to the
                        minimum-cost trajectory as samples grow (Webb &amp; van den Berg 2013,
                        Karaman &amp; Frazzoli 2011).</li>
                    <li><strong>Cost per iteration</strong>: one quartic root solve per candidate
                        connection. A Euclidean prefilter picks the <InlineMath math="K"/> nearest
                        before the exact-cost comparison, and the neighbourhood is capped at{" "}
                        <InlineMath math="K"/>, so the work stays bounded on dense trees.</li>
                    <li><strong>Linear-dynamics only</strong>: the closed form needs{" "}
                        <InlineMath math="x' = Ax + Bu"/>. Nonlinear dynamics need a linearisation
                        or a numerical two-point boundary-value solve, which is exactly the cost
                        SST sidesteps by not steering at all.</li>
                </ul>}
                ko={<ul>
                    <li><strong>정확한 steering, 닫힌 형태로</strong>: 제어 가능한 선형계에서
                        고정 최종상태 제어기는 어떤 목표 상태에도 정확히 도달하므로{" "}
                        <InlineMath math="x_{\text{new}}"/>은 표본 자체이고{" "}
                        <InlineMath math="\eta"/>-스텝 절단이 없다. SST는 steering을 포기했지만
                        kinodynamic RRT*은 그것을 유지하고 quartic 풀이로 값을 치른다.</li>
                    <li><strong>동역학적으로 실행 가능한 간선</strong>: 모든 간선이 실제
                        double-integrator 궤적이라, 반환 경로는 직선 기하 경로와 달리 관성을
                        가진 물체가 그대로 실행할 수 있다.</li>
                    <li><strong>점근 최적</strong>: 최적제어 비용을 척도로 삼으면 RRT*의
                        choose-parent·rewire 장치가 표본이 늘수록 최소 비용 궤적으로 수렴한다
                        (Webb &amp; van den Berg 2013, Karaman &amp; Frazzoli 2011).</li>
                    <li><strong>반복당 비용</strong>: 후보 연결마다 quartic 근 풀이 한 번.
                        exact 비용 비교 전에 유클리드로 <InlineMath math="K"/>개를 선별하고
                        근방도 <InlineMath math="K"/>개로 제한하므로, 밀집 트리에서도 작업량이
                        유계로 남는다.</li>
                    <li><strong>선형 동역학 한정</strong>: 닫힌 형태는{" "}
                        <InlineMath math="x' = Ax + Bu"/>를 요구한다. 비선형 동역학은 선형화나
                        수치 two-point 경계값 풀이가 필요한데, 그것이 바로 SST가 아예 steering을
                        하지 않아 피해 가는 비용이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop is RRT* verbatim, read in the cost geometry of the dynamics: sample
                    a full state, find the cheapest reachable parent, attach, rewire, and keep
                    steering toward the goal:
                </p>}
                ko={<p>
                    루프는 RRT*을 그대로이되 동역학의 비용 기하에서 읽는다. 완전한 상태를
                    표본하고, 가장 싸게 닿는 부모를 찾고, 붙이고, 재배선하고, goal로 계속
                    steering 한다:
                </p>}
            />
            <Pseudocode code={`V ← {x_start (at rest)};  best ← ∞
repeat max_iterations times:
    x_rand ← goal rest-state w.p. p, else (random position, random velocity)  # 1
    x_sel ← node of min optimal cost c*(·, x_rand) among the K nearest        # 2
    x_new ← x_rand           # optimal steering reaches the sample exactly
    N ← tree nodes within neighbor_radius of x_new                            # 3
    attach x_new to argmin over N ∪ {x_sel} of cost(v) + c*(v, x_new)         # 4
    for v in N: if cost(x_new) + c*(x_new, v) < cost(v): rewire v under x_new # 5
    if x_new within goal_tol of goal and steering to goal beats best:         # 6
        best ← that cost;  publish the improved trajectory`}/>
            <T
                en={<ol>
                    <li>A full 4D sample. Unlike geometric RRT* the sample carries a random
                        velocity in <InlineMath math="[-v_{\max}, v_{\max}]^2"/>, not just a
                        position; the goal is drawn as its rest-state (zero velocity).</li>
                    <li>Nearest by optimal cost, not distance: the metric is the connection cost{" "}
                        <InlineMath math="c^\ast"/>. Solving it against every node each iteration
                        is too slow, so a Euclidean prefilter keeps the <InlineMath math="K"/>{" "}
                        closest and the exact minimiser is chosen among them.</li>
                    <li>The neighbourhood uses a cheap position radius as a proxy, capped to the{" "}
                        <InlineMath math="K"/> nearest so per-iteration work stays bounded.</li>
                    <li>Choose-parent in cost geometry. Each candidate edge is the optimal
                        trajectory, and feasibility means the curved trajectory, densified into
                        waypoints, is collision free along its whole length.</li>
                    <li>Rewire is RRT*&apos;s, but the edges are optimal trajectories. A rewire
                        re-propagates the cumulative cost down the subtree without re-solving any
                        steering, since a rewired ancestor changes only the accumulated sums.</li>
                    <li>The goal is never a tree node. Whenever a new node lands within{" "}
                        <InlineMath math="\text{goal\_tol}"/> of the goal, it is steered to the
                        goal rest-state and the cheapest collision-free arrival is kept. The loop
                        runs its full budget, refining the incumbent (anytime).</li>
                </ol>}
                ko={<ol>
                    <li>완전한 4D 표본이다. 기하 RRT*과 달리 표본은 위치뿐 아니라{" "}
                        <InlineMath math="[-v_{\max}, v_{\max}]^2"/>의 랜덤 속도를 함께 싣는다.
                        goal은 정지 상태(속도 0)로 뽑힌다.</li>
                    <li>거리가 아니라 최적 비용으로 nearest를 고른다. 척도는 연결 비용{" "}
                        <InlineMath math="c^\ast"/>이다. 매 반복 전 노드에 이를 푸는 것은 너무
                        느려, 유클리드로 <InlineMath math="K"/>개를 선별한 뒤 그 안에서 exact
                        최소를 택한다.</li>
                    <li>근방은 값싼 위치 반경을 대리로 쓰되 <InlineMath math="K"/>개로 제한해
                        반복당 작업량을 유계로 둔다.</li>
                    <li>비용 기하에서의 choose-parent다. 각 후보 간선은 최적 궤적이고, 실행
                        가능성은 그 곡선 궤적을 waypoint로 촘촘히 나눈 전 구간이 충돌 없음을
                        뜻한다.</li>
                    <li>rewire는 RRT*의 그것이되 간선이 최적 궤적이다. 조상을 rewire 해도 누적
                        합만 바뀌므로, 재배선은 steering을 다시 풀지 않고 누적 비용을 부분
                        트리로 밀어내린다.</li>
                    <li>goal은 결코 트리 노드가 아니다. 새 노드가 goal의{" "}
                        <InlineMath math="\text{goal\_tol}"/> 안에 들 때마다 goal 정지 상태로
                        steering 해 가장 싼 충돌 없는 도착을 남긴다. 루프는 예산을 다 써 현직
                        해를 다듬는다 (anytime).</li>
                </ol>}
            />
            <Proof title={t("Why a finite optimal arrival time exists", "유한한 최적 도달 시간이 존재하는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Take two states with a nonzero position gap,
                            so <InlineMath math="C_3 = \sum_{\text{axes}} 12\,a^2 > 0"/>, and note{" "}
                            <InlineMath math="C_1 = \sum_{\text{axes}} 4\,(v_0^2 + v_0 v_1 + v_1^2) \ge 0"/>{" "}
                            because each summand is a perfect-square form. Study{" "}
                            <InlineMath math="c(t)"/> on <InlineMath math="(0, \infty)"/>.
                        </p>
                        <p>
                            As <InlineMath math="t \to 0^+"/> the <InlineMath math="1/t^3"/> term
                            outgrows the others, and its coefficient is positive:
                        </p>
                        <BlockMath math="c(t) \;\ge\; \frac{r\,C_3}{t^3} - \frac{r\,\lvert C_2\rvert}{t^2} - \frac{r\,C_1}{t} \;\xrightarrow[t \to 0^+]{}\; +\infty"/>
                        <Terms items={[
                            ["c(t)", "arrival cost at fixed final time t"],
                            ["r", "the effort penalty, r > 0"],
                            ["C_1", <>velocity-mismatch coefficient, <InlineMath math="\ge 0"/></>],
                            ["C_2", "position-velocity coupling coefficient, any sign"],
                            ["C_3", <>position-gap coefficient, <InlineMath math="> 0"/> here</>],
                            ["t", "the arrival time"],
                        ]}/>
                        <p>
                            As <InlineMath math="t \to \infty"/> the bracket vanishes and the
                            linear term dominates:
                        </p>
                        <BlockMath math="c(t) \;=\; t + r\!\left(\frac{C_3}{t^3} + \frac{C_2}{t^2} + \frac{C_1}{t}\right) \;=\; t + o(1) \;\xrightarrow[t \to \infty]{}\; +\infty"/>
                        <Terms items={[
                            ["c(t)", "arrival cost at fixed final time t"],
                            ["t", "the linear time term, which dominates as t grows"],
                            ["o(1)", <>the bracketed terms, each <InlineMath math="\to 0"/> as <InlineMath math="t \to \infty"/></>],
                            ["r, C_1, C_2, C_3", "the effort penalty and the three cost coefficients as above"],
                        ]}/>
                        <p>
                            <InlineMath math="c"/> is continuous on{" "}
                            <InlineMath math="(0, \infty)"/> and diverges at both ends, so it
                            attains a global minimum at some interior{" "}
                            <InlineMath math="\tau^\ast > 0"/>, where{" "}
                            <InlineMath math="c'(\tau^\ast) = 0"/>. Multiplying{" "}
                            <InlineMath math="c'(t) = 0"/> by <InlineMath math="t^4"/> gives the
                            quartic, so <InlineMath math="\tau^\ast"/> is one of its positive real
                            roots, and the planner selects the root of least{" "}
                            <InlineMath math="c"/>. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 위치 격차가 0이 아닌 두 상태를 잡으면{" "}
                            <InlineMath math="C_3 = \sum_{\text{axes}} 12\,a^2 > 0"/>이고,{" "}
                            <InlineMath math="C_1 = \sum_{\text{axes}} 4\,(v_0^2 + v_0 v_1 + v_1^2) \ge 0"/>{" "}
                            이다. 각 항이 완전제곱 형태이기 때문이다.{" "}
                            <InlineMath math="c(t)"/>를 <InlineMath math="(0, \infty)"/>에서
                            살핀다.
                        </p>
                        <p>
                            <InlineMath math="t \to 0^+"/>일 때 <InlineMath math="1/t^3"/> 항이
                            나머지를 압도하고 그 계수는 양수다:
                        </p>
                        <BlockMath math="c(t) \;\ge\; \frac{r\,C_3}{t^3} - \frac{r\,\lvert C_2\rvert}{t^2} - \frac{r\,C_1}{t} \;\xrightarrow[t \to 0^+]{}\; +\infty"/>
                        <Terms items={[
                            ["c(t)", "고정 최종시간 t의 도달 비용"],
                            ["r", "제어 페널티, r > 0"],
                            ["C_1", <>속도 불일치 계수, <InlineMath math="\ge 0"/></>],
                            ["C_2", "위치·속도 결합 계수, 부호 임의"],
                            ["C_3", <>위치 격차 계수, 여기서는 <InlineMath math="> 0"/></>],
                            ["t", "도달 시간"],
                        ]}/>
                        <p>
                            <InlineMath math="t \to \infty"/>일 때 괄호는 사라지고 선형 항이
                            지배한다:
                        </p>
                        <BlockMath math="c(t) \;=\; t + r\!\left(\frac{C_3}{t^3} + \frac{C_2}{t^2} + \frac{C_1}{t}\right) \;=\; t + o(1) \;\xrightarrow[t \to \infty]{}\; +\infty"/>
                        <Terms items={[
                            ["c(t)", "고정 최종시간 t의 도달 비용"],
                            ["t", "t가 커질수록 지배하는 선형 시간 항"],
                            ["o(1)", <>괄호 항들. <InlineMath math="t \to \infty"/>에서 각각 <InlineMath math="\to 0"/></>],
                            ["r, C_1, C_2, C_3", "위와 같은 제어 페널티와 세 비용 계수"],
                        ]}/>
                        <p>
                            <InlineMath math="c"/>는 <InlineMath math="(0, \infty)"/>에서
                            연속이고 양 끝에서 발산하므로, 내부의 어떤{" "}
                            <InlineMath math="\tau^\ast > 0"/>에서 전역 최솟값을 가지며 거기서{" "}
                            <InlineMath math="c'(\tau^\ast) = 0"/>이다.{" "}
                            <InlineMath math="c'(t) = 0"/>에 <InlineMath math="t^4"/>을 곱하면
                            quartic이 되므로 <InlineMath math="\tau^\ast"/>은 그 양의 실근 중
                            하나이고, planner는 <InlineMath math="c"/>가 가장 작은 근을 고른다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox grows one tree around a diagonal slash wall welded to the
                    top-left corner, so the only way through is to sweep around its free tip in
                    the middle. Because the double integrator carries momentum, the car rounds
                    that tip in a wide arc rather than a sharp corner. Raise the effort penalty{" "}
                    <InlineMath math="r"/> from 0.3 to 3.0 and the control cost climbs (about 30
                    up to 56 at seed 1) as the trajectories smooth out and slow down: momentum
                    is no longer free. The replay below is the repository demo on the benchmark
                    grids, where every edge is an optimal double-integrator trajectory and
                    rewires re-route the tree through cheaper arrivals.
                </p>}
                ko={<p>
                    sandbox는 왼쪽 위 모서리에 붙은 대각 slash 벽 하나를 도는 트리를 키운다.
                    유일한 통로는 가운데 자유 끝을 감아 도는 것뿐이다. double integrator가
                    관성을 실으므로 차는 그 끝을 날카로운 모서리가 아니라 넓은 호로 돈다. 제어
                    페널티 <InlineMath math="r"/>을 0.3에서 3.0으로 올리면 궤적이 완만해지고
                    느려지면서 제어 비용이 오른다(seed 1에서 약 30에서 56으로). 관성이 더는
                    공짜가 아니다. 아래 replay는 벤치마크 grid 위 저장소 demo다. 모든 간선은
                    최적 double-integrator 궤적이고, rewire는 트리를 더 싼 도착으로 다시
                    돌린다.
                </p>}
            />
            <KinodynamicSandbox/>
            <TraceReplay vehicle algo="kinodynamic_rrt_star" maps={["maze01", "open01"]} label={t(
                "Real traces from the repository's Kinodynamic RRT* demo — every edge is an optimal double-integrator trajectory, and the car drives the momentum-respecting path it returns",
                "저장소 Kinodynamic RRT* demo의 실제 trace. 모든 간선이 최적 double-integrator 궤적이고, 차는 그것이 반환한 관성을 존중하는 경로를 주행한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The planner owns its double-integrator dynamics and the steering closed form
                    (quartic solve for <InlineMath math="\tau^\ast"/>, cubic Hermite for the
                    trajectory) in parallel node arrays, since the shared sampling tree models
                    Euclidean edges only. The map is queried purely on the{" "}
                    <InlineMath math="(x, y)"/> projection. Both language versions are embedded in
                    full below.
                </p>}
                ko={<p>
                    planner는 double-integrator 동역학과 steering 닫힌 형태(<InlineMath math="\tau^\ast"/>{" "}
                    quartic 풀이, 궤적용 3차 Hermite)를 병렬 노드 배열로 직접 소유한다. 공유
                    sampling 트리는 유클리드 간선만 모델링하기 때문이다. 맵은 오직{" "}
                    <InlineMath math="(x, y)"/> 투영으로만 질의된다. 두 언어 버전 전체를 아래에
                    embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/kinodynamic_rrt_star.py",
                            code: kinoPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/kinodynamic_rrt_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/kinodynamic_rrt_star.cpp",
                            code: kinoCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/kinodynamic_rrt_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Kinodynamic RRT* implementation, embedded from the repository sources",
                    "Kinodynamic RRT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    D. J. Webb, J. van den Berg,{" "}
                    <a href="https://doi.org/10.1109/ICRA.2013.6631299" target="_blank"
                       rel="noopener noreferrer">
                        <em>Kinodynamic RRT*: Asymptotically Optimal Motion Planning for Robots
                            with Linear Dynamics</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2013.
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
                    S. M. LaValle, J. J. Kuffner,{" "}
                    <a href="https://doi.org/10.1177/02783640122067453" target="_blank"
                       rel="noopener noreferrer">
                        <em>Randomized Kinodynamic Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2001.
                </li>
            </ol>
        </>
    )
}

export default KinodynamicRrtStar
