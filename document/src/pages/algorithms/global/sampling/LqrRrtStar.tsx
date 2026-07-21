import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import LqrRrtStarSandbox from "../../../../components/panels/global/lqr_rrt_star/LqrRrtStarSandbox";
import lqrPy from "../../../../../../python/navigation/global_planning/sampling/lqr_rrt_star.py?raw";
import lqrCpp from "../../../../../../cpp/src/global_planning/sampling/lqr_rrt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const LqrRrtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    SST on the previous page threw the steering function away and grew the
                    tree by firing <em>random</em> controls forward. That keeps planning
                    possible under dynamics, but it wastes the one thing RRT* is good at:
                    connecting a new state to the <em>best</em> nearby node. To rewire, RRT*
                    needs two hand-designed primitives it takes for granted on a geometric
                    map, a distance metric and a steering function, and for a system with
                    dynamics both are hard to write down. LQR-RRT* (Perez, Platt, Konidaris,
                    Kaelbling &amp; Lozano-Pérez, 2012) neither hand-designs them nor solves
                    an exact two-point boundary-value problem. It linearises the dynamics,
                    picks a quadratic cost, and lets the <em>Linear-Quadratic Regulator</em>{" "}
                    derive both automatically: the metric becomes the LQR cost-to-go, and
                    steering becomes the LQR feedback policy rolled forward. One Riccati solve
                    at construction, and the full RRT* rewiring machinery runs on top.
                </p>}
                ko={<p>
                    앞 페이지의 SST는 steering function을 버리고 <em>랜덤</em> control을
                    정방향으로 쏴 트리를 키웠다. 그러면 동역학 아래에서도 계획이 되지만,
                    RRT*이 가장 잘하는 것 하나를 버린다. 새 상태를 근방의 <em>가장 나은</em>{" "}
                    노드에 잇는 일이다. rewire하려면 RRT*은 기하 맵에서는 당연하게 여기는 손으로
                    설계한 두 원시 연산, 거리 metric과 steering function이 필요한데, 동역학이
                    있는 계에서는 둘 다 적기 어렵다. LQR-RRT*(Perez, Platt, Konidaris,
                    Kaelbling &amp; Lozano-Pérez, 2012)는 이 둘을 손으로 짜지도, 정확한
                    two-point 경계값 문제를 풀지도 않는다. 동역학을 선형화하고 이차 비용을 골라,{" "}
                    <em>Linear-Quadratic Regulator</em>가 둘을 자동으로 끌어내게 한다. metric은
                    LQR cost-to-go가 되고, steering은 정방향으로 굴린 LQR 피드백 정책이 된다.
                    구성 시 Riccati를 한 번 풀면, 그 위에서 RRT* rewire 기구 전체가 돈다.
                </p>}
            />

            <h2>{t("Steering Derived from a Regulator", "조절기가 steering을 만든다")}</h2>
            <T
                en={<>
                    <p>
                        This repository runs LQR-RRT* on a 2D double integrator, the same
                        system its kinodynamic RRT* uses so the two compare on one benchmark.
                        A planning state is <InlineMath math="(x, y, v_x, v_y)"/>, the control
                        is acceleration, and the two axes are decoupled and identical, so
                        everything below is a per-axis <InlineMath math="2\times 2"/> problem
                        with <InlineMath math="\dot{x} = Ax + Bu"/>. The map is queried only on
                        the <InlineMath math="(x, y)"/> projection through the same{" "}
                        <InlineMath math="\text{SamplingSpace}"/> the RRT family uses; the
                        planner owns its dynamics. Given a quadratic cost{" "}
                        <InlineMath math="J = \int (x^\top Q x + u^\top R u)\, dt"/>, the LQR
                        solution hands us both extension heuristics at once:
                    </p>
                    <BlockMath math="\mathrm{dist}(a, b) = (a - b)^\top S\,(a - b), \qquad\qquad u = -K\,(x - x_{\text{ref}}), \quad K = (R + B^\top P B)^{-1} B^\top P A"/>
                    <Terms items={[
                        ["a, b", <>two full states <InlineMath math="(x, y, v_x, v_y)"/> whose distance is being ranked</>],
                        ["x", "the current state driven by the feedback law (per axis, the pair position/velocity)"],
                        ["x_{\\text{ref}}", "the reference the regulator steers toward — here a rest waypoint (velocity zero)"],
                        ["u", "the control (acceleration) the feedback law commands"],
                        ["A, B", <>the (discretised) system matrices of the double integrator, <InlineMath math="\dot{x}=Ax+Bu"/></>],
                        ["Q, R", <>the state and control cost weights of <InlineMath math="J"/>; here <InlineMath math="Q=\mathrm{diag}(q_{\text{pos}}, q_{\text{vel}})"/> and <InlineMath math="R=r\,I"/></>],
                        ["P", <><strong>the new term</strong>: the solution of the Riccati equation below — the quadratic value-function matrix</>],
                        ["S", <><strong>the new term</strong>: the steady-state cost-to-go matrix (the converged <InlineMath math="P"/>), which is exactly the distance metric</>],
                        ["K", <><strong>the new term</strong>: the LQR feedback gain; rolling <InlineMath math="u=-K(x-x_{\text{ref}})"/> forward is the steering primitive</>],
                    ]}/>
                    <p>
                        Both <InlineMath math="S"/> and <InlineMath math="K"/> come from the
                        same object, the matrix <InlineMath math="P"/> that solves the discrete
                        algebraic Riccati equation. Because the system is linear
                        time-invariant and <InlineMath math="Q"/> is diagonal,{" "}
                        <InlineMath math="P"/> is state-independent and shared by both axes, so
                        it is computed <em>once</em> at construction as the fixed point of the
                        recursion:
                    </p>
                    <BlockMath math="P \;\leftarrow\; Q + A^\top P A - A^\top P B\,(R + B^\top P B)^{-1} B^\top P A"/>
                    <Terms items={[
                        ["P", "the Riccati matrix being iterated; started at Q and driven to its fixed point"],
                        ["Q", <>the state cost weight (position/velocity penalty), <InlineMath math="Q \succ 0"/></>],
                        ["R", <>the control cost weight, <InlineMath math="R \succ 0"/></>],
                        ["A, B", "the discretised double-integrator system matrices"],
                        ["A^\\top P B\\,(R + B^\\top P B)^{-1} B^\\top P A", <><strong>the new term</strong>: the control-feedback correction subtracted each step, which is what makes the fixed point stabilising rather than the open-loop Lyapunov equation</>],
                    ]}/>
                    <p>
                        The distance metric is now <em>direction-aware</em>: unlike a Euclidean
                        norm, <InlineMath math="(a-b)^\top S (a-b)"/> charges more for a
                        velocity difference the dynamics cannot cheaply undo, so nearest-node
                        selection ranks by true cost-to-go rather than raw position gap. And a
                        stored edge is a genuine trajectory: the feedback regulates to a rest
                        waypoint, and because the closed loop is stable it reaches that rest
                        state exactly, so every parent/child join is collision-free and
                        dynamically feasible. That exactness is what keeps RRT*&apos;s rewiring
                        valid. The price, relative to an exact kinodynamic two-point solve, is
                        that the regulator converges only asymptotically, so a roll is capped
                        at a fixed horizon and rejected if it neither arrives nor stays free.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 저장소는 LQR-RRT*을 2D double integrator에서 돌린다. kinodynamic
                        RRT*이 쓰는 것과 같은 계라, 둘이 한 벤치마크에서 비교된다. 계획 상태는{" "}
                        <InlineMath math="(x, y, v_x, v_y)"/>, control은 가속도이고, 두 축은
                        분리되어 동일하므로 아래는 모두 축별{" "}
                        <InlineMath math="2\times 2"/> 문제(<InlineMath math="\dot{x} = Ax + Bu"/>)다.
                        맵은 <InlineMath math="(x, y)"/> 투영에 대해서만, RRT 계열이 쓰는 그{" "}
                        <InlineMath math="\text{SamplingSpace}"/>로 질의된다. 동역학은 planner가
                        소유한다. 이차 비용 <InlineMath math="J = \int (x^\top Q x + u^\top R u)\, dt"/>이
                        주어지면, LQR 해가 확장 heuristic 둘을 한꺼번에 건넨다:
                    </p>
                    <BlockMath math="\mathrm{dist}(a, b) = (a - b)^\top S\,(a - b), \qquad\qquad u = -K\,(x - x_{\text{ref}}), \quad K = (R + B^\top P B)^{-1} B^\top P A"/>
                    <Terms items={[
                        ["a, b", <>거리를 재려는 두 전체 상태 <InlineMath math="(x, y, v_x, v_y)"/></>],
                        ["x", "피드백 법칙이 모는 현재 상태 (축별로 위치/속도 쌍)"],
                        ["x_{\\text{ref}}", "조절기가 향하는 기준점. 여기서는 rest waypoint(속도 0)"],
                        ["u", "피드백 법칙이 명령하는 control(가속도)"],
                        ["A, B", <>double integrator의 (이산화된) 시스템 행렬, <InlineMath math="\dot{x}=Ax+Bu"/></>],
                        ["Q, R", <><InlineMath math="J"/>의 상태·제어 비용 가중. 여기서 <InlineMath math="Q=\mathrm{diag}(q_{\text{pos}}, q_{\text{vel}})"/>, <InlineMath math="R=r\,I"/></>],
                        ["P", <><strong>새로 추가된 항</strong>: 아래 Riccati 방정식의 해. 이차 가치 함수 행렬</>],
                        ["S", <><strong>새로 추가된 항</strong>: 정상 상태 cost-to-go 행렬(수렴한 <InlineMath math="P"/>). 이것이 곧 거리 metric이다</>],
                        ["K", <><strong>새로 추가된 항</strong>: LQR 피드백 이득. <InlineMath math="u=-K(x-x_{\text{ref}})"/>를 정방향으로 굴리는 것이 steering 원시 연산이다</>],
                    ]}/>
                    <p>
                        <InlineMath math="S"/>와 <InlineMath math="K"/>는 같은 대상, 이산
                        대수 Riccati 방정식을 푸는 행렬 <InlineMath math="P"/>에서 나온다. 계가
                        선형 시불변이고 <InlineMath math="Q"/>가 대각이라{" "}
                        <InlineMath math="P"/>는 상태 독립이고 두 축이 공유하므로, 구성 시{" "}
                        <em>한 번</em> 다음 반복의 고정점으로 계산된다:
                    </p>
                    <BlockMath math="P \;\leftarrow\; Q + A^\top P A - A^\top P B\,(R + B^\top P B)^{-1} B^\top P A"/>
                    <Terms items={[
                        ["P", "반복되는 Riccati 행렬. Q에서 출발해 고정점으로 몰린다"],
                        ["Q", <>상태 비용 가중(위치/속도 페널티), <InlineMath math="Q \succ 0"/></>],
                        ["R", <>제어 비용 가중, <InlineMath math="R \succ 0"/></>],
                        ["A, B", "이산화된 double integrator 시스템 행렬"],
                        ["A^\\top P B\\,(R + B^\\top P B)^{-1} B^\\top P A", <><strong>새로 추가된 항</strong>: 매 스텝 빼는 제어 피드백 보정. 이 항이 고정점을 개루프 Lyapunov 방정식이 아니라 안정화하는 해로 만든다</>],
                    ]}/>
                    <p>
                        이제 거리 metric은 <em>방향을 안다</em>. 유클리드 norm과 달리{" "}
                        <InlineMath math="(a-b)^\top S (a-b)"/>은 동역학이 값싸게 되돌릴 수 없는
                        속도 차이에 더 큰 값을 매기므로, nearest 노드 선택이 위치 격차가 아니라
                        참 cost-to-go로 순위를 매긴다. 그리고 저장된 간선은 진짜 궤적이다.
                        피드백이 rest waypoint로 조절하고, 폐루프가 안정하므로 그 rest 상태에
                        정확히 도달해, 모든 부모/자식 접합이 충돌 없이 동역학적으로 실현 가능하다.
                        이 정확성이 RRT*의 rewire를 유효하게 유지한다. 정확한 kinodynamic
                        two-point 해에 견준 대가는, 조절기가 점근적으로만 수렴한다는 점이다.
                        그래서 roll은 고정 지평선에서 잘리고, 도달하지도 자유롭지도 못하면
                        기각된다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Extension heuristics for free</strong>: the nearest-neighbour
                        metric and the steering primitive are both derived from one Riccati
                        solve, so no bespoke distance or <InlineMath math="\text{steer}()"/> is
                        written by hand for the dynamics.</li>
                    <li><strong>Direction-aware nearest</strong>: ranking by{" "}
                        <InlineMath math="(a-b)^\top S(a-b)"/> accounts for velocity, unlike a
                        position-only Euclidean metric, so a node that is close but moving the
                        wrong way is correctly ranked as expensive.</li>
                    <li><strong>Feasible, exact edges</strong>: each stored edge regulates to a
                        rest equilibrium and reaches it (the closed loop{" "}
                        <InlineMath math="A - BK"/> is stable), so RRT*&apos;s choose-parent and
                        rewire stay exact rather than approximate.</li>
                    <li><strong>Asymptotic, not exact, connection</strong>: unlike kinodynamic
                        RRT*&apos;s fixed-final-state two-point solve, an LQR roll converges only
                        asymptotically and is capped at a horizon; rolls that do not arrive are
                        rejected.</li>
                    <li><strong>Cost per iteration</strong>: a Euclidean prefilter to the{" "}
                        <InlineMath math="k"/> nearest, then an exact LQR-metric compare, a
                        bounded choose-parent / rewire neighbourhood, and one forward roll per
                        candidate edge. The Riccati solve is one-time, not per iteration.</li>
                </ul>}
                ko={<ul>
                    <li><strong>확장 heuristic이 공짜</strong>: nearest metric과 steering
                        원시 연산이 모두 Riccati 한 번에서 나오므로, 동역학용 거리나{" "}
                        <InlineMath math="\text{steer}()"/>를 손으로 짤 필요가 없다.</li>
                    <li><strong>방향 인지 nearest</strong>: <InlineMath math="(a-b)^\top S(a-b)"/>로
                        순위를 매기면 위치만 보는 유클리드 metric과 달리 속도를 셈에 넣어,
                        가깝지만 반대로 움직이는 노드를 올바르게 비싼 것으로 판정한다.</li>
                    <li><strong>실현 가능한 정확한 간선</strong>: 저장된 각 간선은 rest 평형점으로
                        조절되어 그리로 도달하므로(폐루프 <InlineMath math="A - BK"/>가 안정),
                        RRT*의 choose-parent와 rewire가 근사가 아니라 정확하게 유지된다.</li>
                    <li><strong>정확이 아니라 점근 연결</strong>: kinodynamic RRT*의
                        고정 종단 상태 two-point 해와 달리, LQR roll은 점근적으로만 수렴하고
                        지평선에서 잘린다. 도달하지 못한 roll은 기각된다.</li>
                    <li><strong>반복당 비용</strong>: <InlineMath math="k"/>개 nearest로 가는
                        유클리드 prefilter 뒤 정확한 LQR-metric 비교, 유계인 choose-parent /
                        rewire 근방, 후보 간선당 정방향 roll 하나. Riccati 해는 반복마다가 아니라
                        한 번뿐이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop is RRT* with the geometric metric and straight-line steer swapped
                    for the LQR ones. Sample a full state, find the nearest node under the
                    cost-to-go metric, roll the feedback toward a rest waypoint, then run the
                    ordinary choose-parent and rewire using LQR rolls as the edges:
                </p>}
                ko={<p>
                    루프는 RRT*에서 기하 metric과 직선 steer를 LQR 것으로 바꾼 것이다. 전체
                    상태를 뽑고, cost-to-go metric으로 nearest 노드를 찾고, 피드백을 rest
                    waypoint 쪽으로 굴린 뒤, LQR roll을 간선으로 삼아 보통의 choose-parent와
                    rewire를 돌린다:
                </p>}
            />
            <Pseudocode code={`solve Riccati once → S (metric), K (feedback)      # 0
tree ← {start at rest};  best_goal ← ∞
repeat max_iterations times:
    q ← goal-rest with probability p, else free position + random velocity  # 1
    near ← argmin over k-Euclidean-nearest of (q−·)ᵀ S (q−·)                # 2
    x_new ← rest waypoint ≤ step_size from near toward q                    # 3
    edge ← roll u = −K(x − x_new) forward;  skip if it never arrives / hits # 4
    choose-parent: attach x_new to the min-cost neighbour whose roll reaches it  # 5
    rewire: for each neighbour, if routing through x_new is cheaper, re-roll and reparent  # 6
    if x_new within goal_tol: roll x_new → goal;  if cheaper, publish path  # 7`}/>
            <T
                en={<ol>
                    <li>The Riccati solve happens once, before the loop. Its outputs{" "}
                        <InlineMath math="S"/> and <InlineMath math="K"/> are reused by every
                        metric compare and every roll for the whole run.</li>
                    <li>A sample is a <em>full</em> state: the goal (at rest) with probability{" "}
                        <InlineMath math="p"/>, otherwise a free position with a random
                        velocity, so the cost-to-go metric ranks nodes on all four
                        coordinates.</li>
                    <li>Nearest is a two-stage compare: a cheap Euclidean prefilter to the{" "}
                        <InlineMath math="k"/> closest positions, then the exact LQR metric{" "}
                        <InlineMath math="(q-\cdot)^\top S (q-\cdot)"/> among those.</li>
                    <li>The extension target is a rest waypoint at most{" "}
                        <InlineMath math="\text{step\_size}"/> from the near node toward the
                        sample, its velocity set to zero so the feedback has a true
                        equilibrium to settle on.</li>
                    <li>The roll integrates <InlineMath math="u=-K(x-x_{\text{new}})"/> with
                        the control clamped, collision-checking each chord, and is discarded if
                        it neither reaches the waypoint within the horizon nor stays free.</li>
                    <li>Choose-parent is ordinary RRT*: among the neighbourhood, attach{" "}
                        <InlineMath math="x_{\text{new}}"/> to whichever node&apos;s roll reaches
                        it at least total cost, defaulting to the already-rolled near node.</li>
                    <li>Rewire re-rolls from <InlineMath math="x_{\text{new}}"/> to each
                        neighbour and reparents any that become cheaper, pushing the cost delta
                        through the whole subtree so descendants stay exact.</li>
                    <li>The goal is not a growth node: whenever a new node lands within{" "}
                        <InlineMath math="\text{goal\_tol}"/>, one more roll to the goal rest
                        state is attempted, and the incumbent path is replaced only if it
                        improves — the loop runs its full budget refining.</li>
                </ol>}
                ko={<ol>
                    <li>Riccati 해는 루프 전에 한 번 이뤄진다. 그 산출물{" "}
                        <InlineMath math="S"/>와 <InlineMath math="K"/>는 실행 내내 모든 metric
                        비교와 모든 roll에 재사용된다.</li>
                    <li>표본은 <em>전체</em> 상태다. 확률 <InlineMath math="p"/>로 goal(정지),
                        아니면 자유 위치에 랜덤 속도를 실어, cost-to-go metric이 네 좌표 모두로
                        노드 순위를 매기게 한다.</li>
                    <li>nearest는 두 단계 비교다. 위치가 가장 가까운{" "}
                        <InlineMath math="k"/>개로 가는 값싼 유클리드 prefilter, 그 안에서
                        정확한 LQR metric <InlineMath math="(q-\cdot)^\top S (q-\cdot)"/>.</li>
                    <li>확장 목표는 near 노드에서 표본 쪽으로 최대{" "}
                        <InlineMath math="\text{step\_size}"/> 떨어진 rest waypoint다. 속도를
                        0으로 두어 피드백이 안착할 진짜 평형점이 되게 한다.</li>
                    <li>roll은 control을 clamp한 채{" "}
                        <InlineMath math="u=-K(x-x_{\text{new}})"/>를 적분하며 chord마다 충돌을
                        검사하고, 지평선 안에 waypoint에 닿지도 자유 상태로 남지도 못하면
                        버린다.</li>
                    <li>choose-parent는 보통의 RRT*이다. 근방에서{" "}
                        <InlineMath math="x_{\text{new}}"/>에 roll이 닿는 노드 중 총비용이 가장
                        작은 곳에 붙이고, 기본값은 이미 굴린 near 노드다.</li>
                    <li>rewire는 <InlineMath math="x_{\text{new}}"/>에서 각 근방 노드로 다시
                        굴려 더 싸지는 것을 재부모하고, 비용 delta를 부분 트리 전체로 밀어 후손이
                        정확하게 유지되게 한다.</li>
                    <li>goal은 성장 노드가 아니다. 새 노드가{" "}
                        <InlineMath math="\text{goal\_tol}"/> 안에 들 때마다 goal 정지 상태로
                        가는 roll을 한 번 더 시도하고, 개선될 때만 현직 경로를 교체한다. 루프는
                        예산을 다 써 다듬는다.</li>
                </ol>}
            />
            <Proof title={t("Why the feedback reaches the rest waypoint", "피드백이 rest waypoint에 도달하는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Let <InlineMath math="S \succ 0"/> solve
                            the discrete Riccati equation, and write the error from the rest
                            waypoint as <InlineMath math="e = x - x_{\text{ref}}"/>, so the
                            feedback is <InlineMath math="u = -Ke"/> and the closed loop is{" "}
                            <InlineMath math="e^{+} = (A - BK)\,e"/>. Take the candidate
                            Lyapunov function <InlineMath math="V(e) = e^\top S e"/>. The
                            Riccati identity can be written in closed-loop form:
                        </p>
                        <BlockMath math="S = Q + K^\top R K + (A - BK)^\top S\,(A - BK)"/>
                        <Terms items={[
                            ["S", "the Riccati solution, used both as the metric and as the Lyapunov matrix"],
                            ["e", <>the error state <InlineMath math="x - x_{\text{ref}}"/> from the rest waypoint</>],
                            ["A - BK", "the closed-loop system matrix under the LQR feedback"],
                            ["Q, R", <>the state and control weights, with <InlineMath math="Q \succ 0"/>, <InlineMath math="R \succ 0"/></>],
                            ["V(e) = e^\\top S e", <><strong>the new term</strong>: the candidate Lyapunov function, equal to the cost-to-go of the error</>],
                        ]}/>
                        <p>
                            Substituting the closed-loop update into the one-step change of{" "}
                            <InlineMath math="V"/> and using that identity:
                        </p>
                        <BlockMath math="V(e^{+}) - V(e) = e^\top\!\big[(A-BK)^\top S (A-BK) - S\big] e = -\,e^\top (Q + K^\top R K)\, e"/>
                        <Terms items={[
                            ["V(e^{+}) - V(e)", "the change in the Lyapunov value over one closed-loop step"],
                            ["Q + K^\\top R K", "the total per-step penalty on the error, combining state and control cost"],
                        ]}/>
                        <p>
                            Since <InlineMath math="Q \succ 0"/> and{" "}
                            <InlineMath math="K^\top R K \succeq 0"/>, the right-hand side is
                            strictly negative for every <InlineMath math="e \neq 0"/>:
                        </p>
                        <BlockMath math="V(e^{+}) - V(e) \;\le\; -\,e^\top Q\, e \;<\; 0 \quad (e \neq 0)"/>
                        <Terms items={[
                            ["e^\\top Q\\, e", <>the position/velocity penalty alone, positive whenever <InlineMath math="e \neq 0"/></>],
                        ]}/>
                        <p>
                            So <InlineMath math="V"/> strictly decreases every step and is
                            bounded below by 0, forcing <InlineMath math="e \to 0"/>: the
                            feedback drives the state to the rest waypoint. The roll therefore
                            terminates at the waypoint (up to the reach tolerance), which is
                            why each stored edge is a real, dynamically-feasible trajectory and
                            RRT*&apos;s rewiring stays exact. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> <InlineMath math="S \succ 0"/>이 이산
                            Riccati 방정식을 풀고, rest waypoint에서의 오차를{" "}
                            <InlineMath math="e = x - x_{\text{ref}}"/>로 쓰면, 피드백은{" "}
                            <InlineMath math="u = -Ke"/>, 폐루프는{" "}
                            <InlineMath math="e^{+} = (A - BK)\,e"/>이다. 후보 Lyapunov 함수{" "}
                            <InlineMath math="V(e) = e^\top S e"/>를 잡는다. Riccati 항등식은
                            폐루프 형태로 쓸 수 있다:
                        </p>
                        <BlockMath math="S = Q + K^\top R K + (A - BK)^\top S\,(A - BK)"/>
                        <Terms items={[
                            ["S", "Riccati 해. metric이자 Lyapunov 행렬로 쓴다"],
                            ["e", <>rest waypoint에서의 오차 상태 <InlineMath math="x - x_{\text{ref}}"/></>],
                            ["A - BK", "LQR 피드백 아래 폐루프 시스템 행렬"],
                            ["Q, R", <>상태·제어 가중, <InlineMath math="Q \succ 0"/>, <InlineMath math="R \succ 0"/></>],
                            ["V(e) = e^\\top S e", <><strong>새로 추가된 항</strong>: 후보 Lyapunov 함수. 오차의 cost-to-go와 같다</>],
                        ]}/>
                        <p>
                            폐루프 갱신을 <InlineMath math="V"/>의 한 스텝 변화에 대입하고 그
                            항등식을 쓰면:
                        </p>
                        <BlockMath math="V(e^{+}) - V(e) = e^\top\!\big[(A-BK)^\top S (A-BK) - S\big] e = -\,e^\top (Q + K^\top R K)\, e"/>
                        <Terms items={[
                            ["V(e^{+}) - V(e)", "폐루프 한 스텝 동안 Lyapunov 값의 변화"],
                            ["Q + K^\\top R K", "오차에 대한 스텝당 총 페널티. 상태 비용과 제어 비용을 합친 것"],
                        ]}/>
                        <p>
                            <InlineMath math="Q \succ 0"/>이고{" "}
                            <InlineMath math="K^\top R K \succeq 0"/>이므로, 우변은 모든{" "}
                            <InlineMath math="e \neq 0"/>에서 엄밀히 음수다:
                        </p>
                        <BlockMath math="V(e^{+}) - V(e) \;\le\; -\,e^\top Q\, e \;<\; 0 \quad (e \neq 0)"/>
                        <Terms items={[
                            ["e^\\top Q\\, e", <><InlineMath math="e \neq 0"/>이면 양수인 위치/속도 페널티</>],
                        ]}/>
                        <p>
                            따라서 <InlineMath math="V"/>는 매 스텝 엄밀히 줄고 0으로 하계가
                            있어 <InlineMath math="e \to 0"/>을 강제한다. 피드백이 상태를 rest
                            waypoint로 몬다. 그래서 roll은 waypoint에서(도달 tolerance까지)
                            끝나고, 저장된 각 간선이 실제로 동역학적으로 실현 가능한 궤적이라
                            RRT*의 rewire가 정확하게 유지된다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox grows one LQR-RRT* tree through a chevron of offset blocks,
                    with the car driving the found trajectory at the end. The buttons change
                    the control cost <InlineMath math="r"/>: at{" "}
                    <InlineMath math="r = 0.2"/> the regulator spends control freely and reaches
                    each rest waypoint on short, aggressive arcs (LQR cost about 33 at seed 1);
                    at <InlineMath math="r = 5"/> control is expensive, the steering is
                    sluggish, and the same-size tree pays roughly 56 for the same route. That
                    swing is the whole point: the metric and the edges are shaped by the cost
                    you choose, not hand-tuned. The replay below is the repository demo on the
                    benchmark grids, where each edge is a forward-rolled LQR feedback
                    trajectory rather than a straight line.
                </p>}
                ko={<p>
                    sandbox는 어긋난 블록으로 이뤄진 chevron을 지나는 LQR-RRT* 트리 하나를
                    키우고, 끝에서 차량이 찾은 궤적을 주행한다. 버튼은 제어 비용{" "}
                    <InlineMath math="r"/>을 바꾼다. <InlineMath math="r = 0.2"/>이면 조절기가
                    제어를 아끼지 않아 짧고 공격적인 호로 각 rest waypoint에 도달하고(seed 1에서
                    LQR 비용 약 33), <InlineMath math="r = 5"/>이면 제어가 비싸 steering이 굼떠,
                    같은 크기의 트리가 같은 경로에 약 56을 치른다. 그 진폭이 핵심이다. metric과
                    간선이 손으로 맞춘 것이 아니라 고른 비용에 의해 빚어진다. 아래 replay는
                    벤치마크 grid 위 저장소 demo다. 각 간선은 직선이 아니라 정방향으로 굴린 LQR
                    피드백 궤적이다.
                </p>}
            />
            <LqrRrtStarSandbox/>
            <TraceReplay vehicle algo="lqr_rrt_star" maps={["maze01", "open01"]} label={t(
                "Real traces from the repository's LQR-RRT* demo — every edge is an LQR feedback roll to a rest waypoint, and the car drives the refined trajectory",
                "저장소 LQR-RRT* demo의 실제 trace. 모든 간선은 rest waypoint로 가는 LQR 피드백 roll이고, 차량이 다듬어진 궤적을 주행한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The planner solves the discrete Riccati recursion once at construction and
                    owns a tree over rest states in parallel node arrays, each edge storing its
                    realised LQR cost and the dense trajectory of the roll so a rewire
                    propagates cost through a subtree without re-steering. Both language
                    versions are embedded in full below.
                </p>}
                ko={<p>
                    planner는 구성 시 이산 Riccati 반복을 한 번 풀고, rest 상태 위 트리를 병렬
                    노드 배열로 소유한다. 각 간선은 실현 LQR 비용과 roll의 dense 궤적을 저장해,
                    rewire가 re-steer 없이 부분 트리로 비용을 전파한다. 두 언어 버전 전체를
                    아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/lqr_rrt_star.py",
                            code: lqrPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/lqr_rrt_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/lqr_rrt_star.cpp",
                            code: lqrCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/lqr_rrt_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete LQR-RRT* implementation, embedded from the repository sources",
                    "LQR-RRT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    A. Perez, R. Platt, G. Konidaris, L. Kaelbling, T. Lozano-Pérez,{" "}
                    <a href="https://doi.org/10.1109/ICRA.2012.6225177" target="_blank"
                       rel="noopener noreferrer">
                        <em>LQR-RRT*: Optimal Sampling-Based Motion Planning with Automatically
                        Derived Extension Heuristics</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2012.
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

export default LqrRrtStar
