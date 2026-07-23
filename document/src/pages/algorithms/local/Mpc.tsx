import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import MpcSandbox from "../../../components/panels/local/mpc/MpcSandbox";
import MpcNonconvexDemo from "../../../components/panels/local/mpc/MpcNonconvexDemo";
import MpcHorizonFigure from "../../../components/panels/local/mpc/MpcHorizonFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import mpcPy from "../../../../../python/navigation/local_planning/predictive/mpc.py?raw";
import mpcHpp from "../../../../../cpp/include/navigation/local_planning/predictive/mpc.hpp?raw";
import mpcCpp from "../../../../../cpp/src/local_planning/predictive/mpc.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록. 다른 알고리즘 페이지와 같은 패턴(본문은 직관, 형식적 전개는 원할 때만 편다).
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Mpc = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Timed Elastic Band optimized a chain of poses and their timing, holding the
                    can't-slide-sideways constraint as a penalty. Model Predictive Control keeps the idea of
                    optimizing against a cost every tick, but moves the decision variable off the geometry
                    entirely: instead of poses, it optimizes the <strong>control sequence</strong> the robot
                    would execute over the next few steps. It rolls that sequence forward through a motion
                    model, scores the predicted trajectory, executes only the very first control, and then
                    throws the rest away and re-optimizes on the next tick. That last part is the{" "}
                    <em>receding horizon</em>, and it is what makes MPC a controller rather than a one-shot
                    planner.
                </p>}
                ko={<p>
                    Timed Elastic Band은 pose 사슬과 그 시간 배분을 최적화하며, 옆으로 미끄러질 수
                    없다는 제약을 페널티로 지켰다. Model Predictive Control은 매 tick 비용에 맞춰
                    최적화한다는 발상은 그대로 두되, 결정 변수를 기하에서 완전히 떼어낸다. pose가
                    아니라 로봇이 앞으로 몇 스텝 동안 실행할 <strong>제어열</strong>을 최적화한다.
                    그 제어열을 운동 모델로 굴려 예측 궤적을 만들고, 그 궤적을 채점하고, 맨 첫 제어
                    하나만 실행한 뒤 나머지는 버리고 다음 tick에 다시 최적화한다. 이 마지막 부분이{" "}
                    <em>receding horizon</em>이고, MPC를 한 번에 끝나는 planner가 아니라 컨트롤러로
                    만드는 지점이다.
                </p>}
            />

            <h2>{t("From Poses to Controls", "pose에서 control로")}</h2>
            <T
                en={<p>
                    Move the decision variable from poses to controls and the kinematics stop being something
                    to enforce. A pose chain can bend in ways a differential-drive robot cannot follow, which
                    is exactly why TEB needed a nonholonomic penalty. A control sequence rolled through the
                    unicycle model can only ever produce trajectories the robot can actually drive, so that
                    penalty disappears by construction. The horizon is built by iterating one step:
                </p>}
                ko={<p>
                    결정 변수를 pose에서 control로 옮기면 kinematics는 더 이상 강제할 대상이 아니게
                    된다. pose 사슬은 차동 구동 로봇이 따라갈 수 없는 방식으로 휠 수 있고, TEB가
                    비홀로노믹 페널티를 둔 이유가 바로 그것이었다. 제어열을 unicycle 모델로 굴리면
                    로봇이 실제로 달릴 수 있는 궤적만 나오므로 그 페널티는 구성상 사라진다. horizon은
                    한 스텝을 반복해 쌓는다:
                </p>}
            />
            <BlockMath math="x_{k+1} = f(x_k, u_k), \qquad x_k = (x, y, \theta),\quad u_k = (v_k, \omega_k),\quad k = 0, \ldots, H-1"/>
            <T
                en={<Terms items={[
                    ["x_k", "predicted state at step k: 2D position and heading. x_0 is the robot's current pose, held fixed"],
                    ["u_k", "control applied at step k: forward speed v_k and turn rate \\omega_k — the optimizer's decision variable"],
                    ["f", "one-step unicycle motion model — the exact constant-(v, \\omega) circular arc over one interval h"],
                    ["h", "prediction step, equal to the control period \\Delta t (predicting and executing on the same discretization)"],
                    ["H", "horizon length — the number of steps predicted ahead each tick"],
                    ["U", <>the full control sequence <InlineMath math="U = [u_0, \ldots, u_{H-1}]"/>, the decision variable MPC optimizes</>],
                ]}/>}
                ko={<Terms items={[
                    ["x_k", "스텝 k의 예측 상태. 2D 위치와 heading. x_0는 로봇의 현재 pose로 고정"],
                    ["u_k", "스텝 k에 가하는 제어. 전진 속도 v_k와 각속도 \\omega_k. 최적화의 결정 변수"],
                    ["f", "한 스텝 unicycle 운동 모델. 한 구간 h 동안의 정확한 정속 (v, \\omega) 원호"],
                    ["h", "예측 스텝. 제어 주기 \\Delta t와 같다(예측·실행을 같은 이산화로)"],
                    ["H", "horizon 길이. 매 tick 앞으로 예측하는 스텝 수"],
                    ["U", <>제어열 전체 <InlineMath math="U = [u_0, \ldots, u_{H-1}]"/>. MPC가 최적화하는 결정 변수</>],
                ]}/>}
            />

            <h2>{t("The Receding-Horizon Cost", "Horizon을 따라가는 비용")}</h2>
            <T
                en={<p>
                    One cost scores an entire control sequence by rolling it out and summing over the predicted
                    trajectory. It has three terms, and this is the same cost MPPI will minimize on the next
                    page — the whole point is that MPC and MPPI differ only in the optimizer, not the objective:
                </p>}
                ko={<p>
                    하나의 비용이 제어열 전체를 굴려 예측 궤적을 따라 합산해 채점한다. 세 항으로
                    이루어지고, 이것은 다음 장에서 MPPI가 최소화할 바로 그 비용이다. MPC와 MPPI는
                    목적함수가 아니라 옵티마이저만 다르다는 것이 핵심이다:
                </p>}
            />
            <BlockMath math="J(U) = \sum_{k=1}^{H} \Big[\, w_{\text{goal}}\,\lVert p_k - g \rVert^2 \;+\; w_{\text{obstacle}}\,\max(0,\ d_{\min} - c_k)^2 \;+\; w_{\text{control}}\,(v_{k-1}^2 + \omega_{k-1}^2) \,\Big]"/>
            <T
                en={<Terms items={[
                    ["J(U)", "total cost of the control sequence U — what the optimizer drives downhill"],
                    ["p_k", "predicted position (x, y) at step k, from rolling U out through f"],
                    ["g", "the goal position — MPC is goal-seeking, with no reference path"],
                    ["c_k", "clearance at p_k: continuous distance to the nearest occupied cell center minus the footprint radius"],
                    ["d_{\\min}", "obstacle activation distance — the penalty is zero once clearance reaches d_{\\min}"],
                    ["v_{k-1},\\ \\omega_{k-1}", "the control that produced step k, penalized to keep the effort small"],
                    ["w_{\\text{goal}},\\ w_{\\text{obstacle}},\\ w_{\\text{control}}", "the three term weights, identical in MPC and MPPI so the two compare on one objective"],
                ]}/>}
                ko={<Terms items={[
                    ["J(U)", "제어열 U의 총 비용. 옵티마이저가 내리막으로 미는 대상"],
                    ["p_k", "스텝 k의 예측 위치 (x, y). U를 f로 굴려 얻는다"],
                    ["g", "goal 위치. MPC는 참조 경로 없이 goal을 향하는 goal-seeking이다"],
                    ["c_k", "p_k의 clearance. 최근접 occupied cell 중심까지의 연속 거리에서 footprint 반경을 뺀 값"],
                    ["d_{\\min}", "장애물 활성화 거리. clearance가 d_{\\min}에 이르면 페널티는 0"],
                    ["v_{k-1},\\ \\omega_{k-1}", "스텝 k를 만든 제어. 제어 노력을 작게 유지하려는 페널티"],
                    ["w_{\\text{goal}},\\ w_{\\text{obstacle}},\\ w_{\\text{control}}", "세 항의 가중치. MPC와 MPPI가 같아 두 방식이 하나의 목적함수 위에서 비교된다"],
                ]}/>}
            />
            <T
                en={<p>
                    The goal term is summed over every step, not just the last, so it pulls the whole predicted
                    path toward the goal rather than only its endpoint. The obstacle term is a squared hinge,
                    active only inside <InlineMath math="d_{\min}"/>. It measures clearance as a{" "}
                    <em>continuous</em> distance to the nearest occupied cell center, not the grid-quantized
                    clearance a distance transform returns. That distinction matters more here than anywhere
                    else: MPC's gradient is a finite difference, and a clearance that is constant across a whole
                    grid cell would give a zero gradient inside it, so obstacle avoidance would silently stop
                    working. The continuous distance is exactly what keeps the finite-difference gradient alive.
                </p>}
                ko={<p>
                    goal 항은 마지막 스텝만이 아니라 매 스텝 합산되므로, 끝점만이 아니라 예측 경로
                    전체를 goal 쪽으로 당긴다. 장애물 항은 제곱 hinge로, <InlineMath math="d_{\min}"/>{" "}
                    안에서만 활성화된다. clearance를 거리 변환이 내주는 격자 양자화 값이 아니라 최근접
                    occupied cell 중심까지의 <em>연속</em> 거리로 잰다. 이 구분은 다른 어디보다 여기서
                    중요하다. MPC의 gradient는 유한차분이고, clearance가 한 셀 전체에서 상수라면 셀
                    안에서 gradient가 0이 되어 장애물 회피가 소리 없이 멈춘다. 연속 거리가 바로 그
                    유한차분 gradient를 살아 있게 한다.
                </p>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Cost: <InlineMath math="O(\text{iterations} \cdot H \cdot 2H)"/> per
                            tick.</strong> Each of the fixed <InlineMath math="\text{iterations}"/> gradient
                            steps builds a central finite-difference gradient over all{" "}
                            <InlineMath math="2H"/> control components, and every one of those perturbations
                            rolls out and scores an <InlineMath math="O(H)"/> trajectory. This is the honest
                            real-time bottleneck — the finite-difference gradient is simple and language-portable
                            but pays for it in rollouts per tick.</li>
                        <li><strong>Nonconvex, so only a local optimum.</strong> The obstacle term makes{" "}
                            <InlineMath math="J"/> nonconvex, and gradient descent from a warm-started sequence
                            finds a nearby local minimum, not the globally best control sequence. A concave or
                            symmetric obstacle can trap it, which is the weakness the page documents below.</li>
                        <li><strong>Only as good as the model.</strong> The horizon is rolled out through the
                            same unicycle model the simulator integrates, so here prediction and execution match
                            exactly and there is no model mismatch. On a real robot the dominant failure is that
                            the predicted arc and the driven arc disagree, which no amount of optimization fixes.</li>
                        <li><strong>The executed command is always feasible.</strong> Box projection keeps every
                            control inside <InlineMath math="v \in [0, v_{\max}]"/> and{" "}
                            <InlineMath math="|\omega| \le \omega_{\max}"/> during optimization, and the executed{" "}
                            <InlineMath math="u_0"/> is additionally acceleration-clamped against the robot's
                            current speed, so what the robot actually does is always within its physical limits.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>비용: tick당 <InlineMath math="O(\text{iterations} \cdot H \cdot 2H)"/>.</strong>{" "}
                            고정된 <InlineMath math="\text{iterations}"/>회 gradient step 각각이{" "}
                            <InlineMath math="2H"/>개 제어 성분 전체에 대해 central finite-difference
                            gradient를 만들고, 그 섭동 하나하나가 <InlineMath math="O(H)"/> 궤적을 굴려
                            채점한다. 이것이 정직한 실시간성 병목이다. 유한차분 gradient는 단순하고 언어
                            이식이 쉽지만 tick당 rollout 수로 그 대가를 치른다.</li>
                        <li><strong>비볼록이라 국소 최적만 보장한다.</strong> 장애물 항이{" "}
                            <InlineMath math="J"/>를 비볼록으로 만들고, warm-start된 제어열에서 시작한
                            경사하강은 전역 최선이 아니라 근처의 국소 최소를 찾는다. 오목하거나 대칭인
                            장애물이 이를 가둘 수 있고, 그것이 아래에서 다루는 약점이다.</li>
                        <li><strong>모델만큼만 좋다.</strong> horizon은 시뮬레이터가 적분하는 바로 그
                            unicycle 모델로 굴리므로, 여기서는 예측과 실행이 정확히 일치해 model
                            mismatch가 없다. 실차에서 지배적인 실패는 예측한 원호와 실제로 달린 원호가
                            어긋나는 것이고, 이는 최적화로 고쳐지지 않는다.</li>
                        <li><strong>실행 명령은 항상 실현 가능하다.</strong> box 투영이 최적화 내내 모든
                            제어를 <InlineMath math="v \in [0, v_{\max}]"/>, <InlineMath math="|\omega| \le
                            \omega_{\max}"/> 안에 두고, 실행되는 <InlineMath math="u_0"/>는 로봇의 현재
                            속도에 대해 가속까지 clamp된다. 그래서 로봇이 실제로 하는 동작은 항상 물리
                            한계 이내다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Every tick warm-starts the control sequence from last tick's, runs a fixed number of
                    projected gradient-descent steps, and extracts a command from only the first control.
                    Each gradient step perturbs every control component both ways, rolls each perturbation out,
                    and steps downhill with a per-component clamp and a box projection.
                </p>}
                ko={<p>
                    매 tick은 제어열을 지난 tick에서 warm-start하고, 고정 횟수의 투영 경사하강 step을
                    돌린 뒤, 첫 제어 하나에서만 명령을 뽑는다. 각 gradient step은 모든 제어 성분을 양쪽으로
                    섭동해 각 섭동을 굴리고, 성분별 clamp와 box 투영으로 내리막을 밟는다.
                </p>}
            />
            <Pseudocode code={`U ← [u_1, ..., u_{H-1}, u_{H-1}]     # 1  warm start: left-shift, duplicate last
for it in 1..iterations:             # 2
    for k in 0..H-1:                 # 3  central finite-difference gradient
        g_v[k]  ← (J(U; v_k+eps) - J(U; v_k-eps)) / (2 eps)
        g_w[k]  ← (J(U; w_k+eps) - J(U; w_k-eps)) / (2 eps)
    for k in 0..H-1:                 # 4  step + clamp + box projection
        v_k ← clamp01(v_k - clamp(alpha g_v[k], max_step_v),  0, v_max)
        w_k ← clampsym(w_k - clamp(alpha g_w[k], max_step_w), omega_max)
traj ← rollout(x_0, U)               # 5
emit(band_updated, traj, J(U))
v0 ← clamp(u_0.v, state.v ± a_max·h) # 6  accel-limit the executed command
v0 ← clamp(v0, 0, v_max);  w0 ← clampsym(u_0.omega, omega_max)
return (v0, w0)                       # 7  execute only u_0`}/>
            <T
                en={<ol>
                    <li>Warm-start from last tick's sequence by dropping the executed <InlineMath math="u_0"/>,
                        shifting everything left, and duplicating the last control to refill the horizon — a
                        cold start (first tick, or after reset) seeds <InlineMath math="H"/> zero controls
                        instead.</li>
                    <li>Run a fixed number of iterations with no convergence check and no early exit, for the
                        same cross-language determinism reason TEB fixes its iteration count.</li>
                    <li>Build the gradient by central finite differences: perturb each control component by{" "}
                        <InlineMath math="\pm\,\text{grad\_eps}"/>, roll out and score both, and divide by{" "}
                        <InlineMath math="2\,\text{grad\_eps}"/>. Components are always visited in the same fixed
                        order — <InlineMath math="k"/> ascending, <InlineMath math="v"/> before{" "}
                        <InlineMath math="\omega"/> — because floating-point summation is not associative and a
                        different order can flip a discrete clamp branch into a different trajectory. This is
                        also where the <em>continuous</em> clearance earns its keep: a step of{" "}
                        <InlineMath math="\text{grad\_eps} \approx 0.02"/> is finer than one grid cell, so a
                        cell-quantized obstacle cost would return an identical value at{" "}
                        <InlineMath math="U \pm \text{grad\_eps}"/> and the obstacle gradient would vanish.</li>
                    <li>Step downhill, clamping each component's step to{" "}
                        <InlineMath math="\text{max\_step\_v}"/> or{" "}
                        <InlineMath math="\text{max\_step\_omega}"/> so no single iteration lurches too far, then
                        project back into the box <InlineMath math="v \in [0, v_{\max}]"/>,{" "}
                        <InlineMath math="|\omega| \le \omega_{\max}"/>.</li>
                    <li>Roll the optimized sequence out once more to emit the predicted horizon as a band, with
                        its total cost, for the visualizer.</li>
                    <li>Acceleration-limit the executed speed against the velocity the simulator reports for
                        this tick, then box-clamp it — this is the one place the physical acceleration limit is
                        enforced, on the executed command only, not inside the prediction.</li>
                    <li><strong>The pitfall the whole method rests on:</strong> only <InlineMath math="u_0"/> is
                        ever executed. Everything past it exists solely to shape what that first control should
                        be, and is discarded the moment the next tick warm-starts. Treating the whole optimized
                        sequence as a plan to follow open-loop would throw away the one thing that makes MPC
                        robust — that it re-optimizes from the true measured state every single tick.</li>
                </ol>}
                ko={<ol>
                    <li>실행된 <InlineMath math="u_0"/>를 버리고 전체를 왼쪽으로 시프트한 뒤 마지막 제어를
                        복제해 horizon을 채워, 지난 tick의 제어열에서 warm-start한다. cold start(첫 tick
                        또는 reset 직후)는 대신 zero 제어 <InlineMath math="H"/>개를 시드한다.</li>
                    <li>수렴 검사도 조기 종료도 없이 고정 횟수만큼 반복한다. TEB가 반복 횟수를 고정하는
                        것과 같은, 언어 간 결정론을 위한 이유다.</li>
                    <li>central finite difference로 gradient를 만든다. 각 제어 성분을{" "}
                        <InlineMath math="\pm\,\text{grad\_eps}"/>만큼 섭동해 양쪽을 굴려 채점하고{" "}
                        <InlineMath math="2\,\text{grad\_eps}"/>로 나눈다. 성분은 항상 같은 고정 순서로
                        본다. <InlineMath math="k"/> 오름차순, <InlineMath math="v"/> 먼저{" "}
                        <InlineMath math="\omega"/> 나중이다. 부동소수 합이 결합법칙을 따르지 않아 다른
                        순서는 이산 clamp 분기를 다른 궤적으로 뒤집을 수 있기 때문이다. 여기서{" "}
                        <em>연속</em> clearance가 값을 한다. <InlineMath math="\text{grad\_eps} \approx
                        0.02"/> 크기의 섭동은 한 격자 셀보다 작아, 셀 양자화 장애물 비용은{" "}
                        <InlineMath math="U \pm \text{grad\_eps}"/>에서 같은 값을 내주고 장애물 gradient가
                        사라진다.</li>
                    <li>내리막을 밟되 각 성분의 step을 <InlineMath math="\text{max\_step\_v}"/> 또는{" "}
                        <InlineMath math="\text{max\_step\_omega}"/>로 clamp해 한 반복이 지나치게 튀지
                        않게 하고, box <InlineMath math="v \in [0, v_{\max}]"/>,{" "}
                        <InlineMath math="|\omega| \le \omega_{\max}"/> 안으로 다시 투영한다.</li>
                    <li>최적화된 제어열을 한 번 더 굴려 예측 horizon을 총 비용과 함께 band로 방출한다.
                        시각화용이다.</li>
                    <li>실행 속도를 시뮬레이터가 이 tick에 넘긴 속도에 대해 가속 clamp한 뒤 box-clamp한다.
                        물리 가속 한계가 강제되는 유일한 지점으로, 예측 내부가 아니라 실행 명령에만
                        적용된다.</li>
                    <li><strong>방법 전체가 걸려 있는 함정.</strong> 실행되는 것은 오직{" "}
                        <InlineMath math="u_0"/>뿐이다. 그 뒤 전부는 그 첫 제어가 무엇이어야 하는지를 빚는
                        데만 있고, 다음 tick이 warm-start하는 순간 버려진다. 최적화된 제어열 전체를 개루프로
                        따라갈 계획으로 여기면, MPC를 견고하게 만드는 단 하나, 즉 매 tick 실제 측정 상태에서
                        다시 최적화한다는 점을 버리게 된다.</li>
                </ol>}
            />

            <Proof title={t(
                "Derivation (why fixed-step descent lowers the cost)",
                "유도 (고정 스텝 하강이 비용을 낮추는 이유)",
            )}>
                <T
                    en={<>
                        <p><strong>Assumptions.</strong></p>
                        <ul>
                            <li><InlineMath math="J"/> has an <InlineMath math="L"/>-Lipschitz gradient:{" "}
                                <InlineMath math="\lVert \nabla J(U') - \nabla J(U) \rVert \le L \lVert U' - U \rVert"/>.</li>
                            <li>The step is a plain gradient step <InlineMath math="U' = U - \alpha\,\nabla J(U)"/>{" "}
                                with <InlineMath math="0 < \alpha \le 1/L"/> (no clamp or box active).</li>
                        </ul>
                        <p>The descent lemma for an <InlineMath math="L"/>-smooth function gives:</p>
                        <BlockMath math="J(U') \le J(U) + \nabla J(U)^\top (U' - U) + \tfrac{L}{2}\lVert U' - U \rVert^2"/>
                        <Terms items={[
                            ["U'", "the sequence after one gradient step"],
                            ["\\nabla J(U)", "the gradient of the cost at U"],
                            ["L", "the Lipschitz constant of the gradient"],
                        ]}/>
                        <p>Substitute <InlineMath math="U' - U = -\alpha\,\nabla J(U)"/>:</p>
                        <BlockMath math="J(U') \le J(U) - \alpha \lVert \nabla J(U) \rVert^2 + \tfrac{L\alpha^2}{2}\lVert \nabla J(U) \rVert^2 = J(U) - \alpha\Big(1 - \tfrac{L\alpha}{2}\Big)\lVert \nabla J(U) \rVert^2"/>
                        <p>With <InlineMath math="\alpha \le 1/L"/> we have{" "}
                            <InlineMath math="1 - \tfrac{L\alpha}{2} \ge \tfrac{1}{2}"/>, hence:</p>
                        <BlockMath math="J(U') \le J(U) - \tfrac{\alpha}{2}\lVert \nabla J(U) \rVert^2"/>
                        <p>The right side is <InlineMath math="\le J(U)"/>, with equality only when{" "}
                            <InlineMath math="\nabla J(U) = 0"/>. Each unconstrained step strictly lowers{" "}
                            <InlineMath math="J"/> until a stationary point, which is the local optimum the
                            nonconvex <InlineMath math="J"/> settles into — never a guarantee of the global one.</p>
                    </>}
                    ko={<>
                        <p><strong>가정.</strong></p>
                        <ul>
                            <li><InlineMath math="J"/>의 gradient가 <InlineMath math="L"/>-Lipschitz다.{" "}
                                <InlineMath math="\lVert \nabla J(U') - \nabla J(U) \rVert \le L \lVert U' - U \rVert"/>.</li>
                            <li>step은 순수 gradient step <InlineMath math="U' = U - \alpha\,\nabla J(U)"/>이고{" "}
                                <InlineMath math="0 < \alpha \le 1/L"/> (clamp·box 미활성).</li>
                        </ul>
                        <p><InlineMath math="L"/>-smooth 함수의 descent lemma는 다음을 준다:</p>
                        <BlockMath math="J(U') \le J(U) + \nabla J(U)^\top (U' - U) + \tfrac{L}{2}\lVert U' - U \rVert^2"/>
                        <Terms items={[
                            ["U'", "gradient step 한 번 뒤의 제어열"],
                            ["\\nabla J(U)", "U에서의 비용 gradient"],
                            ["L", "gradient의 Lipschitz 상수"],
                        ]}/>
                        <p><InlineMath math="U' - U = -\alpha\,\nabla J(U)"/>를 대입하면:</p>
                        <BlockMath math="J(U') \le J(U) - \alpha \lVert \nabla J(U) \rVert^2 + \tfrac{L\alpha^2}{2}\lVert \nabla J(U) \rVert^2 = J(U) - \alpha\Big(1 - \tfrac{L\alpha}{2}\Big)\lVert \nabla J(U) \rVert^2"/>
                        <p><InlineMath math="\alpha \le 1/L"/>이면 <InlineMath math="1 - \tfrac{L\alpha}{2}
                            \ge \tfrac{1}{2}"/>이므로:</p>
                        <BlockMath math="J(U') \le J(U) - \tfrac{\alpha}{2}\lVert \nabla J(U) \rVert^2"/>
                        <p>우변은 <InlineMath math="\le J(U)"/>이고, 등호는 <InlineMath math="\nabla J(U) =
                            0"/>일 때만 성립한다. 제약이 걸리지 않은 각 step은 정류점에 이를 때까지{" "}
                            <InlineMath math="J"/>를 엄격히 낮춘다. 그 정류점이 비볼록{" "}
                            <InlineMath math="J"/>가 안착하는 국소 최적이고, 결코 전역 최적의 보장은 아니다.</p>
                    </>}
                />
            </Proof>

            <h2>{t("The Nonconvex Trap", "비볼록 함정")}</h2>
            <T
                en={<p>
                    The obstacle term makes the cost nonconvex, and a finite-difference gradient can only ever
                    walk downhill from where it stands — it cannot see a better route that lies on the far side
                    of a hill in the cost surface. The demo below plants a goal in the top-right corner, tucked
                    closer to the boundary walls than <InlineMath math="d_{\min}"/>. The goal's pull and the
                    obstacle penalty's push cancel right where the goal sits, so the gradient settles into a
                    local optimum and the robot stalls a step short of a goal it physically cannot touch. This
                    is the sibling of the potential-field local minimum, one page removed: there a force
                    balance froze the robot, here a gradient balance does. MPPI, on the next page, escapes
                    exactly this kind of trap by shaking the sequence with random samples instead of following
                    a single gradient.
                </p>}
                ko={<p>
                    장애물 항이 비용을 비볼록으로 만들고, 유한차분 gradient는 선 자리에서 내리막으로만
                    걸을 수 있다. 비용 지형의 언덕 반대편에 놓인 더 나은 길은 보지 못한다. 아래 데모는
                    goal을 우상단 코너, 경계 벽까지의 거리가 <InlineMath math="d_{\min}"/>보다 가까운
                    자리에 심는다. goal의 인력과 장애물 페널티의 반발이 바로 goal 자리에서 상쇄되어
                    gradient가 국소 최적에 안착하고, 로봇은 물리적으로 밟을 수 없는 goal 한 발짝 앞에서
                    정체한다. 이것은 한 장 앞 potential field 국소 최소의 형제다. 거기서는 힘의 균형이
                    로봇을 얼렸고, 여기서는 gradient의 균형이 그렇게 한다. 다음 장의 MPPI는 하나의
                    gradient를 따르는 대신 무작위 표본으로 제어열을 흔들어 바로 이런 함정을 벗어난다.
                </p>}
            />
            <MpcNonconvexDemo/>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs MPC live in your browser. The blue chain is the predicted horizon,
                    re-optimized every tick and running ahead of the robot, while only its first control is
                    executed. Raise <InlineMath math="w_{\text{obstacle}}"/> and watch the chain bow further
                    from the blocks, or shorten the horizon and watch it grow shortsighted.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 MPC를 라이브로 실행한다. 파란 사슬은 예측 horizon으로,
                    매 tick 다시 최적화되어 로봇보다 앞서 달리고, 첫 제어만 실행된다.{" "}
                    <InlineMath math="w_{\text{obstacle}}"/>을 올리면 사슬이 블록에서 더 크게 휘는
                    모습을, horizon을 줄이면 근시안이 되는 모습을 볼 수 있다.
                </p>}
            />
            <MpcSandbox/>
            <MpcHorizonFigure/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below follows the algorithm above line for line: the warm-start shift,
                    the fixed-iteration central finite-difference gradient, the clamped-and-projected step, and
                    the acceleration-limited first control. The shared rollout and cost live in the family's{" "}
                    <code>_rollout</code> module. The code is the actual repository source, not an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 그대로 옮긴 것이다. warm-start 시프트, 고정 반복 central
                    finite-difference gradient, clamp 후 투영하는 step, 가속 제한된 첫 제어까지 그대로다.
                    공유 rollout과 비용은 패밀리의 <code>_rollout</code> 모듈에 있다. 아래 코드는 발췌가
                    아니라 저장소의 실제 소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/predictive/mpc.py",
                                code: mpcPy,
                                href: `${REPO}/python/navigation/local_planning/predictive/mpc.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/predictive/mpc.hpp",
                                code: mpcHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/predictive/mpc.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/predictive/mpc.cpp",
                                code: mpcCpp,
                                href: `${REPO}/cpp/src/local_planning/predictive/mpc.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "Warm-start shift, fixed-iteration finite-difference gradient descent with box projection, and the acceleration-limited first control, embedded from the repository sources",
                    "warm-start 시프트, box 투영을 곁들인 고정 반복 유한차분 경사하강, 가속 제한된 첫 제어. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert,{" "}
                    <a href="https://doi.org/10.1016/S0005-1098(99)00214-9" target="_blank" rel="noopener noreferrer">
                        <em>Constrained model predictive control: Stability and optimality</em>
                    </a>, Automatica, vol. 36, no. 6, pp. 789–814, 2000.
                </li>
                <li>
                    G. Klančar, I. Škrjanc,{" "}
                    <a href="https://doi.org/10.1016/j.robot.2007.01.002" target="_blank" rel="noopener noreferrer">
                        <em>Tracking-error model-based predictive control for mobile robots in real time</em>
                    </a>, Robotics and Autonomous Systems, vol. 55, no. 6, pp. 460–469, 2007.
                </li>
            </ol>
        </>
    )
}

export default Mpc
