import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import TebSandbox from "../../../components/panels/local/teb/TebSandbox";
import TebHomotopyDemo from "../../../components/panels/local/teb/TebHomotopyDemo";
import TebOptimizationFigure from "../../../components/panels/local/teb/TebOptimizationFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import tebPy from "../../../../../python/navigation/local_planning/band/teb.py?raw";
import tebHpp from "../../../../../cpp/include/navigation/local_planning/band/teb.hpp?raw";
import tebCpp from "../../../../../cpp/src/local_planning/band/teb.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록. 다른 알고리즘 페이지와 같은 패턴(본문은 직관, 형식적 전개는 원할 때만 편다).
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Teb = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    An Elastic Band is a shape — a chain of bubbles reshaped by force balance, with no notion
                    of when the robot should be where along it. The Timed Elastic Band, introduced by
                    Christoph Rösmann and collaborators in 2012 and extended through 2017, keeps the idea of a
                    deformable pose chain but attaches a clock to it: every consecutive pair of poses gets its
                    own time interval, and a single optimization jointly reshapes the geometry and stretches
                    or compresses those intervals against a cost function that cares about tracking, clearance,
                    speed, acceleration, and how fast the robot actually gets there. It is the local controller
                    behind the widely used <code>teb_local_planner</code> ROS package.
                </p>}
                ko={<p>
                    Elastic Band은 모양이다. 힘의 균형으로 다시 빚어지는 bubble 사슬일 뿐, 그 위 어디에
                    로봇이 언제 있어야 하는지에 대한 개념이 없다. Christoph Rösmann과 동료들이 2012년
                    내놓고 2017년까지 확장한 Timed Elastic Band(TEB)는 변형 가능한 pose 사슬이라는 발상은
                    그대로 두되, 거기에 시계를 하나 달아 둔다. 연속한 pose 쌍마다 자기만의 시간 구간을
                    갖고, 하나의 최적화가 기하와 그 시간 구간들을 동시에 늘리거나 줄이며 추종·clearance·
                    속도·가속도·실제로 얼마나 빨리 도착하는지를 함께 고려하는 비용함수에 맞춰 다시
                    빚는다. 널리 쓰이는 ROS 패키지 <code>teb_local_planner</code>의 local controller가
                    바로 이것이다.
                </p>}
            />

            <h2>{t("Adding Time to the Band", "밴드에 시간을 더하다")}</h2>
            <T
                en={<p>
                    The state is a pose chain paired with per-segment durations:
                </p>}
                ko={<p>
                    상태는 pose 사슬과 구간별 소요 시간을 짝지은 것이다:
                </p>}
            />
            <BlockMath math="\text{poses} = [s_0, \ldots, s_{n-1}],\quad s_i = (x_i, y_i, \theta_i); \qquad \text{dts} = [\Delta T_0, \ldots, \Delta T_{n-2}]"/>
            <T
                en={<Terms items={[
                    ["n", "number of poses currently in the chain"],
                    ["s_i", "pose i in the chain: 2D position plus heading"],
                    ["\\Delta T_i", "time budgeted for the robot to travel from s_i to s_{i+1}, always \\ge \\Delta T_{\\min}"],
                    ["s_0,\\ s_{n-1}", "the two fixed endpoints — s_0 is the robot's executed pose, s_{n-1} is the local goal"],
                ]}/>}
                ko={<Terms items={[
                    ["n", "사슬 안 pose의 현재 개수"],
                    ["s_i", "사슬 안의 pose i. 2D 위치와 heading의 쌍"],
                    ["\\Delta T_i", "s_i에서 s_{i+1}로 이동하는 데 배정된 시간. 항상 \\Delta T_{\\min} 이상"],
                    ["s_0,\\ s_{n-1}", "고정된 양 끝점. s_0는 로봇이 실제로 도달한 pose, s_{n-1}은 local goal"],
                ]}/>}
            />
            <T
                en={<p>
                    Every internal pose and every <InlineMath math="\Delta T_i"/> is a free variable the
                    optimizer touches; the endpoints are anchors, exactly as in Elastic Bands. The one new
                    wrinkle is what "goal" means here: instead of the true final goal, TEB optimizes toward a{" "}
                    <strong>local goal</strong> — the point where the reference path, clipped to a fixed
                    horizon ahead of the robot's own progress along it, ends. Once the remaining path is
                    shorter than the horizon, the local goal collapses to the real one.
                </p>}
                ko={<p>
                    모든 내부 pose와 모든 <InlineMath math="\Delta T_i"/>는 최적화가 손대는 자유
                    변수다. 양 끝점은 Elastic Bands와 똑같이 닻이다. 여기서 새로 생기는 주름 하나는
                    "goal"이 뜻하는 바다. TEB는 진짜 최종 목표가 아니라 <strong>local goal</strong>을
                    향해 최적화한다. 로봇이 참조 경로 위에서 이미 나아간 지점부터 고정된 horizon만큼
                    잘라낸 조각의 끝점이다. 남은 경로가 horizon보다 짧아지면 local goal은 곧 실제 목표로
                    수렴한다.
                </p>}
            />

            <h2>{t("Cost Terms", "비용 항")}</h2>
            <T
                en={<p>
                    Six soft-constraint terms sum to one cost that gradient descent minimizes every tick, each
                    weighted independently. The first pulls the chain toward the reference path, anchored to
                    where each pose started this tick (not a moving target — that would never let the term
                    settle):
                </p>}
                ko={<p>
                    여섯 개의 soft-constraint 항이 더해져 매 tick gradient descent가 최소화하는 하나의
                    비용이 되고, 각 항은 독립된 가중치를 갖는다. 첫 항은 사슬을 참조 경로 쪽으로
                    당긴다. 각 pose가 이번 tick 시작한 위치에 고정된 anchor를 기준으로 한다(움직이는
                    목표라면 이 항은 결코 안정되지 못한다):
                </p>}
            />
            <T
                en={<p>
                    <strong>On the solver:</strong> the original TEB formulates this same objective as a sparse
                    factor graph — a hyper-graph in g2o's terms, since terms like the kinematics residual below
                    couple more than two poses — and solves it with the Levenberg-Marquardt algorithm. This
                    page's engine is a from-scratch gradient-descent reimplementation of the same six terms,
                    chosen for cross-language determinism and easy step-by-step visualization, not for solver
                    fidelity to the paper.
                </p>}
                ko={<p>
                    <strong>솔버에 대해:</strong> 원래 TEB는 이와 같은 목적함수를 sparse factor graph로
                    구성한다. g2o 식으로는 hyper-graph인데, 아래 kinematics 잔차처럼 pose 두 개보다 많이
                    엮이는 항이 있기 때문이다. 그리고 이를 Levenberg-Marquardt 알고리즘으로 푼다. 이
                    페이지의 엔진은 같은 여섯 항을 대상으로 처음부터 새로 짠 gradient descent다. 논문
                    솔버에 대한 충실도가 아니라 언어 간 결정론과 단계별 시각화 편의성을 위해 고른
                    선택이다.
                </p>}
            />
            <BlockMath math="f_{\text{path}} = w_{\text{path}} \sum_{i=1}^{n-2} \lVert p_i - v_i \rVert^2"/>
            <T
                en={<Terms items={[
                    ["f_{\\text{path}}", "reference-tracking cost, summed over internal poses only"],
                    ["p_i", "the (x, y) position of internal pose i — the optimizer's variable"],
                    ["v_i", "the anchor for pose i: nearest point on the clipped reference path to p_i at the start of this tick's optimization, held fixed for the whole tick"],
                    ["w_{\\text{path}}", "reference-tracking weight"],
                ]}/>}
                ko={<Terms items={[
                    ["f_{\\text{path}}", "참조 경로 추종 비용. 내부 pose에 대해서만 합산"],
                    ["p_i", "내부 pose i의 (x, y) 위치. 최적화가 움직이는 변수"],
                    ["v_i", "pose i의 anchor. 이번 tick 최적화 시작 시점의 p_i에서 clip된 참조 경로 위 최근접점, tick 내내 고정"],
                    ["w_{\\text{path}}", "참조 경로 추종 가중치"],
                ]}/>}
            />
            <T
                en={<p>
                    The second keeps distance from obstacles — but unlike Elastic Bands' bubble radius, this
                    term needs a distance that is continuous in <InlineMath math="p_i"/> for a gradient to
                    exist at all, so it uses the distance to the nearest occupied cell's <em>center</em>
                    directly, not the grid-quantized clearance:
                </p>}
                ko={<p>
                    두 번째 항은 장애물과 거리를 둔다. 다만 Elastic Bands의 bubble 반경과 달리, 이
                    항에는 <InlineMath math="p_i"/>에 대해 연속인 거리가 필요하다(그래야 gradient가
                    존재한다). 그래서 격자로 양자화된 clearance 대신 최근접 점유 셀의{" "}
                    <em>중심</em>까지의 거리를 직접 쓴다:
                </p>}
            />
            <BlockMath math="g_i = \max\big(0,\ d_{\min} - \tilde d_i\big), \qquad f_{\text{obs}} = w_{\text{obstacle}} \sum_{i=1}^{n-2} g_i^2"/>
            <T
                en={<Terms items={[
                    ["\\tilde d_i", "continuous distance from p_i to the nearest occupied cell center (0 if none within d_{\\min})"],
                    ["d_{\\min}", "activation distance — clearance at or beyond this contributes nothing"],
                    ["g_i", "clearance violation at pose i — positive only when p_i is closer than d_{\\min}"],
                    ["f_{\\text{obs}}", "obstacle-clearance cost"],
                    ["w_{\\text{obstacle}}", "obstacle-clearance weight"],
                ]}/>}
                ko={<Terms items={[
                    ["\\tilde d_i", "p_i에서 최근접 점유 셀 중심까지의 연속 거리 (d_{\\min} 안에 없으면 0)"],
                    ["d_{\\min}", "활성화 거리. 이 값 이상 벌어지면 아무 기여도 없다"],
                    ["g_i", "pose i의 clearance 위반량. p_i가 d_{\\min}보다 가까울 때만 양수"],
                    ["f_{\\text{obs}}", "장애물 clearance 비용"],
                    ["w_{\\text{obstacle}}", "장애물 clearance 가중치"],
                ]}/>}
            />
            <T
                en={<p>
                    The third and fourth are soft physical limits on every segment's realized velocity and
                    acceleration — realized, because nothing in the state directly stores speed; it falls out
                    of geometry divided by time. The paper's penalty functions activate gradually, inside the
                    bound by a margin <InlineMath math="\epsilon"/>; this implementation activates exactly at
                    the bound with a plain <InlineMath math="\max(0, \cdot)^2"/>:
                </p>}
                ko={<p>
                    세 번째와 네 번째는 매 구간에서 실현되는 속도와 가속도에 대한 soft 물리 한계다.
                    "실현되는"이라 부르는 이유는, 상태 어디에도 속도가 직접 저장돼 있지 않고 기하를
                    시간으로 나눈 값으로 매번 계산되기 때문이다. 논문의 페널티 함수는 한계보다 여유{" "}
                    <InlineMath math="\epsilon"/>만큼 안쪽에서부터 서서히 활성화되지만, 이 구현은 한계
                    지점에서 곧바로 <InlineMath math="\max(0, \cdot)^2"/>로 활성화된다:
                </p>}
            />
            <BlockMath math="v_i = \frac{\ell_i}{\Delta T_i},\quad \omega_i = \frac{\operatorname{wrap}(\theta_{i+1} - \theta_i)}{\Delta T_i}, \qquad f_{\text{vel}} = w_{\text{velocity}} \sum_{i=0}^{n-2} \Big[ \max(0, v_i - v_{\max})^2 + \max(0, |\omega_i| - \omega_{\max})^2 \Big]"/>
            <T
                en={<Terms items={[
                    ["\\ell_i", "chord length \\lVert p_{i+1} - p_i \\rVert of segment i"],
                    ["v_i,\\ \\omega_i", "linear and angular speed realized on segment i, from geometry and \\Delta T_i alone"],
                    ["v_{\\max},\\ \\omega_{\\max}", "the robot's physical speed and turn-rate limits"],
                    ["f_{\\text{vel}}", "velocity-limit cost — zero whenever both v_i and \\omega_i are within bounds"],
                    ["w_{\\text{velocity}}", "velocity-limit weight"],
                ]}/>}
                ko={<Terms items={[
                    ["\\ell_i", "구간 i의 chord 길이 \\lVert p_{i+1} - p_i \\rVert"],
                    ["v_i,\\ \\omega_i", "구간 i에서 실현되는 선속·각속. 기하와 \\Delta T_i만으로 계산"],
                    ["v_{\\max},\\ \\omega_{\\max}", "로봇의 물리적 속도·회전율 한계"],
                    ["f_{\\text{vel}}", "속도 한계 비용. v_i, \\omega_i 모두 한계 안이면 0"],
                    ["w_{\\text{velocity}}", "속도 한계 가중치"],
                ]}/>}
            />
            <BlockMath math="a_i = \frac{v_{i+1} - v_i}{0.5(\Delta T_i + \Delta T_{i+1})}, \qquad f_{\text{acc}} = w_{\text{acceleration}} \sum_{i=0}^{n-3} \max(0, |a_i| - a_{\max})^2"/>
            <T
                en={<Terms items={[
                    ["a_i", "translational acceleration realized between segments i and i+1"],
                    ["a_{\\max}", "the robot's physical acceleration limit"],
                    ["f_{\\text{acc}}", "acceleration-limit cost (translational only — rotational acceleration is left out, since it adds a term with no visible effect on this unicycle demo)"],
                    ["w_{\\text{acceleration}}", "acceleration-limit weight"],
                ]}/>}
                ko={<Terms items={[
                    ["a_i", "구간 i와 i+1 사이에서 실현되는 병진 가속도"],
                    ["a_{\\max}", "로봇의 물리적 가속 한계"],
                    ["f_{\\text{acc}}", "가속 한계 비용 (병진만이며, 회전 가속 항은 이 unicycle 데모에서 눈에 띄는 효과 없이 항 수만 늘려 제외했다)"],
                    ["w_{\\text{acceleration}}", "가속 한계 가중치"],
                ]}/>}
            />
            <T
                en={<p>
                    The fifth is what makes this a <em>timed</em> band rather than just a smoother one: it
                    directly rewards small time intervals, so the optimizer has an incentive to move fast
                    wherever nothing else is holding it back:
                </p>}
                ko={<p>
                    다섯 번째 항이 이것을 그저 매끄러운 밴드가 아니라 <em>시간이 있는</em> 밴드로
                    만드는 핵심이다. 작은 시간 구간에 직접 보상을 줘서, 다른 무엇도 붙잡지 않는 한 빨리
                    움직일 유인을 최적화에 심어 둔다:
                </p>}
            />
            <BlockMath math="f_{\text{time}} = w_{\text{time}} \sum_{i=0}^{n-2} \Delta T_i"/>
            <T
                en={<Terms items={[
                    ["f_{\\text{time}}", "time-optimality cost — total horizon time, penalized directly"],
                    ["w_{\\text{time}}", "time-optimality weight — raising it shrinks \\Delta T wherever the other five terms allow"],
                ]}/>}
                ko={<Terms items={[
                    ["f_{\\text{time}}", "시간 최적성 비용. 전체 horizon 시간을 그대로 페널티로 준다"],
                    ["w_{\\text{time}}", "시간 최적성 가중치. 올릴수록 나머지 다섯 항이 허용하는 한 \\Delta T가 줄어든다"],
                ]}/>}
            />
            <T
                en={<p>
                    The paper's g2o edge instead minimizes the sum of <em>squared</em> interval errors{" "}
                    <InlineMath math="\sum_k \Delta T_k^2"/>; the linear sum used here gives every segment the
                    same constant gradient regardless of how large <InlineMath math="\Delta T_i"/> already is.
                </p>}
                ko={<p>
                    논문의 g2o edge는 대신 구간 시간의 <em>제곱</em> 오차 합{" "}
                    <InlineMath math="\sum_k \Delta T_k^2"/>을 최소화한다. 여기서 쓰는 선형 합은{" "}
                    <InlineMath math="\Delta T_i"/>가 이미 얼마나 크든 모든 구간에 같은 상수
                    gradient를 준다.
                </p>}
            />
            <T
                en={<p>
                    The sixth term is new relative to the plain Elastic Band page's terms, and is the piece
                    that keeps the geometry honest for a robot that cannot slide sideways. Nothing so far stops
                    the optimizer from moving <InlineMath math="p_i"/> in a direction its own heading{" "}
                    <InlineMath math="\theta_i"/> disagrees with — the <strong>nonholonomic kinematics</strong>{" "}
                    term, the defining constraint introduced by Rösmann et al. in the original 2012 TEB paper,
                    penalizes exactly that. It is derived below from a two-pose circular-arc model:
                </p>}
                ko={<p>
                    여섯 번째 항은 밋밋한 Elastic Band 페이지의 항들에 견주면 새로 등장하며, 옆으로
                    미끄러질 수 없는 로봇에 대해 기하를 정직하게 지켜 주는 조각이다. 지금까지는
                    최적화가 <InlineMath math="p_i"/>를 그 pose 자신의 heading{" "}
                    <InlineMath math="\theta_i"/>와 어긋나는 방향으로 옮기는 것을 막을 것이 없다.{" "}
                    <strong>비홀로노믹 kinematics</strong> 항, 즉 Rösmann과 동료들이 2012년 원 논문에서
                    도입한 TEB의 정의적 제약이 정확히 그것을 페널티로 준다. 아래에서 two-pose 원호
                    모델로 유도한다:
                </p>}
            />
            <BlockMath math="h_i = (\cos\theta_i + \cos\theta_{i+1})\, \Delta y_i - (\sin\theta_i + \sin\theta_{i+1})\, \Delta x_i, \qquad f_{\text{kin}} = w_{\text{kinematics}} \sum_{i=0}^{n-2} h_i^2"/>
            <T
                en={<Terms items={[
                    ["\\Delta x_i,\\ \\Delta y_i", "components of the chord d_i = p_{i+1} - p_i"],
                    ["h_i", "two-pose arc residual — zero exactly when s_i and s_{i+1} lie on a common constant-curvature arc (Rösmann et al. 2012)"],
                    ["f_{\\text{kin}}", "nonholonomic kinematics cost (Rösmann et al. 2012)"],
                    ["w_{\\text{kinematics}}", "kinematics weight — typically the largest of the six, since a geometrically inconsistent pose chain undermines every other term's meaning"],
                ]}/>}
                ko={<Terms items={[
                    ["\\Delta x_i,\\ \\Delta y_i", "chord d_i = p_{i+1} - p_i의 성분"],
                    ["h_i", "two-pose 원호 잔차. s_i와 s_{i+1}이 공통의 등곡률 원호 위에 있을 때 정확히 0이 된다 (Rösmann et al. 2012)"],
                    ["f_{\\text{kin}}", "비홀로노믹 kinematics 비용 (Rösmann et al. 2012)"],
                    ["w_{\\text{kinematics}}", "kinematics 가중치. 보통 여섯 항 중 가장 크다. 기하적으로 앞뒤가 안 맞는 pose 사슬은 나머지 다섯 항의 의미 자체를 흔들기 때문이다"],
                ]}/>}
            />
            <TebOptimizationFigure/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Cost: <InlineMath math="O(n)"/> per gradient iteration, fixed number of
                            iterations per tick.</strong> Every term's gradient touches only a constant number
                            of neighboring poses, so one full accumulation pass is linear in the pose count —
                            and it runs a fixed <InlineMath math="\text{iterations}"/> times, no early exit,
                            for the same cross-language determinism reason Elastic Bands fixes its deformation
                            count.</li>
                        <li><strong>Purely a soft-constraint balance, not a constrained optimizer.</strong> Every
                            term above is a squared penalty, not a hard bound — a candidate can end one tick
                            still slightly over <InlineMath math="v_{\max}"/> if the weights pull elsewhere
                            harder. Command extraction still clamps the final <InlineMath math="v"/> and{" "}
                            <InlineMath math="\omega"/> to the true limits, so the executed command is always
                            safe even when the optimized pose chain is not perfectly so.</li>
                        <li><strong>No global optimality, and no completeness.</strong> Gradient descent from
                            a warm-started chain finds a nearby local optimum, not the best chain reachable —
                            exactly the same caveat as Elastic Bands, now over a larger joint space of position,
                            heading, and time.</li>
                        <li><strong>Time-optimal only relative to the other five terms.</strong> Raising{" "}
                            <InlineMath math="w_{\text{time}}"/> shrinks <InlineMath math="\Delta T"/> wherever
                            clearance, velocity, acceleration, and kinematics allow — it is not solving a
                            time-optimal control problem in isolation.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>비용: gradient 반복 1회당 <InlineMath math="O(n)"/>, tick당 반복
                            횟수는 고정.</strong> 모든 항의 gradient는 이웃 pose 몇 개만 건드리므로 한 번의
                            전체 누적은 pose 수에 선형이다. 그리고 고정된 <InlineMath
                            math="\text{iterations}"/>회만 돌고 조기 종료하지 않는다. Elastic Bands가 변형
                            횟수를 고정하는 것과 같은, 언어 간 결정론을 위한 이유다.</li>
                        <li><strong>순전히 soft-constraint 균형이지 제약 최적화가 아니다.</strong> 위의
                            모든 항은 하드 한계가 아니라 제곱 페널티다. 다른 가중치가 더 세게 당기면
                            한 tick의 결과가 <InlineMath math="v_{\max}"/>를 살짝 넘긴 채 끝날 수도
                            있다. 명령 추출 단계는 그래도 최종 <InlineMath math="v"/>와 <InlineMath
                            math="\omega"/>를 진짜 한계로 클램프하므로, 최적화된 pose 사슬이 완벽하지
                            않아도 실행되는 명령은 항상 안전하다.</li>
                        <li><strong>전역 최적성도, 완전성도 없다.</strong> warm-start된 사슬에서 시작한
                            gradient descent는 도달 가능한 최선이 아니라 근처의 국소 최적점을 찾는다.
                            Elastic Bands와 똑같은 한계이되, 이번엔 위치·heading·시간을 아우르는 더 큰
                            결합 공간 위에서다.</li>
                        <li><strong>시간 최적성은 나머지 다섯 항에 상대적일 뿐이다.</strong> <InlineMath
                            math="w_{\text{time}}"/>을 올리면 clearance·속도·가속·kinematics가 허용하는
                            한 <InlineMath math="\Delta T"/>가 줄어들 뿐, 이것만 따로 떼어 푸는
                            시간-최적 제어 문제가 아니다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Every tick reprojects progress along the reference path, decides whether to rebuild the
                    band from scratch or warm-start from last tick's, resizes it to a healthy segment count,
                    then runs a fixed number of gradient steps before extracting a command from the very first
                    segment.
                </p>}
                ko={<p>
                    매 tick은 참조 경로 위 진행을 다시 사영하고, 밴드를 처음부터 다시 지을지 지난
                    tick에서 이어갈지 정한 뒤, 건강한 구간 수로 resize하고, 고정 횟수의 gradient step을
                    돌린 다음, 맨 첫 구간에서 명령을 뽑는다.
                </p>}
            />
            <Pseudocode code={`idx ← advance_progress_index(path, robot_xy, idx)                    # 1
origin ← closest_point_on_segment(robot_xy, path[idx], path[idx+1])
local_goal, clip ← clip_forward(path, idx, origin, horizon)          # 2
if band is empty or dist(band[-1], local_goal) > reinit_distance:    # 3
    band ← init_band(resample(clip, v_max * dt_ref))
else:
    while len(band) > 2 and segment_t(robot_xy, band[0], band[1]) >= 1:  # 4
        pop_front(band)
band[0] ← (x, y, theta)                                              # 5
band[-1] ← local_goal
if len(band) < 3:                                                    # 6
    return heading_control(local_goal)
resize(band)                                                          # 7
anchors ← [nearest_point_on(clip, p_i) for interior i]                # 8
for k in 1..iterations:                                               # 9
    g ← 0
    accumulate: path -> obstacle -> velocity -> acceleration -> kinematics -> time   # 10
    for i in interior poses:
        p_i ← p_i - clamp(step_alpha * g[p_i], max_step_xy, max_step_theta)
    for i in segments:
        dt_i ← max(dt_min, dt_i - clamp(step_alpha * g[dt_i], max_step_dt))          # 11
emit(band_updated)
v ← clamp(sign(d_0 . heading) * ell_0 / dt_0, v_max)                  # 12
omega ← clamp(wrap(theta_1 - theta_0) / dt_0, omega_max)
return (v, omega)`}/>
            <T
                en={<ol>
                    <li>Project the robot forward-only onto the reference path's segment index — this is the
                        band's guide rail, distinct from the band's own warm-start cursor two steps below.</li>
                    <li>Clip the reference path from that projection out to a fixed horizon; its endpoint is
                        the local goal for this tick.</li>
                    <li>Rebuild from scratch if there is no band yet, or if last tick's band has drifted too
                        far from where the local goal is now (the horizon slides as the robot advances, so the
                        band's far end has to occasionally jump to follow it).</li>
                    <li>Otherwise warm-start: drop poses the robot has already advanced past, judged against
                        the band's <em>own</em> first segment — not the reference-path progress index from
                        step 1, which tracks a different, un-warped line.</li>
                    <li>Re-pin both endpoints: the executed pose behind, the (possibly just-recomputed) local
                        goal ahead.</li>
                    <li><strong>Degenerate case:</strong> if re-pinning left fewer than three poses, there is
                        no interior left to optimize — the robot is essentially at the local goal already.
                        Skip the solver entirely and steer straight at it with plain heading control, the same
                        structure Elastic Bands uses for its own command extraction.</li>
                    <li>Resize: split any segment whose <InlineMath math="\Delta T"/> grew past{" "}
                        <InlineMath math="1.5 \cdot \Delta T_{\text{ref}}"/> with a wrap-aware midpoint pose,
                        merge any that shrank below <InlineMath math="0.5 \cdot \Delta T_{\text{ref}}"/> into
                        its neighbor — keeps the pose count from ballooning or collapsing as the band deforms
                        tick after tick.</li>
                    <li>Fix every interior pose's path-attraction anchor once, before optimization moves
                        anything — the nearest point on the clipped reference path to that pose's position{" "}
                        <em>right now</em>. Recomputing anchors mid-optimization would make the term chase a
                        target that keeps running from it.</li>
                    <li>Run a fixed number of gradient steps — no convergence check, no early exit, for the
                        same cross-language determinism reason as Elastic Bands' fixed deformation count.</li>
                    <li><strong>The pitfall that determinism depends on:</strong> every term's gradient must
                        accumulate in the same fixed order every time — path, then obstacle, then velocity,
                        then acceleration, then kinematics, then time, each with poses visited in ascending
                        index. Floating-point addition is not associative; summing in a different order can
                        produce a different last bit, and after enough iterations a different bit can flip a
                        discrete branch (which segment resize splits, say) into a different trajectory
                        entirely.</li>
                    <li><strong>The other pitfall:</strong> the <InlineMath math="\Delta T"/> update floors at{" "}
                        <InlineMath math="\Delta T_{\min}"/> before anything downstream divides by it. Skip the
                        floor and a large enough gradient step can push <InlineMath math="\Delta T"/> to zero
                        or negative, and every velocity computed from it next iteration is a division by zero
                        or a sign flip that sends the whole chain diverging.</li>
                    <li>Only the very first segment ever produces a command — everything past it exists purely
                        to shape where that first segment points and how much time it gets. The sign flag lets
                        the robot execute a short reverse move if the optimized first step ended up behind the
                        robot's current heading.</li>
                </ol>}
                ko={<ol>
                    <li>로봇을 참조 경로의 세그먼트 index에 전진 전용으로 사영한다. 이것이 밴드의
                        가이드레일이고, 아래 4번의 밴드 자신의 warm-start 커서와는 다른 것이다.</li>
                    <li>그 사영점부터 고정 horizon까지 참조 경로를 잘라낸다. 그 끝점이 이번 tick의
                        local goal이다.</li>
                    <li>밴드가 아직 없거나, 지난 tick의 밴드가 지금 local goal 위치에서 너무 멀어졌다면
                        (로봇이 나아가며 horizon이 밀려가므로, 밴드의 먼 쪽 끝이 이따금 그것을 따라
                        점프해야 한다) 처음부터 다시 짓는다.</li>
                    <li>그렇지 않으면 warm-start한다. 로봇이 이미 지나친 pose를 버리되, 1번의 참조 경로
                        진행 index가 아니라 밴드 <em>자신의</em> 첫 구간을 기준으로 판단한다(서로 다른,
                        휘어지지 않은 별개의 선을 추적하는 값이기 때문이다).</li>
                    <li>양 끝점을 다시 고정한다. 뒤는 실제로 도달한 pose로, 앞은 (방금 다시 계산됐을 수도
                        있는) local goal로.</li>
                    <li><strong>퇴화 케이스.</strong> 재고정 후 pose가 셋 미만으로 남았다면 최적화할
                        내부가 아예 없다. 로봇이 사실상 local goal에 이미 도달한 것이다. 솔버를 완전히
                        생략하고 단순 heading 제어로 그쪽을 향해 직진한다. Elastic Bands가 자신의 명령
                        추출에 쓰는 것과 같은 구조다.</li>
                    <li>Resize. <InlineMath math="\Delta T"/>가 <InlineMath math="1.5 \cdot \Delta
                        T_{\text{ref}}"/>보다 커진 구간은 wrap-aware 중간각 midpoint pose로 쪼개고,{" "}
                        <InlineMath math="0.5 \cdot \Delta T_{\text{ref}}"/> 아래로 줄어든 구간은 이웃과
                        합친다. 밴드가 tick마다 변형되면서 pose 수가 부풀거나 무너지지 않게 한다.</li>
                    <li>최적화가 무엇 하나 움직이기 전에, 모든 내부 pose의 경로 추종 anchor를 한 번만
                        고정한다. <em>바로 지금</em> 그 pose 위치에서 clip된 참조 경로 위 최근접점이다.
                        최적화 도중 anchor를 다시 계산하면 그 항은 계속 도망가는 목표를 뒤쫓게 된다.</li>
                    <li>고정 횟수만큼 gradient step을 돌린다. 수렴 검사도, 조기 종료도 없다. Elastic
                        Bands가 변형 횟수를 고정하는 것과 같은, 언어 간 결정론을 위한 이유다.</li>
                    <li><strong>결정론이 걸려 있는 함정.</strong> 모든 항의 gradient는 매번 같은 고정
                        순서로 누적돼야 한다. path, obstacle, velocity, acceleration, kinematics, time
                        순서로, 각 항 안에서는 pose를 index 오름차순으로. 부동소수 덧셈은 결합법칙이
                        성립하지 않는다. 다른 순서로 더하면 마지막 비트가 달라질 수 있고, 반복이 충분히
                        쌓이면 그 비트 하나가 이산 분기(예: 어느 구간이 resize에서 쪼개지는지)를 뒤집어
                        완전히 다른 궤적으로 갈라질 수 있다.</li>
                    <li><strong>또 다른 함정.</strong> <InlineMath math="\Delta T"/> 갱신은 이후 어디선가
                        그것으로 나누기 전에 <InlineMath math="\Delta T_{\min}"/>으로 바닥을 둔다. 이
                        바닥을 빠뜨리면 충분히 큰 gradient step이 <InlineMath math="\Delta T"/>를 0이나
                        음수로 밀어낼 수 있고, 다음 반복에서 그것으로 계산되는 모든 속도가 0으로 나누기나
                        부호 반전이 되어 사슬 전체가 발산한다.</li>
                    <li>오직 맨 첫 구간만 명령을 만든다. 그 뒤 전부는 그 첫 구간이 어디를 가리키고 얼마의
                        시간을 받는지를 빚는 데만 쓰인다. 부호 플래그는 최적화된 첫 step이 로봇의 현재
                        heading보다 뒤쪽으로 나왔을 때 짧은 후진을 실행하게 해 준다.</li>
                </ol>}
            />

            <Proof title={t(
                "Derivation (the two-pose arc constraint)",
                "유도 (two-pose 원호 제약)",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Assume poses <InlineMath math="s_i = (p_i, \theta_i)"/>{" "}
                            and <InlineMath math="s_{i+1} = (p_{i+1}, \theta_{i+1})"/> lie on a common arc of
                            constant curvature — the two-pose arc model. A classical property of circles (the
                            tangent-chord angle theorem) says the chord between two points on a circle makes
                            equal angles with the tangent lines at each point, which means the chord's
                            direction bisects the two tangent directions <InlineMath math="\theta_i"/> and{" "}
                            <InlineMath math="\theta_{i+1}"/>.
                        </p>
                        <p>
                            A general vector fact does the rest of the work without any trigonometric
                            angle-averaging: the sum of two unit vectors always points along the bisector of
                            the angle between them. So the bisector direction is exactly:
                        </p>
                        <BlockMath math="u_i = (\cos\theta_i + \cos\theta_{i+1},\ \sin\theta_i + \sin\theta_{i+1})"/>
                        <Terms items={[
                            ["u_i", "sum of the two endpoint heading unit vectors — points along their angle bisector"],
                            ["\\theta_i,\\ \\theta_{i+1}", "headings at the two poses"],
                        ]}/>
                        <p>
                            The two-pose arc assumption requires this bisector to be parallel to the chord{" "}
                            <InlineMath math="d_i = p_{i+1} - p_i"/>. Two 2D vectors{" "}
                            <InlineMath math="u = (u_x, u_y)"/> and <InlineMath math="w = (w_x, w_y)"/> are
                            parallel exactly when their cross product vanishes:
                        </p>
                        <BlockMath math="u_x w_y - u_y w_x = 0"/>
                        <Terms items={[
                            ["u_x w_y - u_y w_x", "2D cross product of u and w — zero iff the two vectors are parallel (or either is zero)"],
                        ]}/>
                        <p>
                            Substituting <InlineMath math="u = u_i"/> and <InlineMath math="w = d_i =
                            (\Delta x_i, \Delta y_i)"/>:
                        </p>
                        <BlockMath math="(\cos\theta_i + \cos\theta_{i+1})\, \Delta y_i - (\sin\theta_i + \sin\theta_{i+1})\, \Delta x_i = 0"/>
                        <Terms items={[
                            ["\\Delta x_i,\\ \\Delta y_i", "components of the chord d_i = p_{i+1} - p_i"],
                        ]}/>
                        <p>
                            The left-hand side is exactly <InlineMath math="h_i"/> from the cost term above, so
                            the two-pose arc assumption is equivalent to <InlineMath math="h_i = 0"/>. Squaring
                            and weighting it turns an exact constraint into a soft penalty: poses that satisfy
                            it exactly cost nothing, and the further a pose pair drifts from lying on a common
                            arc, the more it costs.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> pose <InlineMath math="s_i = (p_i, \theta_i)"/>와{" "}
                            <InlineMath math="s_{i+1} = (p_{i+1}, \theta_{i+1})"/>이 등곡률을 갖는 공통
                            원호 위에 있다고 하자. two-pose 원호 모델이다. 원의 고전적 성질(접선-현
                            각도 정리)에 따르면, 원 위 두 점을 잇는 현은 각 점의 접선과 같은 각을
                            이룬다. 이는 곧 현의 방향이 두 접선 방향 <InlineMath math="\theta_i"/>와{" "}
                            <InlineMath math="\theta_{i+1}"/>을 이등분한다는 뜻이다.
                        </p>
                        <p>
                            삼각함수로 각도를 평균 내지 않고도, 일반적인 벡터 성질 하나가 나머지를
                            해결한다. 두 단위벡터의 합은 항상 그 둘 사이 각의 이등분선 방향을
                            가리킨다. 그래서 이등분선 방향은 정확히:
                        </p>
                        <BlockMath math="u_i = (\cos\theta_i + \cos\theta_{i+1},\ \sin\theta_i + \sin\theta_{i+1})"/>
                        <Terms items={[
                            ["u_i", "두 pose의 heading 단위벡터의 합. 그 둘의 각 이등분선 방향을 가리킨다"],
                            ["\\theta_i,\\ \\theta_{i+1}", "두 pose에서의 heading"],
                        ]}/>
                        <p>
                            two-pose 원호 가정은 이 이등분선이 현 <InlineMath math="d_i = p_{i+1} -
                            p_i"/>와 평행할 것을 요구한다. 2D 벡터 <InlineMath math="u = (u_x, u_y)"/>와{" "}
                            <InlineMath math="w = (w_x, w_y)"/>는 외적이 0일 때 정확히 평행이다:
                        </p>
                        <BlockMath math="u_x w_y - u_y w_x = 0"/>
                        <Terms items={[
                            ["u_x w_y - u_y w_x", "u와 w의 2D 외적. 두 벡터가 평행(또는 어느 하나가 영벡터)일 때만 0"],
                        ]}/>
                        <p>
                            <InlineMath math="u = u_i"/>, <InlineMath math="w = d_i = (\Delta x_i, \Delta
                            y_i)"/>를 대입하면:
                        </p>
                        <BlockMath math="(\cos\theta_i + \cos\theta_{i+1})\, \Delta y_i - (\sin\theta_i + \sin\theta_{i+1})\, \Delta x_i = 0"/>
                        <Terms items={[
                            ["\\Delta x_i,\\ \\Delta y_i", "현 d_i = p_{i+1} - p_i의 성분"],
                        ]}/>
                        <p>
                            좌변은 위 비용 항의 <InlineMath math="h_i"/>와 정확히 같다. 따라서 two-pose
                            원호 가정은 <InlineMath math="h_i = 0"/>과 동치다. 이를 제곱하고 가중치를
                            매기면 정확한 제약이 soft 페널티로 바뀐다. 정확히 만족하는 pose 쌍은 비용이
                            0이고, 공통 원호에서 벗어날수록 비용이 커진다.
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("The Homotopy Trap", "Homotopy 함정")}</h2>
            <T
                en={<p>
                    Every deformation step moves each pose a small, continuous distance, so the band can
                    slide along an obstacle but can never jump <em>across</em> one: whichever side of an
                    obstacle the reference path chose, the optimizer is confined to that homotopy class and
                    only polishes the detour within it. The demo below makes the cost visible — a straight,
                    2 m-wide corridor is open under the block, yet because the reference path rounds the
                    block from above, the band dutifully optimizes the long way and the robot travels
                    roughly 19 m where 7.5 m would do. Later work fixes exactly this by keeping several TEBs
                    alive in parallel, one per distinct homotopy class, and switching to the cheapest
                    (Rösmann et al. 2017, in the references).
                </p>}
                ko={<p>
                    변형 한 스텝은 각 pose를 짧고 연속적인 거리만큼만 옮긴다. 그래서 밴드는 장애물을
                    따라 미끄러질 수는 있어도 장애물을 <em>건너뛰지는</em> 못한다. reference path가
                    장애물의 어느 쪽을 골랐든 최적화는 그 homotopy 부류 안에 갇혀, 그 안의 우회로를
                    다듬을 뿐이다. 아래 데모가 그 대가를 눈으로 보여준다. 블록 아래로 폭 2m 직선 통로가
                    열려 있는데도 reference path가 블록을 위로 돌기 때문에, 밴드는 충실하게 먼 길을
                    최적화하고 로봇은 7.5m면 될 길을 약 19m 돌아간다. 후속 연구는 정확히 이 문제를,
                    서로 다른 homotopy 부류마다 TEB를 하나씩 병렬로 살려 두고 가장 싼 것으로 갈아타는
                    방식으로 고친다(참고문헌의 Rösmann 2017).
                </p>}
            />
            <TebHomotopyDemo/>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs TEB live in your browser. The pose chain is drawn with segment
                    thickness proportional to its own <InlineMath math="\Delta T"/> — thin segments are fast,
                    thick ones are slow. Raise <InlineMath math="w_{\text{time}}"/> and watch the chain pull
                    tighter through each corner; switch to the sharp-corner preset to see the turn-rate limit
                    force a real, visible slowdown rather than an invisible one.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 TEB를 라이브로 실행한다. pose 사슬은 자신의{" "}
                    <InlineMath math="\Delta T"/>에 비례한 굵기로 그려진다. 가는 구간은 빠르고, 굵은
                    구간은 느리다. <InlineMath math="w_{\text{time}}"/>을 올리며 사슬이 각 코너를 더
                    바짝 당겨 지나가는 모습을 보거나, sharp corner 프리셋으로 바꿔 회전율 한계가 눈에
                    보이지 않는 감속이 아니라 실제로 뚜렷한 감속을 강제하는 모습을 보라.
                </p>}
            />
            <TebSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below follows the algorithm above line for line: progress projection
                    and clip, warm start versus reinitialization, resize, fixed-order gradient accumulation,
                    and the clamped update. The code is the actual repository source, not an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 거의 그대로 옮긴 것이다. 진행 사영과 clip, warm start와
                    재초기화 판정, resize, 고정 순서 gradient 누적, clamp 갱신까지 그대로다. 아래 코드는
                    발췌가 아니라 저장소의 실제 소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/band/teb.py",
                                code: tebPy,
                                href: `${REPO}/python/navigation/local_planning/band/teb.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/band/teb.hpp",
                                code: tebHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/band/teb.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/band/teb.cpp",
                                code: tebCpp,
                                href: `${REPO}/cpp/src/local_planning/band/teb.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "Progress projection and clip, warm start versus reinitialization, resize, and the fixed-order gradient solver, embedded from the repository sources",
                    "진행 사영과 clip, warm start와 재초기화 판정, resize, 고정 순서 gradient 솔버. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    C. Rösmann, F. Feiten, T. Wösch, F. Hoffmann, T. Bertram,{" "}
                    <em>Trajectory Modification Considering Dynamic Constraints of Autonomous Robots</em>,
                    Proceedings of ROBOTIK 2012, 7th German Conference on Robotics.{" "}
                    <a href="https://ieeexplore.ieee.org/document/6309484" target="_blank"
                       rel="noopener noreferrer">
                        IEEE Xplore 6309484
                    </a>
                </li>
                <li>
                    C. Rösmann, F. Feiten, T. Wösch, F. Hoffmann, T. Bertram,{" "}
                    <a href="https://doi.org/10.1109/ECMR.2013.6698833" target="_blank" rel="noopener noreferrer">
                        <em>Efficient Trajectory Optimization Using a Sparse Model</em>
                    </a>, Proceedings of the European Conference on Mobile Robots (ECMR) 2013.
                </li>
                <li>
                    C. Rösmann, F. Hoffmann, T. Bertram,{" "}
                    <a href="https://doi.org/10.1016/j.robot.2016.11.007" target="_blank" rel="noopener noreferrer">
                        <em>Integrated Online Trajectory Planning and Optimization in Distinctive Topologies</em>
                    </a>, Robotics and Autonomous Systems, vol. 88, pp. 142–153, 2017.
                </li>
            </ol>
        </>
    )
}

export default Teb
