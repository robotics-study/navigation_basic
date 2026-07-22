import {T, useTr} from "../../libs/i18n";
import {BlockMath, InlineMath} from "../../components/math/Tex";
import Terms from "../../components/math/Terms";
import ArcCandidates from "../../components/panels/intro/ArcCandidates";
import LocalVelocityWindow from "../../components/panels/intro/LocalVelocityWindow";
import LocalPursuitGeometry from "../../components/panels/intro/LocalPursuitGeometry";

const LocalPlanning = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Local planning is the part of navigation that actually moves the robot. The
                    global planner hands over a path computed on a map; the local planner turns it
                    into velocity commands, right now, against the world as the sensors currently
                    see it.
                </p>}
                ko={<p>
                    Local planning은 navigation에서 로봇을 실제로 움직이는 부분이다. global
                    planner가 지도 위에서 계산한 경로를 넘겨주면, local planner는 지금 이 순간
                    센서가 보고 있는 세계에 맞춰 그것을 속도 명령으로 바꾼다.
                </p>}
            />

            <h2>{t("Why a Second Planner", "왜 planner가 하나 더 필요한가")}</h2>
            <T
                en={<>
                    <p>
                        Three things the global plan cannot handle force a second layer:
                    </p>
                    <ul>
                        <li><strong>The map lies.</strong> People walk by, doors close, furniture
                            moves. The path was correct for a world that no longer exists.</li>
                        <li><strong>The robot is not a point.</strong> Real platforms have velocity
                            and acceleration limits, and often nonholonomic constraints — a
                            differential-drive robot cannot slide sideways onto the path.</li>
                        <li><strong>Time.</strong> Control runs at 10–100 Hz. Replanning globally at
                            that rate is wasteful when 99% of the plan is still fine.</li>
                    </ul>
                    <p>
                        So the stack splits deliberation from reflex: the global planner thinks
                        rarely and far ahead; the local planner thinks constantly, but only a few
                        seconds into the future.
                    </p>
                </>}
                ko={<>
                    <p>
                        global plan이 감당할 수 없는 세 가지가 두 번째 계층을 강제한다:
                    </p>
                    <ul>
                        <li><strong>지도는 현실을 다 담지 못한다.</strong> 사람이 지나가고, 문이 닫히고,
                            가구가 움직인다. 경로가 계산된 시점의 세계는 이미 지나갔다.</li>
                        <li><strong>로봇은 점이 아니다.</strong> 실제 플랫폼에는 속도·가속 한계가
                            있고, 대개 nonholonomic 제약도 있다. differential-drive 로봇은 경로
                            위로 옆걸음질할 수 없다.</li>
                        <li><strong>시간.</strong> 제어는 10–100 Hz로 돈다. 계획의 99%가 멀쩡한데
                            그 주기로 전역 재계획을 하는 것은 낭비다.</li>
                    </ul>
                    <p>
                        그래서 스택은 계획과 반응을 분리한다. global planner는 드물게, 멀리
                        내다보며 생각하고, local planner는 끊임없이, 그러나 몇 초 앞까지만
                        생각한다.
                    </p>
                </>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    At each control cycle: given the current state (pose, velocity), the latest
                    sensor view of nearby obstacles, and a reference path, choose a
                    command <InlineMath math="(v, \omega)"/> that (1) makes progress along the path,
                    (2) avoids collision over a short horizon, and (3) respects the platform's
                    kinematic and dynamic limits. It is an optimization under a deadline: a good
                    answer in 10 ms beats a perfect answer in 100 ms.
                </p>}
                ko={<p>
                    매 제어 주기마다: 현재 상태(자세, 속도), 주변 장애물에 대한 최신 센서 뷰,
                    참조 경로가 주어졌을 때, (1) 경로를 따라 전진하고 (2) 짧은 horizon 안에서
                    충돌을 피하며 (3) 플랫폼의 기구학·동역학 한계를 지키는 명령{" "}
                    <InlineMath math="(v, \omega)"/>을 고른다. 마감이 있는 최적화다: 10 ms 안의
                    좋은 답이 100 ms 뒤의 완벽한 답을 이긴다.
                </p>}
            />

            <ArcCandidates/>

            <h2>{t("Families of Local Planners", "Local planner의 계열")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Sampling in command space</strong> — enumerate feasible commands,
                            forward-simulate each for a short horizon, score the trajectories
                            (progress, clearance, speed), pick the best. DWA is the canonical
                            example.
                        </li>
                        <li>
                            <strong>Geometric path tracking</strong> — assume the path is fine and
                            chase a point on it; Pure Pursuit steers toward a lookahead point with
                            one clean geometric rule. Simple, fast, and blind to obstacles — pair it
                            with something that isn't.
                        </li>
                        <li>
                            <strong>Reactive steering from sensor histograms</strong> — VFH builds a
                            polar obstacle density around the robot and picks the best open sector.
                        </li>
                        <li>
                            <strong>Optimization over trajectories</strong> — write progress,
                            clearance, and dynamics as costs/constraints and solve each cycle. MPC
                            solves a receding-horizon optimal control problem; TEB deforms an elastic
                            band of poses in time. Most capable, most expensive.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>명령 공간 sampling</strong>: 실행 가능한 명령을 열거하고, 각각을
                            짧은 horizon으로 전방 시뮬레이션한 뒤 궤적을 채점(전진, 여유 거리,
                            속도)해 최선을 고른다. DWA가 표준 예다.
                        </li>
                        <li>
                            <strong>기하학적 경로 추종</strong>: 경로가 괜찮다고 가정하고 그 위의
                            점을 쫓는다. Pure Pursuit은 lookahead 점을 향해 하나의 깔끔한 기하
                            규칙으로 조향한다. 단순하고 빠르지만 장애물을 못 보므로, 장애물을 보는
                            다른 계층과 함께 쓴다.
                        </li>
                        <li>
                            <strong>센서 히스토그램 기반 반응 조향</strong>: VFH는 로봇 주변의
                            극좌표 장애물 밀도를 만들고 가장 좋은 열린 sector를 고른다.
                        </li>
                        <li>
                            <strong>궤적 최적화</strong>: 전진·여유·동역학을 비용/제약으로 적고 매
                            주기 푼다. MPC는 receding-horizon 최적 제어 문제를 풀고, TEB는 시간
                            축을 가진 elastic band를 변형한다. 가장 유능하고 가장 비싸다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("Two Geometries Up Close", "두 기하를 가까이서")}</h2>
            <T
                en={<p>
                    The arc fan above lives in the <em>workspace</em> — the paths the robot could
                    trace on the floor. DWA does its actual choosing in <em>command</em> space, the
                    plane of velocities <InlineMath math="(v, \omega)"/>. The trick that makes it
                    real-time is that it never considers every velocity: from the current velocity
                    it can only reach a small box in one control cycle, bounded by the acceleration
                    limits. That box is the <strong>dynamic window</strong>.
                </p>}
                ko={<p>
                    위의 부채꼴 호들은 <em>작업 공간</em>, 곧 로봇이 바닥에 그릴 수 있는 경로에
                    산다. DWA가 실제로 고르는 곳은 속도 <InlineMath math="(v, \omega)"/>의 평면인{" "}
                    <em>명령 공간</em>이다. 실시간을 가능하게 하는 요령은 모든 속도를 보지 않는
                    데 있다. 현재 속도에서 한 제어 주기에 도달할 수 있는 범위는 가속 한계가 두른
                    작은 상자뿐이고, 그 상자가 <strong>dynamic window</strong>다.
                </p>}
            />
            <BlockMath math="V_d = \{\,(v,\omega) : v \in [v_c - a_v\Delta t,\; v_c + a_v\Delta t],\;\; \omega \in [\omega_c - a_\omega\Delta t,\; \omega_c + a_\omega\Delta t]\,\}"/>
            <Terms items={[
                ["V_d", t("the dynamic window — velocities reachable within one cycle",
                    "dynamic window, 한 주기에 도달 가능한 속도 집합")],
                ["v,\\ \\omega", t("a candidate translational / angular velocity",
                    "후보 병진 속도 / 각속도")],
                ["v_c,\\ \\omega_c", t("the robot's current translational / angular velocity",
                    "로봇의 현재 병진 속도 / 각속도")],
                ["a_v,\\ a_\\omega", t("translational / angular acceleration limits",
                    "병진 / 각 가속 한계")],
                ["\\Delta t", t("one control cycle", "한 제어 주기")],
            ]}/>
            <LocalVelocityWindow/>

            <T
                en={<p>
                    Pure Pursuit needs no such search. It assumes the reference path is good and
                    only asks how to steer toward it: draw a circle of
                    radius <InlineMath math="L_a"/> around the robot, take the point where it
                    crosses the path, and follow the one arc that passes through that point. The
                    whole controller is a single curvature.
                </p>}
                ko={<p>
                    Pure Pursuit에는 그런 탐색이 없다. 참조 경로가 좋다고 가정하고 그쪽으로 어떻게
                    조향할지만 묻는다. 로봇을 중심으로 반경 <InlineMath math="L_a"/>의 원을 그려
                    경로와 만나는 점을 잡고, 그 점을 지나는 유일한 원호를 따라간다. 제어기 전체가
                    곡률 하나다.
                </p>}
            />
            <BlockMath math="\kappa = \frac{2\,\sin\alpha}{L_a}, \qquad R = \frac{1}{\kappa}"/>
            <Terms items={[
                ["\\kappa", t("curvature of the arc the robot follows",
                    "로봇이 따라가는 원호의 곡률")],
                ["R", t("radius of that arc (= 1/\\kappa)", "그 원호의 반경 (= 1/κ)")],
                ["L_a", t("lookahead distance — radius of the lookahead circle",
                    "lookahead 거리, lookahead 원의 반경")],
                ["\\alpha", t("angle between the robot's heading and the lookahead point",
                    "로봇의 진행 방향과 lookahead 점 사이 각")],
            ]}/>
            <LocalPursuitGeometry/>

            <h2>{t("What Is Coming", "구현 예정")}</h2>
            <T
                en={<p>
                    The reactive branch (<strong>Potential Fields</strong>, <strong>VFH</strong>,{" "}
                    <strong>DWA</strong>), the geometric tracking branch (<strong>Pure
                    Pursuit</strong>, <strong>Stanley</strong>, <strong>Regulated Pure
                    Pursuit</strong>), and the band-deformation pair (<strong>Elastic
                    Bands</strong>, <strong>Timed Elastic Band</strong>) are all written. What
                    remains closes the optimization lineage: <strong>MPC</strong> and{" "}
                    <strong>MPPI</strong> with full receding-horizon trajectory optimization.
                </p>}
                ko={<p>
                    반응형 가지(<strong>Potential Fields</strong>, <strong>VFH</strong>,{" "}
                    <strong>DWA</strong>), 기하학적 추종 가지(<strong>Pure Pursuit</strong>,{" "}
                    <strong>Stanley</strong>, <strong>Regulated Pure Pursuit</strong>), 그리고
                    밴드 변형 짝(<strong>Elastic Bands</strong>, <strong>Timed Elastic
                    Band</strong>)까지 모두 채워졌다. 남은 것은 최적화 계보의 마무리다.{" "}
                    <strong>MPC</strong>와 <strong>MPPI</strong>가 receding-horizon 궤적
                    최적화로 이 섹션을 완성한다.
                </p>}
            />
        </>
    )
}

export default LocalPlanning
