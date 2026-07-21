import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import ArcCandidates from "../../components/panels/intro/ArcCandidates";

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
                    Local planning 은 navigation 에서 로봇을 실제로 움직이는 부분이다. global
                    planner 가 지도 위에서 계산한 경로를 넘겨주면, local planner 는 그것을 —
                    지금 이 순간, 센서가 보고 있는 세계에 맞춰 — 속도 명령으로 바꾼다.
                </p>}
            />

            <h2>{t("Why a Second Planner", "왜 planner 가 하나 더 필요한가")}</h2>
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
                        global plan 이 감당할 수 없는 세 가지가 두 번째 계층을 강제한다:
                    </p>
                    <ul>
                        <li><strong>지도는 거짓말을 한다.</strong> 사람이 지나가고, 문이 닫히고,
                            가구가 움직인다. 경로는 더 이상 존재하지 않는 세계에 대해 옳았다.</li>
                        <li><strong>로봇은 점이 아니다.</strong> 실제 플랫폼에는 속도·가속 한계가
                            있고, 대개 nonholonomic 제약도 있다 — differential-drive 로봇은 경로
                            위로 옆걸음질할 수 없다.</li>
                        <li><strong>시간.</strong> 제어는 10–100 Hz 로 돈다. 계획의 99%가 멀쩡한데
                            그 주기로 전역 재계획을 하는 것은 낭비다.</li>
                    </ul>
                    <p>
                        그래서 스택은 숙고와 반사를 나눈다: global planner 는 드물게, 멀리 내다보며
                        생각하고 — local planner 는 끊임없이, 그러나 몇 초 앞까지만 생각한다.
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
                    <InlineMath math="(v, \omega)"/> 을 고른다. 마감이 있는 최적화다: 10 ms 안의
                    좋은 답이 100 ms 뒤의 완벽한 답을 이긴다.
                </p>}
            />

            <ArcCandidates/>

            <h2>{t("Families of Local Planners", "Local planner 의 계열")}</h2>
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
                            <strong>명령 공간 sampling</strong> — 실행 가능한 명령을 열거하고, 각각을
                            짧은 horizon 으로 전방 시뮬레이션한 뒤 궤적을 채점(전진, 여유 거리,
                            속도)해 최선을 고른다. DWA 가 표준 예다.
                        </li>
                        <li>
                            <strong>기하학적 경로 추종</strong> — 경로가 괜찮다고 가정하고 그 위의
                            점을 쫓는다; Pure Pursuit 은 lookahead 점을 향해 하나의 깔끔한 기하
                            규칙으로 조향한다. 단순하고 빠르지만 장애물을 못 본다 — 보는 것과 짝을
                            지어 쓴다.
                        </li>
                        <li>
                            <strong>센서 히스토그램 기반 반응 조향</strong> — VFH 는 로봇 주변의
                            극좌표 장애물 밀도를 만들고 가장 좋은 열린 sector 를 고른다.
                        </li>
                        <li>
                            <strong>궤적 최적화</strong> — 전진·여유·동역학을 비용/제약으로 적고 매
                            주기 푼다. MPC 는 receding-horizon 최적 제어 문제를 풀고, TEB 는 시간
                            축을 가진 elastic band 를 변형한다. 가장 유능하고 가장 비싸다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("What Is Coming", "구현 예정")}</h2>
            <T
                en={<p>
                    Planned for this section: <strong>DWA</strong>, <strong>Pure Pursuit</strong>,{" "}
                    <strong>VFH</strong>, and <strong>MPC</strong> — one representative per family —
                    implemented in C++ and Python against the{" "}
                    <code>ObstacleQuery</code> capability, with the same live demos and recorded
                    traces as the global planners. Trajectory-optimization variants such as TEB are
                    natural follow-ups.
                </p>}
                ko={<p>
                    이 섹션의 구현 예정: <strong>DWA</strong>, <strong>Pure Pursuit</strong>,{" "}
                    <strong>VFH</strong>, <strong>MPC</strong> — 계열별 대표 하나씩 — 을{" "}
                    <code>ObstacleQuery</code> capability 위에서 C++/Python 으로 구현하고, global
                    planner 와 같은 라이브 demo·기록 trace 를 제공한다. TEB 같은 궤적 최적화
                    변형이 자연스러운 후속이다.
                </p>}
            />
        </>
    )
}

export default LocalPlanning
