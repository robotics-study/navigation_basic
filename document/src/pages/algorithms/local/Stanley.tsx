import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import StanleySandbox from "../../../components/panels/local/stanley/StanleySandbox";
import StanleyGeometryFigure from "../../../components/panels/local/stanley/StanleyGeometryFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import stanleyPy from "../../../../../python/navigation/local_planning/tracking/stanley.py?raw";
import stanleyHpp from "../../../../../cpp/include/navigation/local_planning/tracking/stanley.hpp?raw";
import stanleyCpp from "../../../../../cpp/src/local_planning/tracking/stanley.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도/증명 블록. 본문 흐름은 직관 중심으로 유지하고, 형식적 전개는 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Stanley = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Pure Pursuit throws away one piece of information every tick: how the robot is
                    <em> pointed</em> relative to the path, as opposed to where it sits relative to it.
                    Stanley steers on both at once. Gabriel Hoffmann, Sebastian Thrun and colleagues built
                    it for Stanford's "Stanley" vehicle, winner of the 2005 DARPA Grand Challenge, and the
                    two-error design has powered path trackers ever since — a separate branch of the
                    lineage from Pure Pursuit's single-arc geometry, not a refinement of it.
                </p>}
                ko={<p>
                    Pure Pursuit은 매 tick마다 정보 하나를 버린다. 로봇이 경로에 대해 어디 있는지는
                    보지만, 어느 방향을 <em>향하고</em> 있는지는 보지 않는다. Stanley는 이 둘을 동시에
                    다룬다. Gabriel Hoffmann, Sebastian Thrun을 비롯한 팀이 2005년 DARPA Grand
                    Challenge에서 우승한 스탠퍼드의 차량 "Stanley"를 위해 만든 방법이고, 이후 두
                    오차를 함께 쓰는 설계가 경로 추종 알고리즘의 한 축을 이뤄 왔다. Pure Pursuit의
                    단일 원호 기하를 다듬은 결과가 아니라, 계보에서 갈라져 나온 독립된 가지다.
                </p>}
            />

            <h2>{t("Two Errors, One Steering Law", "두 오차, 하나의 조향 법칙")}</h2>
            <T
                en={<>
                    <p>
                        At every control step, Stanley measures two things at once: how far off the
                        robot's heading is from the path's tangent direction, and how far off to the side
                        it sits. One steering angle corrects both:
                    </p>
                    <BlockMath math="\delta = \psi + \arctan\!\left(\frac{k\,e}{v + k_{soft}}\right)"/>
                    <Terms items={[
                        ["\\delta", "commanded steering angle — the one number this law outputs each tick"],
                        ["\\psi", <>heading error: the path tangent's bearing minus the robot's heading,
                            wrapped to <InlineMath math="(-\pi, \pi]"/></>],
                        ["e", <>crosstrack error — signed lateral distance from the path, measured at the
                            front axle; positive when the axle sits to the path's <em>right</em> in this
                            equation's convention (the repository's own e flips this sign — see "From
                            Bicycle to Unicycle" below — but computes the identical steering angle)</>],
                        ["k", "crosstrack gain — how aggressively lateral offset gets steered out"],
                        ["k_{soft}", <>a small softening constant that keeps the arctan term finite as{" "}
                            <InlineMath math="v \to 0"/></>],
                        ["v", "current forward speed"],
                    ]}/>
                    <p>
                        The heading term <InlineMath math="\psi"/> alone would just aim the robot parallel
                        to the path, ignoring how far away it is; the crosstrack term alone would chase the
                        path but overshoot past it every time the heading catches up. Added together, the
                        two terms hand off to each other automatically — crosstrack dominates while the
                        robot is far off the path, heading dominates once it's aligned and just needs to
                        stay there.
                    </p>
                </>}
                ko={<>
                    <p>
                        Stanley는 매 제어 스텝마다 두 가지를 동시에 잰다. 로봇의 heading이 경로 접선
                        방향에서 얼마나 벗어났는지, 그리고 옆으로 얼마나 비켜서 있는지. 조향각 하나가
                        둘을 함께 바로잡는다.
                    </p>
                    <BlockMath math="\delta = \psi + \arctan\!\left(\frac{k\,e}{v + k_{soft}}\right)"/>
                    <Terms items={[
                        ["\\delta", "명령되는 조향각. 이 법칙이 매 tick 내놓는 값 하나"],
                        ["\\psi", <>heading 오차. 경로 접선의 방위에서 로봇 heading을 뺀 값을{" "}
                            <InlineMath math="(-\pi, \pi]"/>로 wrap</>],
                        ["e", <>crosstrack 오차. 전륜축에서 잰 경로까지의 부호 있는 측방 거리다. 이
                            식의 표기에서는 전륜축이 경로 <em>오른쪽</em>에 있을 때 양수다 (저장소
                            구현은 이 부호를 뒤집어 쓴다. 아래 "자전거 모델에서 차동 구동으로" 절
                            참고. 다만 계산되는 조향각은 동일하다)</>],
                        ["k", "crosstrack gain. 측방 오프셋을 얼마나 공격적으로 조향해 없애는지"],
                        ["k_{soft}", <><InlineMath math="v \to 0"/>에서 arctan 항이 발산하지 않게
                            막는 작은 softening 상수</>],
                        ["v", "현재 전진 속도"],
                    ]}/>
                    <p>
                        heading 항 <InlineMath math="\psi"/>만 있으면 로봇을 경로와 평행하게 향하게만
                        하고 얼마나 떨어져 있는지는 무시한다. crosstrack 항만 있으면 경로를 쫓아가긴
                        하지만 heading이 따라붙을 때마다 매번 지나쳐 버린다. 둘을 더하면 서로 자연스럽게
                        일을 넘겨받는다. 경로에서 멀리 떨어져 있을 때는 crosstrack 항이 지배하고,
                        정렬이 끝나 그 상태를 유지하기만 하면 될 때는 heading 항이 지배한다.
                    </p>
                </>}
            />

            <h2>{t("From Bicycle to Unicycle", "자전거 모델에서 차동 구동으로")}</h2>
            <T
                en={<>
                    <p>
                        Stanley is, on paper, a steering law for a front-wheel-steered vehicle — it outputs
                        a steering angle <InlineMath math="\delta"/>, not a turn rate. The robot in this
                        demo is a differential-drive unicycle, which has no steering wheel at all. The two
                        are reconciled with one construction: introduce a virtual wheelbase{" "}
                        <InlineMath math="L"/> and treat the robot as if it were a rear-axle-referenced
                        kinematic bicycle. That model's turn rate is exactly
                    </p>
                    <BlockMath math="\omega = \frac{v \tan\delta}{L}"/>
                    <Terms items={[
                        ["\\omega", "angular velocity — the unicycle command this planner ultimately hands the simulator"],
                        ["v", <>forward speed (after the clamp-recompute step below, if the clamp on{" "}
                            <InlineMath math="\omega"/> changed it)</>],
                        ["\\delta", "steering angle from the law above"],
                        ["L", "the virtual wheelbase — a parameter, not a physical dimension of this robot"],
                    ]}/>
                    <p>
                        This is not an approximation of the unicycle model — it <em>is</em> the unicycle
                        model, just parametrized through a steering angle instead of a turn rate directly.
                        Both errors are measured where the original paper measures them: at the front axle,
                        a point <InlineMath math="L"/> ahead of the robot along its heading, not at the
                        robot's own center. What this construction does not capture is everything below the
                        kinematic level — tire slip, actuator lag, the dynamics a real steering vehicle has
                        and a unicycle never will. The figure below shows where every quantity in the
                        steering law actually sits on the robot.
                    </p>
                </>}
                ko={<>
                    <p>
                        Stanley는 원래 전륜 조향 차량을 위한 조향 법칙이다. 회전율이 아니라 조향각{" "}
                        <InlineMath math="\delta"/>를 내놓는다. 이 데모의 로봇은 조향 바퀴가 아예
                        없는 차동 구동 unicycle이다. 둘은 구성 하나로 정확히 맞아떨어진다. 가상의
                        축간거리 <InlineMath math="L"/>을 도입하고, 로봇을 후륜축 기준 기구학 자전거
                        모델로 취급하는 것이다. 그 모델의 회전율은 정확히 다음과 같다.
                    </p>
                    <BlockMath math="\omega = \frac{v \tan\delta}{L}"/>
                    <Terms items={[
                        ["\\omega", "각속도. 이 planner가 최종적으로 시뮬레이터에 넘기는 unicycle 명령"],
                        ["v", <>전진 속도 (아래 클램프-재계산 단계에서 <InlineMath math="\omega"/>가
                            클램프됐다면 그 결과가 반영된 값)</>],
                        ["\\delta", "위 조향 법칙이 내놓은 조향각"],
                        ["L", "가상 축간거리. 이 로봇의 실제 치수가 아니라 파라미터다"],
                    ]}/>
                    <p>
                        이것은 unicycle 모델을 근사한 것이 아니라, unicycle 모델 그 자체를 회전율
                        대신 조향각으로 다시 매개변수화한 것<em>이다</em>. 두 오차 모두 원 논문이
                        측정하는 지점에서 잰다. 로봇 중심이 아니라, heading 방향으로 <InlineMath math="L"/>{" "}
                        앞선 전륜축 지점이다. 이 구성이 담지 못하는 것은 기구학 아래 단계의 모든 것,
                        즉 타이어 슬립이나 액추에이터 지연처럼 실제 조향 차량은 갖지만 unicycle은
                        결코 가질 수 없는 동역학이다. 아래 그림은 조향 법칙의 각 항이 로봇 위 어디에
                        실제로 위치하는지 보여준다.
                    </p>
                </>}
            />
            <StanleyGeometryFigure/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Cost: <InlineMath math="O(1)"/> amortized per tick.</strong> Like
                            Pure Pursuit, the front-axle progress index only ever scans forward, so a full
                            run touches each path segment once rather than re-searching the whole path
                            every tick.</li>
                        <li><strong>Obstacle-blind by design.</strong> Same limitation as Pure Pursuit:
                            Stanley tracks whatever path it is handed and never queries the map for
                            obstacles. A path through a wall gets driven into the wall exactly as
                            faithfully as any other path.</li>
                        <li><strong>One gain sets the return speed.</strong> Too small a{" "}
                            <InlineMath math="k"/> and the crosstrack term barely pulls the robot back, so
                            it drifts onto the path over a long stretch; a larger <InlineMath math="k"/>{" "}
                            returns it sooner. The derivation further down shows the crosstrack error
                            decays monotonically in this kinematic model, with a rate that grows
                            with <InlineMath math="k"/>, so there is no overshoot to trade against. On a
                            real vehicle, steering lag and actuator dynamics do eventually turn too high a
                            gain into oscillation, but that lies outside the bicycle kinematics simulated
                            here.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>비용: tick당 상환 <InlineMath math="O(1)"/>.</strong> Pure Pursuit과
                            마찬가지로 전륜축 progress index는 앞으로만 훑으므로, 전체 실행을 통틀어
                            경로 구간마다 한 번씩만 닿는다. tick마다 경로 전체를 다시 훑지 않는다.</li>
                        <li><strong>설계상 장애물을 보지 못한다.</strong> Pure Pursuit과 같은 한계다.
                            Stanley는 주어진 경로를 추종할 뿐 맵에 장애물을 묻지 않는다. 벽을 관통하는
                            경로를 주면 다른 경로와 똑같이 충실하게 벽으로 걸어 들어간다.</li>
                        <li><strong>gain 하나가 복귀 속도를 정한다.</strong> <InlineMath math="k"/>가 너무
                            작으면 crosstrack 항이 로봇을 거의 끌어당기지 못해 긴 거리에 걸쳐 천천히
                            경로로 붙는다. <InlineMath math="k"/>가 클수록 더 빨리 되돌아온다. 아래 유도는
                            이 기구학 모델에서 crosstrack 오차가 단조 감소하며 그 감소율이{" "}
                            <InlineMath math="k"/>에 비례해 커짐을 보인다. 상충할 오버슈트 자체가 없다.
                            실제 차량에서는 조향 지연과 액추에이터 동역학 때문에 과대 gain이 결국 진동을
                            부르지만, 그 동역학은 여기서 시뮬레이션하는 bicycle 기구학 범위 밖이다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Speed is settled before steering, because the steering law divides by it. Everything
                    else recomputes from scratch every tick except one carried-over integer, the front-axle
                    progress index.
                </p>}
                ko={<p>
                    조향보다 속도를 먼저 정한다. 조향 법칙이 그 값으로 나누기 때문이다. progress
                    index(전륜축 기준) 정수 하나만 tick 사이에 유지되고, 나머지는 매 tick 처음부터
                    다시 계산한다.
                </p>}
            />
            <Pseudocode code={`v ← v_max * min(1, remaining_to_goal / slow_radius)              # 1
front ← (x + L cos(theta), y + L sin(theta))                       # 2
i ← advance_forward(path, i, front)                                 # 3
psi ← wrap(theta_path(i) - theta)                                    # 4
foot ← closest_point_on_segment(front, path[i], path[i+1])          # 5
e ← cross(tangent(i), front - foot)                                  # 6
delta ← psi - atan(k_gain * e / (k_soft + v))                        # 7
delta ← clamp(delta, -delta_max, delta_max)                          # 8
omega ← clamp(v * tan(delta) / L, -omega_max, omega_max)             # 9
if omega was clamped:                                                 # 10
    v ← omega * L / tan(delta)                                        # 11
return (v, omega)                                                     # 12`}/>
            <T
                en={<ol>
                    <li>Ramp forward speed down inside a radius of the goal, exactly as Pure Pursuit does.
                        This has to happen first because step 7 divides by it.</li>
                    <li>Project the front axle out from the robot's own pose along its heading, a distance{" "}
                        <InlineMath math="L"/> — every error below is measured from here, not from
                        <InlineMath math="(x, y)"/>.</li>
                    <li>Advance the progress index forward-only from its last value, scanning for the path
                        segment nearest the front axle — the same monotonic rule Pure Pursuit uses, and for
                        the same reason: a self-crossing path must never snap tracking back to an earlier
                        crossing.</li>
                    <li>Measure the heading error between the path tangent at that segment and the robot's
                        current heading.</li>
                    <li>Project the front axle onto the segment to get the foot point — the closest point
                        on the path.</li>
                    <li>The crosstrack error is the signed perpendicular distance from the front axle to
                        that foot point.</li>
                    <li>Combine both errors into one steering angle, softened by <InlineMath math="k_{soft}"/>{" "}
                        so the crosstrack term stays finite as <InlineMath math="v \to 0"/>.</li>
                    <li>Clamp the steering angle to what the (virtual) front wheel can physically reach.</li>
                    <li>Convert to a turn rate through the bicycle–unicycle equivalence and clamp that to
                        the robot's real angular speed limit.</li>
                    <li>If that second clamp actually changed <InlineMath math="\omega"/>, recompute{" "}
                        <InlineMath math="v"/> from the clamped <InlineMath math="\omega"/> and the{" "}
                        <em>already-clamped</em> <InlineMath math="\delta"/> — the same clamp-recompute
                        pattern as Pure Pursuit, and just as easy to forget: skip it and the executed
                        command quietly traces a gentler arc than <InlineMath math="\delta"/> called for.</li>
                    <li>Hand back the command for the closed-loop simulator to integrate.</li>
                </ol>}
                ko={<ol>
                    <li>Pure Pursuit과 똑같이 goal 반경 안에서 전진 속도를 선형으로 줄인다. 7번
                        단계가 이 값으로 나누므로 먼저 정해야 한다.</li>
                    <li>로봇 pose에서 heading 방향으로 <InlineMath math="L"/>만큼 떨어진 전륜축
                        지점을 구한다. 아래 모든 오차는 <InlineMath math="(x, y)"/>가 아니라 여기서
                        잰다.</li>
                    <li>progress index를 마지막 값에서부터 앞으로만 훑어, 전륜축에 가장 가까운 경로
                        구간으로 갱신한다. Pure Pursuit과 같은 단조 규칙이고 이유도 같다. 자기 자신과
                        교차하는 경로에서 추종이 뒤쪽의 더 가까운 교점으로 되돌아가면 안 된다.</li>
                    <li>그 구간의 경로 접선과 현재 heading 사이 오차를 잰다.</li>
                    <li>전륜축을 그 구간에 정사영해 foot point, 즉 경로 위 최근접점을 구한다.</li>
                    <li>crosstrack 오차는 전륜축에서 foot point까지의 부호 있는 수직 거리다.</li>
                    <li>두 오차를 조향각 하나로 합친다. <InlineMath math="k_{soft}"/>가 crosstrack
                        항을 <InlineMath math="v \to 0"/>에서도 유한하게 유지한다.</li>
                    <li>(가상의) 전륜이 물리적으로 낼 수 있는 값으로 조향각을 클램프한다.</li>
                    <li>자전거-unicycle 등가 관계로 회전율로 바꾸고, 로봇의 실제 각속도 한계로
                        다시 클램프한다.</li>
                    <li>두 번째 클램프가 실제로 <InlineMath math="\omega"/>를 바꿨다면, 클램프된{" "}
                        <InlineMath math="\omega"/>와 <em>이미 클램프된</em> <InlineMath math="\delta"/>로{" "}
                        <InlineMath math="v"/>를 다시 계산한다. Pure Pursuit과 같은 클램프-재계산
                        패턴이고 똑같이 놓치기 쉽다. 건너뛰면 실행되는 명령이 <InlineMath math="\delta"/>가
                        요구한 것보다 조용히 더 완만한 원호를 그린다.</li>
                    <li>명령을 반환해 폐루프 시뮬레이터가 적분하게 한다.</li>
                </ol>}
            />

            <h2>{t("Why the Crosstrack Error Converges", "Crosstrack 오차의 수렴 증명")}</h2>
            <Proof title={t(
                "Derivation (local exponential convergence of e)",
                "유도 (e의 국소 지수 수렴)",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Assumption.</strong> The heading term has already driven{" "}
                            <InlineMath math="\psi \to 0"/> — the robot body is aligned with the path
                            tangent, so the commanded steering angle comes from the crosstrack term alone:
                        </p>
                        <BlockMath math="\delta = \psi - \arctan\!\left(\frac{k e}{k_{soft} + v}\right) \;\xrightarrow{\ \psi \to 0\ }\; \delta = -\arctan\!\left(\frac{k e}{k_{soft} + v}\right)"/>
                        <Terms items={[
                            ["\\delta", "steering angle, this repository's sign convention (e left-positive)"],
                            ["\\psi", "heading error — assumed already zeroed"],
                            ["e", "crosstrack error (front-axle, left-positive)"],
                            ["k,\\ k_{soft},\\ v", "gain, softening constant, forward speed — as defined above"],
                        ]}/>
                        <p>
                            Under the bicycle–unicycle equivalence, the front axle's velocity component
                            perpendicular to the path — the rate of change of <InlineMath math="e"/> — is{" "}
                            <InlineMath math="v"/> times the sine of the vehicle's heading relative to the
                            path, which under the assumption above equals <InlineMath math="\delta"/>:
                        </p>
                        <BlockMath math="\dot e = v \sin\delta = -v \sin\!\left(\arctan\!\left(\frac{k e}{k_{soft} + v}\right)\right)"/>
                        <Terms items={[
                            ["\\dot e", "rate of change of the crosstrack error"],
                            ["v,\\ \\delta", "forward speed and steering angle, as above"],
                            ["k,\\ k_{soft},\\ e", "as above"],
                        ]}/>
                        <p>
                            <InlineMath math="\sin(\arctan z) = z/\sqrt{1+z^2}"/> carries the sign of{" "}
                            <InlineMath math="z"/>, so <InlineMath math="\dot e"/> always carries the sign
                            opposite <InlineMath math="e"/>:
                        </p>
                        <BlockMath math="\operatorname{sign}(\dot e) = -\operatorname{sign}(e) \quad\Longrightarrow\quad \frac{d}{dt}|e| \le 0 \ \text{ for all } e"/>
                        <Terms items={[
                            ["\\operatorname{sign}(\\cdot)", "sign function"],
                            ["|e|", "magnitude of the crosstrack error"],
                        ]}/>
                        <p>
                            so <InlineMath math="|e|"/> never increases. Near <InlineMath math="e = 0"/>{" "}
                            the arctan is small and <InlineMath math="\sin(\arctan z) \approx z"/>, so the
                            dynamics linearize to a first-order decay:
                        </p>
                        <BlockMath math="\dot e \approx -\frac{vk}{k_{soft}+v}\, e \quad\Longrightarrow\quad e(t) \approx e(0)\, \exp\!\left(-\frac{vk}{k_{soft}+v}\, t\right)"/>
                        <Terms items={[
                            ["e(t),\\ e(0)", "crosstrack error at time t and at the start of this local approximation"],
                            ["\\frac{vk}{k_{soft}+v}", "the local convergence rate — larger gain k means faster convergence, but also a stronger overcorrection once e is no longer small (the high-gain oscillation in the demo below)"],
                        ]}/>
                        <p>
                            <strong>Conclusion.</strong> With the heading error already zeroed,{" "}
                            <InlineMath math="|e|"/> decreases monotonically and converges to zero
                            exponentially near the origin — the local stability result of Hoffmann et al.
                            (2007). The assumption itself only holds approximately once{" "}
                            <InlineMath math="\psi"/> is small; this is a statement about the crosstrack
                            term's own dynamics, not a global convergence guarantee for the full two-term
                            law.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> heading 항이 이미 <InlineMath math="\psi \to 0"/>을
                            만들어, 로봇 몸체가 경로 접선과 정렬돼 있다고 하자. 그러면 명령되는
                            조향각은 crosstrack 항만으로 정해진다.
                        </p>
                        <BlockMath math="\delta = \psi - \arctan\!\left(\frac{k e}{k_{soft} + v}\right) \;\xrightarrow{\ \psi \to 0\ }\; \delta = -\arctan\!\left(\frac{k e}{k_{soft} + v}\right)"/>
                        <Terms items={[
                            ["\\delta", "조향각. 이 저장소의 부호 규약(e 좌측 양수)"],
                            ["\\psi", "heading 오차. 이미 0이 됐다고 가정"],
                            ["e", "crosstrack 오차 (전륜축, 좌측 양수)"],
                            ["k,\\ k_{soft},\\ v", "gain, softening 상수, 전진 속도. 위와 동일"],
                        ]}/>
                        <p>
                            자전거-unicycle 등가 관계에서, 전륜축이 경로에 수직으로 움직이는
                            속도 성분(<InlineMath math="e"/>의 변화율)은 <InlineMath math="v"/> 곱하기
                            차량 heading과 경로 사이 각의 sine이고, 위 가정 아래에서 그 각은{" "}
                            <InlineMath math="\delta"/>와 같다.
                        </p>
                        <BlockMath math="\dot e = v \sin\delta = -v \sin\!\left(\arctan\!\left(\frac{k e}{k_{soft} + v}\right)\right)"/>
                        <Terms items={[
                            ["\\dot e", "crosstrack 오차의 변화율"],
                            ["v,\\ \\delta", "전진 속도와 조향각. 위와 동일"],
                            ["k,\\ k_{soft},\\ e", "위와 동일"],
                        ]}/>
                        <p>
                            <InlineMath math="\sin(\arctan z) = z/\sqrt{1+z^2}"/>는{" "}
                            <InlineMath math="z"/>와 부호가 같으므로, <InlineMath math="\dot e"/>는
                            항상 <InlineMath math="e"/>와 반대 부호다.
                        </p>
                        <BlockMath math="\operatorname{sign}(\dot e) = -\operatorname{sign}(e) \quad\Longrightarrow\quad \frac{d}{dt}|e| \le 0 \ \text{ (모든 } e \text{에 대해)}"/>
                        <Terms items={[
                            ["\\operatorname{sign}(\\cdot)", "부호 함수"],
                            ["|e|", "crosstrack 오차의 크기"],
                        ]}/>
                        <p>
                            따라서 <InlineMath math="|e|"/>는 절대 늘지 않는다. <InlineMath math="e = 0"/> 근방에서는
                            arctan 값이 작아 <InlineMath math="\sin(\arctan z) \approx z"/>이므로 동역학이
                            1차 감쇠로 선형화된다.
                        </p>
                        <BlockMath math="\dot e \approx -\frac{vk}{k_{soft}+v}\, e \quad\Longrightarrow\quad e(t) \approx e(0)\, \exp\!\left(-\frac{vk}{k_{soft}+v}\, t\right)"/>
                        <Terms items={[
                            ["e(t),\\ e(0)", "시각 t에서의 crosstrack 오차와, 이 국소 근사가 시작되는 시점의 값"],
                            ["\\frac{vk}{k_{soft}+v}", "국소 수렴률. gain k가 클수록 더 빨리 수렴하지만, e가 더 이상 작지 않을 때는 그만큼 더 크게 과보정한다(아래 데모의 과대 gain 진동)"],
                        ]}/>
                        <p>
                            <strong>결론.</strong> heading 오차가 이미 0이 된 상태에서 <InlineMath math="|e|"/>는
                            단조 감소하며 원점 근방에서 지수적으로 0에 수렴한다. Hoffmann et al.
                            (2007)의 국소 안정성 결과다. 이 가정 자체는 <InlineMath math="\psi"/>가
                            작을 때만 근사적으로 성립한다. 이는 crosstrack 항 자체의 동역학에 대한
                            진술이지, 두 항을 합친 전체 법칙의 전역 수렴을 보장하는 것은 아니다.
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs Stanley live in your browser. The "weak k_gain" and "strong
                    k_gain" presets both start the robot 1.5 m off the S-curve: with a weak gain it drifts
                    back onto the path slowly, while a strong gain snaps it back within the first metre —
                    the faster monotone return the derivation above predicts. The sharp-corner preset then
                    shows both error terms steering together through a string of right angles.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 Stanley를 라이브로 실행한다. "약한 k_gain"과 "강한 k_gain"
                    프리셋은 모두 로봇을 S-곡선에서 1.5 m 벗어난 위치에서 시작한다. gain이 약하면 긴 거리에
                    걸쳐 천천히 경로로 붙고, 강하면 첫 1 m 안에 곧바로 되돌아온다. 위 유도가 예측한 더 빠른
                    단조 복귀다. 급코너 프리셋은 두 오차 항이 연속된 직각 코너를 함께 조향하는 모습을
                    보여준다.
                </p>}
            />
            <StanleySandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below follows the algorithm above line for line: settle the speed,
                    project the front axle, advance the shared progress-index utility, combine the two
                    errors, then convert through the bicycle–unicycle equivalence with the same
                    clamp-recompute step Pure Pursuit uses. The code is the actual repository source, not
                    an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 한 줄씩 그대로 따른다. 속도를 정하고, 전륜축을
                    투영하고, 공유 progress-index 유틸로 진행 구간을 갱신하고, 두 오차를 합친 뒤,
                    Pure Pursuit과 같은 클램프-재계산 단계로 자전거-unicycle 등가 관계를 거쳐
                    변환한다. 아래 코드는 발췌가 아니라 저장소의 실제 소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/tracking/stanley.py",
                                code: stanleyPy,
                                href: `${REPO}/python/navigation/local_planning/tracking/stanley.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/tracking/stanley.hpp",
                                code: stanleyHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/tracking/stanley.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/tracking/stanley.cpp",
                                code: stanleyCpp,
                                href: `${REPO}/cpp/src/local_planning/tracking/stanley.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The two-error steering law and the bicycle–unicycle conversion, embedded from the repository sources",
                    "두 오차 조향 법칙과 자전거-unicycle 변환. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. Thrun, M. Montemerlo, H. Dahlkamp, D. Stavens, A. Aron, J. Diebel, P. Fong,
                    J. Gale, M. Halpenny, G. Hoffmann, K. Lau, C. Oakley, M. Palatucci, V. Pratt,
                    P. Stang, S. Strohband, C. Dupont, L.-E. Jendrossek, C. Koelen, C. Markey,
                    C. Rummel, J. van Niekerk, E. Jensen, P. Alessandrini, G. Bradski, B. Davies,
                    S. Ettinger, A. Kaehler, A. Nefian, P. Mahoney,{" "}
                    <a href="https://doi.org/10.1002/rob.20147" target="_blank" rel="noopener noreferrer">
                        <em>Stanley: The Robot that Won the DARPA Grand Challenge</em>
                    </a>, Journal of Field Robotics, vol. 23, no. 9, pp. 661–692, 2006.
                </li>
                <li>
                    G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun,{" "}
                    <a href="https://doi.org/10.1109/ACC.2007.4282788" target="_blank" rel="noopener noreferrer">
                        <em>Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller
                        Design, Experimental Validation and Racing</em>
                    </a>, American Control Conference, 2007.
                </li>
            </ol>
        </>
    )
}

export default Stanley
