import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import PurePursuitSandbox from "../../../components/panels/local/pure_pursuit/PurePursuitSandbox";
import LookaheadGeometryFigure from "../../../components/panels/local/pure_pursuit/LookaheadGeometryFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import purePursuitPy from "../../../../../python/navigation/local_planning/tracking/pure_pursuit.py?raw";
import geometryPy from "../../../../../python/navigation/local_planning/_geometry.py?raw";
import purePursuitCpp from "../../../../../cpp/src/local_planning/tracking/pure_pursuit.cpp?raw";
import geometryHpp from "../../../../../cpp/include/navigation/local_planning/geometry.hpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도/증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식적 전개는 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const PurePursuit = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Pure Pursuit does one thing: it picks a single point some fixed distance ahead on
                    a given path and steers along the one circular arc that reaches it. No lateral-error
                    term, no separate heading-error term, no gains to tune between them — the geometry of
                    "one arc through two constraints" does all the work. R. Craig Coulter described the
                    method in a 1992 Carnegie Mellon Robotics Institute report after years of use on the
                    Institute's outdoor vehicles (Terragator, then NavLab), and the same construction still
                    drives path-tracking stacks on everything from warehouse robots to full-size autonomous
                    cars.
                </p>}
                ko={<p>
                    Pure Pursuit는 하나만 한다. 주어진 경로 위에서 일정 거리 앞의 점 하나를 고르고,
                    그 점에 도달하는 단 하나의 원호를 따라 조향한다. 횡방향 오차 항도, 별도의 heading
                    오차 항도, 그 사이를 조율할 gain도 없다. "두 조건을 만족하는 원호는 하나뿐"이라는
                    기하가 일을 전부 해낸다. R. Craig Coulter가 1992년 Carnegie Mellon 로보틱스
                    연구소 리포트에서 이 방법을 정리했는데, 그전부터 이미 연구소의 야외 주행
                    차량(Terragator, 이후 NavLab)에서 여러 해 쓰이던 방법이었다. 같은 구성이 지금도
                    창고 로봇부터 실제 크기의 자율주행차까지 경로 추종 스택의 바탕을 이룬다.
                </p>}
            />

            <h2>{t("Chasing a Point on the Path", "경로 위의 점을 쫓는다")}</h2>
            <T
                en={<>
                    <p>
                        Fix a lookahead distance <InlineMath math="L_d"/>. At every control step, draw a
                        circle of radius <InlineMath math="L_d"/> centered on the robot and find where it
                        crosses the reference path:
                    </p>
                    <BlockMath math="\{\, p \in \text{path} \;:\; \lVert p - (x, y) \rVert = L_d \,\}"/>
                    <Terms items={[
                        ["x,\\ y", "robot's current position (world coordinates)"],
                        ["L_d", "lookahead distance — the one tuning knob this algorithm exposes"],
                        ["p", "a point on the reference path"],
                    ]}/>
                    <p>
                        A path can cross that circle more than once (think of a path that loops back near
                        itself), so the rule is: walk forward along the path from wherever the robot left
                        off last tick, and take the <em>first</em> crossing found — never a crossing behind
                        where the robot already is. That crossing is the <strong>lookahead point</strong>,
                        and it is the only piece of the path pure pursuit ever looks at on a given tick.
                        Everything downstream — the steering command, the curvature, the whole "tracking"
                        behavior — is just: aim a single arc at that one point.
                    </p>
                </>}
                ko={<>
                    <p>
                        lookahead 거리 <InlineMath math="L_d"/>를 하나 고정해 둔다. 매 제어 스텝마다
                        로봇 위치를 중심으로 반지름 <InlineMath math="L_d"/>인 원을 그리고, 그 원이
                        참조 경로와 만나는 점을 찾는다:
                    </p>
                    <BlockMath math="\{\, p \in \text{path} \;:\; \lVert p - (x, y) \rVert = L_d \,\}"/>
                    <Terms items={[
                        ["x,\\ y", "로봇의 현재 위치 (world 좌표)"],
                        ["L_d", "lookahead 거리. 이 알고리즘이 노출하는 유일한 튜닝 값"],
                        ["p", "참조 경로 위의 한 점"],
                    ]}/>
                    <p>
                        경로가 자기 자신 근처로 되돌아오는 형태면 이 원과 두 번 이상 만날 수 있다.
                        그래서 규칙은 이렇다. 직전 tick에서 멈춘 지점부터 경로를 따라 앞으로 걸어가며
                        <em>처음</em> 만나는 교점을 취한다. 로봇이 이미 지나온 뒤쪽의 교점은 절대
                        고르지 않는다. 이 교점이 <strong>lookahead 점</strong>이고, pure pursuit이 한
                        tick에서 들여다보는 경로 정보는 이 점 하나뿐이다. 조향 명령도, 곡률도,
                        "추종"이라 부르는 동작 전체도 결국 그 점 하나를 향해 원호 하나를 겨누는
                        일이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Purely reactive, not a planner.</strong> Pure Pursuit never searches
                            for a path and never perceives obstacles — it only ever tracks a path handed
                            to it. Feed it a path through a wall and it will drive into the wall.</li>
                        <li><strong>Corner-cutting is systematic, not a bug.</strong> On a curve, the
                            lookahead point sits ahead of the robot on the inside of the turn, so the
                            commanded arc consistently swings a little tighter than the path itself. The
                            error stays on the order of <InlineMath math="L_d"/>: raising the lookahead
                            distance trades tracking accuracy for smoother, less oscillatory commands, and
                            lowering it does the reverse — small <InlineMath math="L_d"/> hugs the path
                            closely but reacts to every kink and every bit of sensor noise, which is what
                            makes very small settings feel jittery in practice.</li>
                        <li><strong>No formal convergence guarantee.</strong> Unlike a planner's
                            optimality proof, pure pursuit's good behavior is empirical and geometric
                            (Coulter, 1992, §4–5), not a theorem — which is exactly why the derivation
                            below only claims to construct <em>an</em> arc through the lookahead point,
                            not to prove anything about long-run tracking error.</li>
                        <li><strong>Cost: effectively <InlineMath math="O(1)"/> per tick.</strong> The
                            search for the lookahead point resumes from wherever the previous tick left
                            off and only ever moves forward along the path, so across an entire run the
                            path is scanned once, not once per tick.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>순수 반응형이지 planner가 아니다.</strong> Pure Pursuit은 경로를
                            탐색하지도, 장애물을 인지하지도 않는다. 주어진 경로를 추종할 뿐이다.
                            벽을 관통하는 경로를 주면 그대로 벽으로 걸어 들어간다.</li>
                        <li><strong>corner-cutting은 버그가 아니라 구조적 현상이다.</strong> 커브에서는
                            lookahead 점이 회전 안쪽으로 로봇보다 앞서 있어, 명령되는 원호가 실제
                            경로보다 항상 조금 더 조여져 돈다. 이 오차는 대략 <InlineMath math="L_d"/>{" "}
                            크기 수준에 머문다. lookahead 거리를 늘리면 추종 정확도를 내주고 더
                            매끄럽고 덜 진동하는 명령을 얻고, 줄이면 반대다. 작은 <InlineMath math="L_d"/>는
                            경로에 바짝 붙지만 경로의 작은 꺾임과 센서 잡음 하나하나에 반응해, 아주
                            작게 잡으면 실전에서 떨리는 느낌을 준다.</li>
                        <li><strong>형식적 수렴 보장은 없다.</strong> planner의 최적성 증명과 달리
                            pure pursuit의 좋은 동작은 정리가 아니라 경험·기하에 기반한다(Coulter,
                            1992, §4–5). 그래서 아래 유도도 lookahead 점을 지나는 원호 <em>하나</em>를
                            구성한다는 것만 보이지, 장기 추종 오차에 대해서는 아무것도 증명하지
                            않는다.</li>
                        <li><strong>비용: tick당 사실상 <InlineMath math="O(1)"/>.</strong> lookahead
                            점 탐색은 직전 tick이 멈춘 지점에서 재개해 경로를 따라 앞으로만
                            움직이므로, 전체 실행을 통틀어 경로를 한 번만 훑는다. tick마다 한 번씩이
                            아니다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The only state carried between ticks is a <strong>progress index</strong>: which
                    segment of the reference path the search should resume from. Everything else is
                    recomputed from scratch every tick.
                </p>}
                ko={<p>
                    tick 사이에 유지되는 상태는 <strong>progress index</strong> 하나뿐이다. 탐색을
                    재개할 참조 경로 구간을 가리킨다. 그 외에는 매 tick 전부 처음부터 다시 계산한다.
                </p>}
            />
            <Pseudocode code={`i ← advance_forward(path, i, robot_xy)                       # 1
target ← lookahead_point(path, i, robot_xy, L_d)              # 2
alpha ← wrap(atan2(target_y - y, target_x - x) - theta)       # 3
kappa ← 2 sin(alpha) / L_d                                     # 4
v ← v_max * min(1, remaining_to_goal / slow_radius)           # 5
omega ← clamp(kappa * v, -omega_max, omega_max)                # 6
if omega was clamped:                                          # 7
    v ← omega / kappa
return (v, omega)                                               # 8`}/>
            <T
                en={<ol>
                    <li>Advance the progress index by scanning forward from its last value for the
                        path segment closest to the robot, never backward. On a self-crossing path this
                        is what keeps the lookahead point from snapping back to an earlier, geometrically
                        nearer crossing — the robot always chases forward along the path it has already
                        committed to.</li>
                    <li>Find the lookahead point as in the previous section: the forward-most crossing
                        of the <InlineMath math="L_d"/>-circle with the path from the progress index
                        onward. If the remaining path is shorter than <InlineMath math="L_d"/>, no
                        crossing exists — aim at the path's last point (the goal) instead.</li>
                    <li>Measure the bearing of the lookahead point relative to the current heading,
                        wrapped to <InlineMath math="(-\pi, \pi]"/>. Positive <InlineMath math="\alpha"/>{" "}
                        means the point sits to the left.</li>
                    <li>Convert that bearing into a curvature — derived below.</li>
                    <li>Ramp the forward speed down linearly inside a radius of the goal, so the robot
                        does not overshoot at full speed and then have to turn back.</li>
                    <li>Turn rate is speed times curvature; clamp it to what the robot can physically
                        do.</li>
                    <li>If the clamp actually changed <InlineMath math="\omega"/>, recompute{" "}
                        <InlineMath math="v"/> from the clamped <InlineMath math="\omega"/> and the
                        <em>original</em> <InlineMath math="\kappa"/>. Skipping this is a common bug:
                        clamping <InlineMath math="\omega"/> alone leaves <InlineMath math="v"/> at its
                        old value, so the executed <InlineMath math="(v, \omega)"/> traces a
                        gentler arc than <InlineMath math="\kappa"/> called for — understeering relative
                        to the geometry just derived.</li>
                    <li>Hand back the command; the closed-loop simulator integrates it and calls in
                        again next tick.</li>
                </ol>}
                ko={<ol>
                    <li>progress index를 마지막 값에서부터 앞으로만 훑어, 로봇에 가장 가까운 경로
                        구간으로 갱신한다. 절대 뒤로 가지 않는다. 자기 자신과 교차하는 경로에서
                        lookahead 점이 기하적으로 더 가까운 뒤쪽 교점으로 되돌아가지 않게 하는 것이
                        바로 이 규칙이다 — 로봇은 이미 지나온 경로를 따라 항상 앞으로만 쫓는다.</li>
                    <li>앞 절과 같은 방식으로 lookahead 점을 찾는다. progress index 이후 구간에서{" "}
                        <InlineMath math="L_d"/>원과의 교점 중 가장 앞선 것. 남은 경로가{" "}
                        <InlineMath math="L_d"/>보다 짧으면 교점이 없다 — 대신 경로의 마지막
                        점(goal)을 겨눈다.</li>
                    <li>현재 heading 대비 lookahead 점의 방위를 <InlineMath math="(-\pi, \pi]"/>로
                        wrap해 측정한다. <InlineMath math="\alpha"/>가 양수면 점이 왼쪽에 있다는
                        뜻이다.</li>
                    <li>그 방위를 곡률로 바꾼다 — 유도는 아래에.</li>
                    <li>goal 반경 안에서는 전진 속도를 선형으로 줄여, 로봇이 전속력으로 지나쳐
                        되돌아오는 일이 없게 한다.</li>
                    <li>회전율은 속도 곱하기 곡률이고, 로봇이 물리적으로 낼 수 있는 값으로
                        클램프한다.</li>
                    <li>클램프가 실제로 <InlineMath math="\omega"/>를 바꿨다면, 클램프된{" "}
                        <InlineMath math="\omega"/>와 <em>원래의</em> <InlineMath math="\kappa"/>로{" "}
                        <InlineMath math="v"/>를 다시 계산한다. 흔히 놓치는 지점이다.{" "}
                        <InlineMath math="\omega"/>만 클램프하고 <InlineMath math="v"/>를 옛 값 그대로
                        두면, 실행되는 <InlineMath math="(v, \omega)"/>가 방금 유도한{" "}
                        <InlineMath math="\kappa"/>보다 더 완만한 원호를 그린다 — 기하가 요구한
                        것보다 덜 도는(understeer) 결과가 된다.</li>
                    <li>명령을 반환한다. 폐루프 시뮬레이터가 이를 적분하고 다음 tick에 다시
                        불러온다.</li>
                </ol>}
            />

            <h2>{t("The Lookahead Circle", "Lookahead 원의 기하")}</h2>
            <T
                en={<p>
                    Where does <InlineMath math="\kappa = 2\sin\alpha / L_d"/> actually come from? It
                    falls out of a single geometric fact: among all circles that pass through the robot's
                    position while staying tangent to its current heading there, exactly one of them also
                    passes through the lookahead point. That circle <em>is</em> the arc pure pursuit
                    drives — the derivation below constructs it.
                </p>}
                ko={<p>
                    <InlineMath math="\kappa = 2\sin\alpha / L_d"/>는 대체 어디서 나온 식일까?
                    기하 사실 하나로 정리된다. 로봇의 위치를 지나면서 그 자리에서 현재 heading에
                    접하는 모든 원 중, lookahead 점까지 지나는 원은 정확히 하나뿐이다. 그 원이 바로
                    pure pursuit이 그리는 원호<em>다</em>. 아래 유도가 그 원을 구성한다.
                </p>}
            />
            <LookaheadGeometryFigure/>
            <Proof title={t("Derivation (curvature of the pursuit arc)", "유도 (추종 원호의 곡률)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Let the robot sit at the origin heading along the
                            tangent direction, and let the lookahead point be a chord of length{" "}
                            <InlineMath math="L_d"/> away, at bearing <InlineMath math="\alpha"/> from
                            that tangent. Consider the circle of radius <InlineMath math="R"/> through
                            the robot that is tangent to the heading there.
                        </p>
                        <p>
                            The <strong>tangent–chord angle theorem</strong> says the angle between a
                            tangent and a chord drawn from the point of tangency equals half the central
                            angle the chord subtends — the same relationship the inscribed angle theorem
                            gives from any point on the far arc. Applying it to the right triangle formed
                            by the chord, the tangent, and the diameter through the robot gives the chord
                            length directly in terms of <InlineMath math="R"/> and the tangent–chord
                            angle:
                        </p>
                        <BlockMath math="L_d = 2R \sin\alpha"/>
                        <Terms items={[
                            ["L_d", <>chord length — this is exactly the lookahead distance, since the
                                lookahead point lies on the robot-centered circle of radius{" "}
                                <InlineMath math="L_d"/> by construction</>],
                            ["R", "radius of the circle through the robot, tangent to its heading there"],
                            ["\\alpha", "tangent–chord angle: the bearing of the lookahead point relative to the current heading"],
                        ]}/>
                        <p>
                            <strong>Uniqueness.</strong> A circle is fixed by a point plus the tangent
                            direction there, together with one further point it must pass through. The
                            tangent direction fixes the center to lie somewhere on the line through the
                            robot perpendicular to the heading; the lookahead point then picks out exactly
                            one location on that line (for <InlineMath math="\alpha \ne 0"/>), and{" "}
                            <InlineMath math="R"/> above is its distance from the robot. So{" "}
                            <InlineMath math="R"/> is not just <em>a</em> radius consistent with the
                            construction — it is the radius of the one circle that satisfies both
                            constraints simultaneously.
                        </p>
                        <p>
                            Curvature is the reciprocal of that radius:
                        </p>
                        <BlockMath math="\kappa = \frac{1}{R} = \frac{2\sin\alpha}{L_d}"/>
                        <Terms items={[
                            ["\\kappa", "signed curvature of the pursuit arc — positive turns left, matching the sign of \\alpha"],
                            ["R", "radius from the chord relation above"],
                            ["\\alpha,\\ L_d", "as above: tangent–chord angle and lookahead distance"],
                        ]}/>
                        <p>
                            This is exactly the <InlineMath math="\kappa"/> the algorithm computes each
                            tick — commanding it drives the robot along the unique arc that starts on the
                            current heading and reaches the lookahead point.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 로봇이 원점에서 접선 방향을 향해 있고, lookahead
                            점은 그 접선으로부터 방위 <InlineMath math="\alpha"/>, 길이{" "}
                            <InlineMath math="L_d"/>인 현(chord) 끝에 있다고 하자. 로봇을 지나면서
                            그 자리에서 heading에 접하는 반지름 <InlineMath math="R"/>인 원을 생각한다.
                        </p>
                        <p>
                            <strong>접선-현 각 정리(tangent–chord angle theorem)</strong>에 따르면
                            접점에서 그은 접선과 현 사이의 각은 그 현이 만드는 중심각의 절반과
                            같다 — 먼 쪽 호 위 어느 점에서 보든 원주각 정리가 주는 값과 동일하다.
                            이를 현·접선·로봇을 지나는 지름이 만드는 직각삼각형에 적용하면 현의
                            길이가 <InlineMath math="R"/>과 접선-현 각으로 바로 나온다:
                        </p>
                        <BlockMath math="L_d = 2R \sin\alpha"/>
                        <Terms items={[
                            ["L_d", <>현의 길이. lookahead 점은 애초에 로봇 중심 반지름{" "}
                                <InlineMath math="L_d"/> 원 위에 있으므로 이 길이가 곧 lookahead
                                거리 자체다</>],
                            ["R", "로봇을 지나며 그 자리에서 heading에 접하는 원의 반지름"],
                            ["\\alpha", "접선-현 각. 현재 heading 대비 lookahead 점의 방위"],
                        ]}/>
                        <p>
                            <strong>유일성.</strong> 원은 한 점과 그 점에서의 접선 방향, 그리고 그
                            원이 지나야 할 점 하나가 더 있으면 하나로 정해진다. 접선 방향이 정해지면
                            중심은 로봇을 지나고 heading에 수직인 직선 위 어딘가로 제한되고,
                            lookahead 점이 (<InlineMath math="\alpha \ne 0"/>일 때) 그 직선 위 위치를
                            정확히 하나로 골라낸다. 위의 <InlineMath math="R"/>이 바로 로봇에서 그
                            위치까지의 거리다. 즉 <InlineMath math="R"/>은 이 구성과 맞아떨어지는{" "}
                            <em>어떤</em> 반지름이 아니라, 두 조건을 동시에 만족하는 유일한 원의
                            반지름이다.
                        </p>
                        <p>
                            곡률은 그 반지름의 역수다:
                        </p>
                        <BlockMath math="\kappa = \frac{1}{R} = \frac{2\sin\alpha}{L_d}"/>
                        <Terms items={[
                            ["\\kappa", "추종 원호의 부호 있는 곡률. 양수면 왼쪽으로 돈다 — \\alpha의 부호와 일치"],
                            ["R", "위 현 관계식에서 나온 반지름"],
                            ["\\alpha,\\ L_d", "위와 동일: 접선-현 각과 lookahead 거리"],
                        ]}/>
                        <p>
                            이것이 바로 알고리즘이 매 tick 계산하는 <InlineMath math="\kappa"/>다.
                            이를 명령하면 로봇은 현재 heading에서 시작해 lookahead 점에 닿는 유일한
                            원호를 따라간다.
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs Pure Pursuit live in your browser along two fixed reference
                    paths. Drag the robot off the path and watch the lookahead circle pull it back onto
                    the S-curve; switch to the sharp-turn preset and raise <InlineMath math="L_d"/> to
                    watch the arc cut each corner progressively wider.
                </p>}
                ko={<p>
                    아래 sandbox는 두 고정 참조 경로를 따라 브라우저에서 Pure Pursuit을 라이브로
                    실행한다. 로봇을 경로 밖으로 끌어 lookahead 원이 S-곡선 위로 다시 끌어당기는
                    모습을 보고, 급커브 프리셋으로 바꿔 <InlineMath math="L_d"/>를 올리며 원호가 각
                    코너를 점점 더 크게 잘라가는 모습을 보라.
                </p>}
            />
            <PurePursuitSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below is a near-literal transcription of the algorithm above: one
                    forward-only scan for the progress index, one circle–segment intersection for the
                    lookahead point, and the closed-form curvature. The code is the actual repository
                    source, not an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 거의 그대로 옮긴 것이다. progress index를 위한 전진
                    전용 스캔 한 번, lookahead 점을 위한 원-선분 교차 한 번, 그리고 closed-form
                    곡률. 아래 코드는 발췌가 아니라 저장소의 실제 소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/tracking/pure_pursuit.py",
                                code: purePursuitPy,
                                href: `${REPO}/python/navigation/local_planning/tracking/pure_pursuit.py`,
                            },
                            {
                                name: "python/navigation/local_planning/_geometry.py",
                                code: geometryPy,
                                href: `${REPO}/python/navigation/local_planning/_geometry.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/src/local_planning/tracking/pure_pursuit.cpp",
                                code: purePursuitCpp,
                                href: `${REPO}/cpp/src/local_planning/tracking/pure_pursuit.cpp`,
                            },
                            {
                                name: "cpp/include/navigation/local_planning/geometry.hpp",
                                code: geometryHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/geometry.hpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The pursuit-point search and the closed-form curvature, embedded from the repository sources",
                    "lookahead 점 탐색과 closed-form 곡률. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    R. C. Coulter,{" "}
                    <a href="https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Implementation of the Pure Pursuit Path Tracking Algorithm</em>
                    </a>,
                    Carnegie Mellon University Robotics Institute, Technical Report CMU-RI-TR-92-01, 1992.
                </li>
            </ol>
        </>
    )
}

export default PurePursuit
