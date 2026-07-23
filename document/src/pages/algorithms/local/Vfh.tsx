import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import VfhSandbox from "../../../components/panels/local/vfh/VfhSandbox";
import VfhHistogramFigure from "../../../components/panels/local/vfh/VfhHistogramFigure";
import vfhPy from "../../../../../python/navigation/local_planning/reactive/vfh.py?raw";
import vfhCpp from "../../../../../cpp/src/local_planning/reactive/vfh.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — RRT*/Informed RRT* 등 global planner 페이지의 Proof 관례를
// 그대로 따른다(형식 증명은 아니지만 가정→관계식→결론 전개는 같은 틀을 쓴다).
const Collapsible = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Vfh = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Potential Fields answers "which way?" with a single vector — every nearby
                    obstacle pushes, the goal pulls, and the robot follows whatever sum comes
                    out. That sum is convenient but lossy: two very different obstacle layouts
                    can cancel out to the same resultant, and a resultant of zero looks
                    identical whether the robot is boxed in on all sides or standing in open
                    space. Vector Field Histogram (Borenstein &amp; Koren, 1991) keeps the
                    shape of the surroundings instead of collapsing it. It bins nearby
                    obstacles by bearing into a polar histogram, then reads off which
                    directions are actually open before committing to one.
                </p>}
                ko={<p>
                    Potential Fields는 "어느 쪽으로 갈까"라는 질문에 벡터 하나로 답한다.
                    주변 장애물은 밀어내고 목표는 당기고, 로봇은 그 합이 가리키는 쪽을
                    따라간다. 이 합산은 간편하지만 정보를 버린다. 전혀 다른 장애물 배치
                    둘이 같은 합력으로 상쇄될 수 있고, 합력이 0이라는 결과는 로봇이 사방이
                    막혔을 때와 탁 트인 공간에 서 있을 때가 똑같이 나온다. Vector Field
                    Histogram(Borenstein &amp; Koren, 1991)은 주변의 형태를 뭉개지 않고
                    간직한다. 근처 장애물을 방위별로 폴라 히스토그램에 나눠 담은 뒤, 실제로
                    열려 있는 방향이 어디인지 읽고 나서야 하나를 고른다.
                </p>}
            />

            <h2>{t("From Forces to a Histogram", "힘에서 히스토그램으로")}</h2>
            <T
                en={<>
                    <p>
                        Replace the single force vector with a function of bearing: for every
                        direction around the robot, how much nearby obstacle mass sits there?
                        Each sensed obstacle at bearing <InlineMath math="\beta"/> and
                        distance <InlineMath math="d"/> votes into the sector its bearing falls
                        in, weighted by how close it is. Borenstein &amp; Koren's original
                        formulation (eq. 2) derives that weight from a cell's certainty value
                        rather than distance alone:
                    </p>
                    <BlockMath math="m = (c^*)^2 (a - b \cdot d), \qquad 0 \le d \le r_w"/>
                    <Terms items={[
                        ["m", "the vote one obstacle contributes to its sector"],
                        ["c^*", "the cell's certainty value from the occupancy grid — how confident the sensor model is that the cell is occupied"],
                        ["a,\\ b", <>positive constants tuned so the magnitude reaches zero at the
                            edge of the window, i.e. <InlineMath math="a = b \cdot r_w"/></>],
                        ["d", "distance from the robot to that obstacle"],
                        ["r_w", <>the window radius — obstacles farther than <InlineMath math="r_w"/> are
                            ignored entirely</>],
                        ["\\beta", "bearing of the obstacle relative to the robot (which sector it falls in)"],
                    ]}/>
                    <p>
                        On a binary occupancy grid <InlineMath math="c^*"/> is just 0 or 1, so a
                        hit cell contributes <InlineMath math="c^*=1"/> and the certainty term
                        drops out, leaving a linear falloff with
                        distance, <InlineMath math="1 - d/r_w"/>. This implementation instead
                        squares that distance falloff — a deliberate deviation from the paper,
                        meant to weight nearby cells more sharply than a linear term does:
                    </p>
                    <BlockMath math="m = \left(1 - \frac{d}{r_w}\right)^{2}, \qquad 0 \le d \le r_w"/>
                    <Terms items={[
                        ["m", "the vote one obstacle contributes to its sector, as implemented here"],
                        ["d", "distance from the robot to that obstacle"],
                        ["r_w", <>the window radius — obstacles farther than <InlineMath math="r_w"/> are
                            ignored entirely</>],
                    ]}/>
                    <p>
                        Sum every obstacle's vote into its sector and the result is a histogram
                        over bearing, not a single number. A cluster on the left and a cluster
                        on the right no longer cancel — they show up as two separate peaks, and
                        the low ground between them is still visible as a way through.
                    </p>
                </>}
                ko={<>
                    <p>
                        힘 벡터 하나 대신 방위의 함수를 쓴다. 로봇 주변 각 방향마다, 그
                        근처에 장애물이 얼마나 몰려 있는가? 방위{" "}
                        <InlineMath math="\beta"/>, 거리 <InlineMath math="d"/>에서 감지된
                        장애물은 자신의 방위가 속하는 sector에, 가까울수록 크게 표를 던진다.
                        Borenstein &amp; Koren의 원 논문(식 2)은 이 가중치를 거리가 아니라
                        cell의 certainty 값에서 이끌어낸다:
                    </p>
                    <BlockMath math="m = (c^*)^2 (a - b \cdot d), \qquad 0 \le d \le r_w"/>
                    <Terms items={[
                        ["m", "장애물 하나가 자기 sector에 던지는 표"],
                        ["c^*", "occupancy grid의 certainty 값. 그 cell이 점유되어 있다고 센서 모델이 확신하는 정도"],
                        ["a,\\ b", <>window 경계에서 magnitude가 0이 되도록 맞춘 양의 상수, 즉{" "}
                            <InlineMath math="a = b \cdot r_w"/></>],
                        ["d", "로봇에서 그 장애물까지 거리"],
                        ["r_w", <><InlineMath math="r_w"/>보다 먼 장애물은 아예 무시하는 탐색
                            반경(window radius)</>],
                        ["\\beta", "로봇 기준 장애물의 방위(어느 sector에 속하는지 결정)"],
                    ]}/>
                    <p>
                        binary occupancy grid에서는 <InlineMath math="c^*"/>가 0 아니면 1이라
                        명중한 cell은 <InlineMath math="c^*=1"/>을 내고 certainty 항이 사라져,
                        거리에 선형으로 비례하는 감쇠 <InlineMath math="1 - d/r_w"/>만 남는다.
                        이 구현은 그 거리 감쇠를 대신 제곱한다. 논문에서 의도적으로 벗어난
                        지점으로, 선형항보다 가까운 cell에 더 날카롭게 가중치를 준다:
                    </p>
                    <BlockMath math="m = \left(1 - \frac{d}{r_w}\right)^{2}, \qquad 0 \le d \le r_w"/>
                    <Terms items={[
                        ["m", "이 구현에서 장애물 하나가 자기 sector에 던지는 표"],
                        ["d", "로봇에서 그 장애물까지 거리"],
                        ["r_w", <><InlineMath math="r_w"/>보다 먼 장애물은 아예 무시하는 탐색
                            반경(window radius)</>],
                    ]}/>
                    <p>
                        모든 장애물의 표를 각자의 sector에 누적하면 숫자 하나가 아니라
                        방위 전체에 걸친 히스토그램이 나온다. 왼쪽 뭉치와 오른쪽 뭉치는 더는
                        서로를 지우지 않는다. 봉우리 둘로 따로 나타나고, 그 사이 낮은 지대는
                        여전히 지나갈 길로 보인다.
                    </p>
                </>}
            />
            <VfhHistogramFigure/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Memoryless and reactive</strong>: every tick rebuilds the
                        histogram from scratch off the current sensor view. There is no
                        planning horizon and no notion of a path — only "which direction looks
                        open right now."</li>
                    <li><strong>Not complete</strong>: a corridor that only opens up after a
                        dead end is invisible to a purely local histogram. VFH avoids the
                        force-cancellation traps that catch Potential Fields, but it inherits
                        the same blind spot for maze-like structure — pair it with a global
                        planner for that.</li>
                    <li><strong>Cost per tick</strong>: <InlineMath math="O(k)"/> to bin the{" "}
                        <InlineMath math="k"/> obstacles within the window, plus{" "}
                        <InlineMath math="O(n)"/> to smooth and scan <InlineMath math="n"/>{" "}
                        sectors for valleys — both small and constant-time relative to sensor
                        resolution.</li>
                    <li><strong>Density-scaled speed</strong>: the robot brakes on its own as
                        the chosen direction gets denser, without a separate braking rule bolted
                        on.</li>
                    <li><strong>Robust to threshold, mostly</strong>: Borenstein &amp; Koren
                        report that VFH performance is largely insensitive to the threshold —
                        their tests show factor-of-3–4 changes mainly rescaling valley width
                        rather than flipping outcomes. The exception is marginal gaps: whether a
                        gap counts as a valley at all is a single inequality
                        against <InlineMath math="h_{\text{smooth}}[k]"/>, so near that boundary
                        a slightly different threshold can flip a passage between open and
                        blocked.</li>
                </ul>}
                ko={<ul>
                    <li><strong>무기억 반응형</strong>: 매 tick 현재 센서 뷰만으로 히스토그램을
                        처음부터 다시 만든다. 계획 horizon도, 경로라는 개념도 없다. "지금 어느
                        방향이 열려 보이는가"만 있다.</li>
                    <li><strong>완전하지 않다</strong>: 막다른 길을 지나야만 열리는 통로는
                        순전히 국소적인 히스토그램에는 보이지 않는다. Potential Fields를
                        잡는 힘 상쇄 함정은 피하지만, 미로 같은 구조에는 같은 사각지대를
                        물려받는다. 그런 상황엔 global planner를 짝지어야 한다.</li>
                    <li><strong>tick당 비용</strong>: window 안 장애물 <InlineMath math="k"/>개를
                        나눠 담는 데 <InlineMath math="O(k)"/>, sector <InlineMath math="n"/>개를
                        스무딩하고 valley를 스캔하는 데 <InlineMath math="O(n)"/>이 든다.
                        둘 다 센서 해상도에 비해 작고 상수 시간에 가깝다.</li>
                    <li><strong>밀도 비례 감속</strong>: 별도 제동 규칙을 덧붙이지 않아도
                        선택된 방향이 빽빽해질수록 로봇이 스스로 속도를 줄인다.</li>
                    <li><strong>threshold에는 대체로 강건하다</strong>: Borenstein &amp;
                        Koren은 VFH 성능이 threshold에 크게 민감하지 않다고 보고한다. 논문의
                        실험에서 threshold를 3~4배 바꿔도 대개 valley 폭만 달라질 뿐 결과가
                        뒤집히지는 않는다. 예외는 경계에 걸친 gap이다. gap이 valley로
                        인정되는지는 <InlineMath math="h_{\text{smooth}}[k]"/>와의 부등식
                        하나로 결정되므로, 그 경계 근처에서는 threshold를 살짝만 바꿔도 통로가
                        열림과 막힘 사이에서 뒤집힌다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One tick, in four passes over the sector array: build, smooth, threshold,
                    steer.
                </p>}
                ko={<p>
                    한 tick은 sector 배열을 네 번 훑는다: 만들고, 스무딩하고, threshold를
                    적용하고, 조향한다.
                </p>}
            />
            <Pseudocode code={`h ← zeros(n)
for each obstacle (β, d) within window_radius:                    # 1
    k ← sector(β)
    h[k] += (1 − d / window_radius)²
h_smooth ← circular_moving_average(h, smoothing_window)            # 2
valleys ← maximal runs of sectors where h_smooth[k] < threshold     # 3
if no valleys: return stop
k_target ← sector(bearing to goal)
best ← valley minimizing (distance in sectors to k_target, −width)  # 4
θ_sel ← steering direction for best (goal / border-hugged / center) # 5
v ← max_speed · (1 − min(h_smooth[θ_sel], h_m) / h_m)               # 6
return heading_command(θ_sel − θ, v)`}/>
            <BlockMath math="h_{\text{smooth}}[k] = \frac{1}{w}\sum_{j=-\lfloor w/2 \rfloor}^{\lfloor w/2 \rfloor} h[(k+j) \bmod n]"/>
            <Terms items={[
                ["h_{\\text{smooth}}[k]", "the smoothed histogram value at sector k"],
                ["h[k]", "the raw, unsmoothed vote total at sector k, before averaging"],
                ["w", "smoothing_window — the odd number of sectors averaged together"],
                ["n", "num_sectors — total sectors covering the full turn"],
            ]}/>
            <T
                en={<ol>
                    <li>Binning is exactly the histogram magnitude from the previous section,
                        accumulated per sector. Obstacles outside <InlineMath math="r_w"/> never
                        enter the sum.</li>
                    <li>A circular moving average over <InlineMath math="w"/> sectors removes
                        single-sector noise before anything is classified as open or blocked —
                        without it, one lucky ray between two obstacles would register as a
                        gap that is not really wide enough to drive through. Borenstein &amp;
                        Koren's eq. (5) instead uses a triangular kernel that weights the center
                        sector most heavily; this implementation uses a plain (uniform) moving
                        average as a simpler stand-in.</li>
                    <li>A valley is a maximal run below threshold; runs narrower than the
                        smoothing window are treated as ripple and dropped unless they are the
                        only opening left.</li>
                    <li>Among the surviving valleys, the tie-break is sector distance to the
                        goal bearing first, then width — a wider valley at the same distance
                        offers more room to work with.</li>
                    <li>The steering direction inside the chosen valley follows the wide/narrow
                        rule worked out in the next section — aim at the goal when it's safely
                        inside the valley, hug the near border when the valley is wide, or aim
                        for the center when it is narrow.</li>
                    <li>Speed is scaled by how crowded the chosen direction is, capped by{" "}
                        <InlineMath math="h_m"/> — a completely blocked front (no valley at all)
                        returns a stop command and lets the simulator's stall detector end the
                        episode honestly rather than pretending a heading exists.</li>
                </ol>}
                ko={<ol>
                    <li>binning은 앞 절의 히스토그램 magnitude를 sector별로 누적한 것 그대로다.{" "}
                        <InlineMath math="r_w"/> 밖의 장애물은 합에 아예 들어오지 않는다.</li>
                    <li><InlineMath math="w"/> sector에 걸친 원형 이동평균이, 어떤 것을
                        열림/막힘으로 분류하기 전에 sector 하나짜리 잡음을 지운다. 이게 없으면
                        장애물 둘 사이로 우연히 뚫린 레이 한 줄이 실제로는 지나가기엔 너무
                        좁은 gap을 열린 것으로 잘못 기록한다. 논문 식 (5)는 중심 sector에
                        가장 큰 가중치를 주는 삼각 커널을 쓰지만, 이 구현은 더 단순한 균일
                        이동평균을 쓴다.</li>
                    <li>valley는 threshold 미만인 극대 구간이다. 스무딩 창보다 좁은 구간은
                        잔물결로 보고 버리되, 남은 opening이 그것뿐이면 예외로 살린다.</li>
                    <li>살아남은 valley들 사이의 tie-break는 먼저 goal 방위까지의 sector
                        거리, 그다음 폭이다. 같은 거리면 더 넓은 valley가 다룰 여유가
                        많다.</li>
                    <li>고른 valley 안에서 실제 조향 방향은 다음 절에서 다루는 wide/narrow
                        규칙을 따른다. goal이 valley 안쪽으로 충분히 들어와 있으면 goal을
                        직접 겨누고, valley가 넓으면 가까운 경계 쪽에 붙고, 좁으면
                        중심을 겨눈다.</li>
                    <li>속도는 선택된 방향이 얼마나 붐비는지에 비례해 줄되{" "}
                        <InlineMath math="h_m"/>에서 한도가 걸린다. 전방이 완전히 막혀
                        valley가 아예 없으면 정지 명령을 반환해, 있지도 않은 방향을 꾸며내는
                        대신 시뮬레이터의 정체 판정이 에피소드를 정직하게 끝내게 한다.</li>
                </ol>}
            />

            <h2>{t("Valleys, Wide and Narrow", "넓은 valley, 좁은 valley")}</h2>
            <T
                en={<p>
                    The pseudocode above waves at "steering direction for best" without saying
                    what that direction actually is once the goal is not simply inside the
                    chosen valley. The rule splits on the valley's width, and the reasoning
                    behind the split is worth spelling out.
                </p>}
                ko={<p>
                    위 pseudocode는 "best 대한 조향 방향"이라고만 적고, goal이 고른 valley
                    안에 그냥 들어 있지 않을 때 그 방향이 정확히 무엇인지는 얼버무렸다. 규칙은
                    valley의 폭으로 갈리고, 그 갈림의 근거는 풀어 볼 값어치가 있다.
                </p>}
            />
            <Collapsible title={t(
                "Why wide valleys steer to the border, not the center",
                "넓은 valley가 중심이 아니라 경계 쪽으로 도는 이유",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Let valley <InlineMath math="V"/> span
                            sectors <InlineMath math="[s, e]"/> with
                            width <InlineMath math="w_V"/>, and let{" "}
                            <InlineMath math="k_g"/> be the goal's sector. Define the border
                            standoff
                        </p>
                        <BlockMath math="\mu = \min\!\left(\left\lfloor \tfrac{w_{\text{wide}}}{2} \right\rfloor,\; \left\lfloor \tfrac{w_V - 1}{2} \right\rfloor\right)"/>
                        <Terms items={[
                            ["V", "the valley under consideration — a maximal run of open sectors"],
                            ["s,\\ e", <>the first and last sector of <InlineMath math="V"/> (walked
                                +1 mod n from s to e)</>],
                            ["w_V", <>the width of <InlineMath math="V"/>, in sectors</>],
                            ["k_g", "the sector containing the goal bearing"],
                            ["w_{\\text{wide}}", "wide_valley_sectors — the width threshold above which a valley is classified wide"],
                            ["\\mu", <>the minimum sector distance <InlineMath math="k_g"/> must
                                keep from both borders of <InlineMath math="V"/> before the robot
                                steers straight at the goal</>],
                        ]}/>
                        <p>
                            When <InlineMath math="k_g \in V"/> and its distance to both borders
                            is <InlineMath math="\ge \mu"/>, the algorithm steers straight at the
                            goal — that is the easy case, and it is this implementation's own
                            addition on top of the 1991 paper, which always steers to the
                            midpoint <InlineMath math="(k_n + k_f)/2"/> regardless of where the
                            goal sits inside the valley. The interesting case is what happens
                            otherwise, once <InlineMath math="w_V \ge w_{\text{wide}}"/>.
                        </p>
                        <p>
                            <strong>Why not the border itself.</strong> Steering at the border
                            sector places the robot's intended heading one histogram bin away
                            from an obstacle. But the border is a snapshot of one tick's
                            geometry — as the robot advances, newly sensed obstacles can shrink
                            the valley from that same side. A heading with zero standoff is
                            already outside the valley by the time it is executed.
                        </p>
                        <p>
                            <strong>Why not the geometric center.</strong> The center is safe by
                            the same argument, but for a wide valley it can sit many sectors away
                            from <InlineMath math="k_g"/> — the robot would detour toward the
                            middle of an opening it does not need to fully cross, adding path
                            length for no safety gain once it is already <InlineMath math="\mu"/>{" "}
                            sectors clear of the obstacle.
                        </p>
                        <BlockMath math="\theta_{\text{sel}} = \text{sector\_center}\bigl(s + \lfloor w_{\text{wide}}/2 \rfloor\bigr) \ \text{ or }\ \text{sector\_center}\bigl(e - \lfloor w_{\text{wide}}/2 \rfloor\bigr)"/>
                        <Terms items={[
                            ["\\theta_{\\text{sel}}", "the selected steering direction (world bearing)"],
                            ["s,\\ e", <>the same near/far borders of <InlineMath math="V"/> as
                                above — whichever is closer to <InlineMath math="k_g"/> is
                                used</>],
                            ["w_{\\text{wide}}", "wide_valley_sectors, the same fixed offset that defines the standoff margin"],
                        ]}/>
                        <p>
                            Offsetting by exactly <InlineMath math="w_{\text{wide}}/2"/> from
                            whichever border is nearer the goal gives a constant clearance from
                            the obstacle — independent of how wide the valley actually is — while
                            hugging the opening closest to where the robot wants to go. A narrow
                            valley (<InlineMath math="w_V < w_{\text{wide}}"/>) has no room for
                            this compromise: any point inside it is already close to both
                            borders, so aiming for the plain center is both safe and as direct as
                            the valley allows. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>설정.</strong> valley <InlineMath math="V"/>가 sector{" "}
                            <InlineMath math="[s, e]"/>를 폭 <InlineMath math="w_V"/>로 덮고,{" "}
                            <InlineMath math="k_g"/>를 goal이 속한 sector라 하자. border
                            standoff를 다음과 같이 정의한다
                        </p>
                        <BlockMath math="\mu = \min\!\left(\left\lfloor \tfrac{w_{\text{wide}}}{2} \right\rfloor,\; \left\lfloor \tfrac{w_V - 1}{2} \right\rfloor\right)"/>
                        <Terms items={[
                            ["V", "지금 고려 중인 valley(열린 sector의 극대 연속 구간)"],
                            ["s,\\ e", <><InlineMath math="V"/>의 첫/끝 sector(s에서 e까지 +1 mod
                                n으로 걷는다)</>],
                            ["w_V", <><InlineMath math="V"/>의 폭(sector 수)</>],
                            ["k_g", "goal 방위가 속한 sector"],
                            ["w_{\\text{wide}}", "wide_valley_sectors(이 폭 이상이면 valley를 wide로 분류하는 임계값)"],
                            ["\\mu", <>로봇이 goal을 직접 겨누기 전에 <InlineMath math="k_g"/>가{" "}
                                <InlineMath math="V"/>의 양쪽 경계로부터 유지해야 하는 최소 sector
                                거리</>],
                        ]}/>
                        <p>
                            <InlineMath math="k_g \in V"/>이고 양쪽 경계까지 거리가{" "}
                            <InlineMath math="\ge \mu"/>이면 goal을 직접 겨눈다. 쉬운
                            경우이자, 1991년 논문에는 없는 이 구현만의 추가 규칙이다. 논문은
                            goal이 valley 어디에 있든 항상 중점{" "}
                            <InlineMath math="(k_n + k_f)/2"/>를 겨눈다. 흥미로운 쪽은 그렇지
                            않을 때, 곧 <InlineMath math="w_V \ge w_{\text{wide}}"/>일 때
                            벌어지는 일이다.
                        </p>
                        <p>
                            <strong>경계 자체를 겨누지 않는 이유.</strong> 경계 sector를
                            겨누면 로봇의 의도된 heading이 장애물에서 히스토그램 bin 하나
                            거리에 놓인다. 하지만 경계는 그 tick 한 순간의 기하일 뿐이다.
                            로봇이 전진하면 새로 감지된 장애물이 같은 쪽에서 valley를 더
                            줄일 수 있다. standoff가 0인 heading은 실행될 즈음엔 이미 valley
                            밖이다.
                        </p>
                        <p>
                            <strong>기하학적 중심을 겨누지 않는 이유.</strong> 중심은 같은
                            논리로 안전하지만, 넓은 valley라면 <InlineMath math="k_g"/>에서
                            sector 여러 개만큼 떨어져 있을 수 있다. 로봇은 굳이 완전히
                            건너지 않아도 되는 opening의 한가운데 쪽으로 돌아가게 되고, 이미{" "}
                            <InlineMath math="\mu"/> sector만큼 장애물에서 벗어난 뒤로는
                            안전 이득 없이 경로 길이만 늘어난다.
                        </p>
                        <BlockMath math="\theta_{\text{sel}} = \text{sector\_center}\bigl(s + \lfloor w_{\text{wide}}/2 \rfloor\bigr) \ \text{ 또는 }\ \text{sector\_center}\bigl(e - \lfloor w_{\text{wide}}/2 \rfloor\bigr)"/>
                        <Terms items={[
                            ["\\theta_{\\text{sel}}", "선택된 조향 방향(world 방위)"],
                            ["s,\\ e", <>위와 같은 <InlineMath math="V"/>의 근/원 경계. <InlineMath math="k_g"/>에
                                더 가까운 쪽을 쓴다</>],
                            ["w_{\\text{wide}}", "wide_valley_sectors(standoff 여유를 정의하는 같은 고정 offset)"],
                        ]}/>
                        <p>
                            goal에 더 가까운 경계에서 정확히 <InlineMath math="w_{\text{wide}}/2"/>만큼
                            안쪽으로 물러난 지점을 잡으면, valley가 실제로 얼마나 넓든 상관없이
                            장애물로부터 일정한 여유를 유지하면서 로봇이 가려는 쪽에 가장 가까운
                            opening에 붙는다. 좁은 valley(<InlineMath math="w_V < w_{\text{wide}}"/>)는
                            이런 타협의 여지가 없다. 그 안의 어느 점이든 이미 양쪽 경계에 다
                            가깝기 때문에, 그냥 중심을 겨누는 것이 안전하면서 valley가 허락하는
                            한 가장 직접적이다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Collapsible>

            <h2>Demo</h2>
            <T
                en={<p>
                    Both presets run VFH with the same fixed gains as the repository config
                    (smoothing, valley width, speed, and turning limits); only the maps are
                    shaped for the demo. The threshold and window sliders below start at
                    demo-friendly values rather than the config's tuned defaults (0.041
                    and 1.32 m) — that starting point makes valleys open and close more
                    visibly, so drag the sliders toward those numbers to see the tuned
                    behavior. In the narrow corridor the walls flank the robot the
                    whole way, so the forward valley stays pinned and it tracks straight down the
                    middle, slowing where the central pinch crowds the histogram. In dense
                    clutter, several valleys open and close as the robot moves, and the rose
                    visibly reshapes each tick as the selected direction hops between them. Raise
                    the threshold slider high enough and watch valleys disappear one at a time
                    until none are left. Open the space up and the heading wobbles as near-tied
                    valleys flip — a known VFH limitation that VFH+ damps with hysteresis.
                </p>}
                ko={<p>
                    두 프리셋 모두 스무딩, valley 폭, 속도, 회전 한계 등 나머지 게인은 저장소
                    config 값 그대로 쓰고, 맵만 데모에 맞게 구성했다. 다만 아래 threshold와
                    window 슬라이더는 config의 튜닝된 기본값(threshold 0.041, window
                    1.32m)이 아니라 valley가 열리고 닫히는 모습을 더 잘 보여주는 데모용 값에서
                    시작한다. 튜닝된 동작을 보고 싶으면 슬라이더를 그 값 쪽으로 옮기면 된다.
                    좁은 통로에서는 양쪽 벽이 로봇을 처음부터 끝까지 감싸 전방
                    valley가 고정되므로, 로봇이 통로 한가운데를 곧게 따라가다가 중앙 pinch에서
                    히스토그램이 몰리는 지점에서 느려진다. 밀집 프리셋에서는 로봇이 움직이는
                    동안 valley 여럿이 열리고 닫히고, 선택된 방향이 그 사이를 오가며 매 tick
                    장미 모양이 눈에 띄게 바뀐다. threshold 슬라이더를 충분히 올리면 valley가
                    하나씩 사라지는 것도 볼 수 있다. 공간을 열어 두면 거의 동률인 valley들이
                    뒤집히며 조향이 흔들리는데, VFH의 알려진 한계이고 VFH+가 hysteresis로
                    억제한다.
                </p>}
            />
            <VfhSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The heading-command law (turn-rate clamp, cosine-gated forward speed) is
                    shared with Potential Fields; everything specific to VFH — binning,
                    smoothing, valley search, and the wide/narrow steering rule — lives in the
                    file below, embedded in full.
                </p>}
                ko={<p>
                    조향 명령 법칙(각속도 클램프, cosine 게이트 전진 속도)은 Potential
                    Fields와 공유한다. binning, 스무딩, valley 탐색, wide/narrow 조향
                    규칙처럼 VFH 고유의 것은 전부 아래 파일에 있다. 전체를 그대로
                    embed했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/local_planning/reactive/vfh.py",
                            code: vfhPy,
                            href: `${REPO}/python/navigation/local_planning/reactive/vfh.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/local_planning/reactive/vfh.cpp",
                            code: vfhCpp,
                            href: `${REPO}/cpp/src/local_planning/reactive/vfh.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete VFH implementation, embedded from the repository sources",
                    "VFH 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    J. Borenstein, Y. Koren,{" "}
                    <a href="https://doi.org/10.1109/70.88137" target="_blank" rel="noopener noreferrer">
                        <em>The Vector Field Histogram — Fast Obstacle Avoidance for Mobile
                            Robots</em>
                    </a>,
                    IEEE Transactions on Robotics and Automation, 1991.
                </li>
            </ol>
        </>
    )
}

export default Vfh
