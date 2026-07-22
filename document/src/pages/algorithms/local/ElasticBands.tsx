import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import ElasticBandsSandbox from "../../../components/panels/local/elastic_bands/ElasticBandsSandbox";
import BandForcesFigure from "../../../components/panels/local/elastic_bands/BandForcesFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import elasticBandsPy from "../../../../../python/navigation/local_planning/band/elastic_bands.py?raw";
import elasticBandsHpp from "../../../../../cpp/include/navigation/local_planning/band/elastic_bands.hpp?raw";
import elasticBandsCpp from "../../../../../cpp/src/local_planning/band/elastic_bands.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록. 다른 알고리즘 페이지와 같은 패턴(본문은 직관, 형식적 전개는 원할 때만 편다).
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const ElasticBands = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every local planner on the previous pages tracks a fixed reference path or reacts to
                    obstacles moment by moment, but neither view holds both ideas at once: a path worth
                    keeping and a corridor worth deforming to stay clear of what the path did not anticipate.
                    Elastic Bands, introduced by Sean Quinlan and Oussama Khatib in 1993, represents that
                    corridor directly — a chain of discs strung along the path, each sized to how much free
                    space actually surrounds it — and lets two simple forces reshape the chain every control
                    tick so it stays taut, stays clear of obstacles, and still resembles the path it started
                    from.
                </p>}
                ko={<p>
                    앞선 페이지의 local planner는 저마다 고정된 참조 경로를 그대로 따라가거나, 매 순간
                    장애물에 반응할 뿐이다. 둘 중 어느 쪽도 "지킬 가치가 있는 경로"와 "경로가 미처
                    예상하지 못한 것을 피해 변형할 가치가 있는 통로"라는 두 생각을 한꺼번에 담지 못한다.
                    Sean Quinlan과 Oussama Khatib가 1993년 내놓은 Elastic Bands는 이 통로를 직접
                    표현한다. 경로를 따라 꿴 원반들의 사슬이고, 각 원반의 크기는 그 자리에 실제로 남은
                    여유 공간만큼이다. 매 제어 tick마다 단순한 두 힘이 이 사슬을 다시 빚어, 팽팽함을
                    유지하고 장애물에서 벗어나 있으면서도 원래 경로를 여전히 닮은 모습으로 만든다.
                </p>}
            />

            <h2>{t("Bubbles and the Band", "버블과 밴드")}</h2>
            <T
                en={<p>
                    A band is a sequence of <strong>bubbles</strong> <InlineMath math="(c_0, \rho_0), \ldots,
                    (c_{N-1}, \rho_{N-1})"/> — world-space center points paired with a radius. The radius is
                    not a tuning choice; it is read directly off the map as the room the robot actually has
                    at that point:
                </p>}
                ko={<p>
                    밴드는 <strong>bubble</strong> 열 <InlineMath math="(c_0, \rho_0), \ldots, (c_{N-1},
                    \rho_{N-1})"/>이다. world 좌표 중심점에 반경을 붙인 것이다. 이 반경은 튜닝으로 정하는
                    값이 아니라, 그 지점에 로봇이 실제로 갖는 여유를 지도에서 그대로 읽은 값이다:
                </p>}
            />
            <BlockMath math="\rho(c) = \min\big(\text{distance\_to\_nearest}(c),\ \rho_{\max}\big)"/>
            <T
                en={<Terms items={[
                    ["\\rho(c)", "clearance at point c — the radius of the bubble centered there"],
                    ["\\text{distance\\_to\\_nearest}(c)", "distance from c to the nearest occupied cell on the map"],
                    ["\\rho_{\\max}", "a fixed cap on bubble radius, so one bubble in an open room cannot swallow the whole band"],
                ]}/>}
                ko={<Terms items={[
                    ["\\rho(c)", "점 c의 clearance. 그 자리에 놓인 bubble의 반경"],
                    ["\\text{distance\\_to\\_nearest}(c)", "c에서 지도 위 가장 가까운 점유 셀까지의 거리"],
                    ["\\rho_{\\max}", "bubble 반경의 고정 상한. 뻥 뚫린 방에서 bubble 하나가 밴드 전체를 집어삼키지 않게 한다"],
                ]}/>}
            />
            <T
                en={<p>
                    The two endpoints are anchors, not free variables: <InlineMath math="c_0"/> is re-pinned
                    to the robot's executed pose every tick, and <InlineMath math="c_{N-1}"/> to the goal.
                    Every interior bubble is free to move, and as long as consecutive bubbles overlap — each
                    one's disc reaching into its neighbor's — the union of the chain is a guaranteed
                    collision-free corridor from robot to goal, whatever shape it bends into.
                </p>}
                ko={<p>
                    양 끝점은 자유 변수가 아니라 고정된 닻이다. <InlineMath math="c_0"/>는 매 tick 로봇이
                    실제로 도달한 pose로 다시 고정되고, <InlineMath math="c_{N-1}"/>은 goal로 고정된다.
                    내부 bubble은 모두 자유롭게 움직일 수 있고, 인접한 bubble끼리 서로 겹쳐 있는 한(한쪽의
                    원반이 다른 쪽 영역까지 닿아 있는 한) 그 사슬의 합집합은 어떤 모양으로 휘든 로봇에서
                    goal까지 충돌 없는 통로임이 보장된다.
                </p>}
            />

            <h2>{t("Forces on the Band", "밴드에 작용하는 힘")}</h2>
            <T
                en={<p>
                    Two forces reshape the interior bubbles every deformation pass. The first is purely
                    geometric — it does not know obstacles exist — and simply pulls each bubble toward the
                    midpoint of its neighbors:
                </p>}
                ko={<p>
                    매 변형 반복마다 두 힘이 내부 bubble을 다시 빚는다. 첫 번째는 순전히 기하적이다.
                    장애물이 있는지조차 모르고, 그저 각 bubble을 양 이웃의 중간 쪽으로 당길 뿐이다:
                </p>}
            />
            <BlockMath math="f_{c,i} = k_{\text{contraction}} \left( \frac{c_{i-1} - c_i}{\lVert c_{i-1} - c_i \rVert} + \frac{c_{i+1} - c_i}{\lVert c_{i+1} - c_i \rVert} \right)"/>
            <T
                en={<Terms items={[
                    ["f_{c,i}", "internal contraction force on interior bubble i — pulls it taut against its neighbors"],
                    ["c_{i-1},\\ c_i,\\ c_{i+1}", "centers of bubble i and its two immediate neighbors along the chain"],
                    ["k_{\\text{contraction}}", "contraction gain — the only knob on how aggressively the band tautens"],
                ]}/>}
                ko={<Terms items={[
                    ["f_{c,i}", "내부 bubble i에 작용하는 수축력. 양 이웃 쪽으로 당겨 팽팽하게 만든다"],
                    ["c_{i-1},\\ c_i,\\ c_{i+1}", "사슬을 따라 놓인 bubble i와 바로 양옆 두 이웃의 중심"],
                    ["k_{\\text{contraction}}", "수축력 게인. 밴드가 얼마나 세게 팽팽해지는지를 정하는 유일한 손잡이"],
                ]}/>}
            />
            <T
                en={<p>
                    The second force is where the map enters. A naive finite-difference gradient of
                    clearance is nearly useless here — <InlineMath math="\text{distance\_to\_nearest}"/> is
                    quantized to the grid, so a small step almost never changes it and the gradient reads
                    zero almost everywhere. Instead, every occupied cell within an influence radius pushes on
                    its own, and the pushes sum:
                </p>}
                ko={<p>
                    두 번째 힘에서 지도가 개입한다. clearance의 단순 유한차분 gradient는 여기서 거의
                    쓸모가 없다. <InlineMath math="\text{distance\_to\_nearest}"/>는 격자 단위로 양자화돼
                    있어 작은 step으로는 값이 거의 바뀌지 않고, gradient는 거의 모든 곳에서 0으로
                    읽힌다. 대신 영향 반경 안의 점유 셀 하나하나가 저마다 밀어내고, 그 힘들을 모두
                    더한다:
                </p>}
            />
            <BlockMath math="f_{r,i} = k_{\text{repulsion}} \sum_{o \,\in\, \mathcal{O}(c_i,\, \rho_{\text{influence}})} \big(\rho_{\text{influence}} - d_o\big) \frac{c_i - o}{d_o}, \qquad d_o = \lVert c_i - o \rVert"/>
            <T
                en={<Terms items={[
                    ["f_{r,i}", "external repulsion force on interior bubble i, before tangent removal"],
                    ["\\mathcal{O}(c_i,\\ \\rho_{\\text{influence}})", "every occupied cell center within radius \\rho_{\\text{influence}} of c_i"],
                    ["o", "one such occupied cell center"],
                    ["d_o", "distance from c_i to o (terms with d_o \\approx 0 are skipped, not divided by)"],
                    ["k_{\\text{repulsion}}", "repulsion gain"],
                    ["\\rho_{\\text{influence}}", "influence radius — obstacles farther than this contribute nothing"],
                ]}/>}
                ko={<Terms items={[
                    ["f_{r,i}", "내부 bubble i에 작용하는 외부 반발력. 접선 제거 전 값"],
                    ["\\mathcal{O}(c_i,\\ \\rho_{\\text{influence}})", "c_i에서 반경 \\rho_{\\text{influence}} 안의 모든 점유 셀 중심"],
                    ["o", "그 안의 점유 셀 중심 하나"],
                    ["d_o", "c_i에서 o까지의 거리 (d_o \\approx 0인 항은 나눗셈 없이 skip)"],
                    ["k_{\\text{repulsion}}", "반발력 게인"],
                    ["\\rho_{\\text{influence}}", "영향 반경. 이보다 먼 장애물은 아무 기여도 하지 않는다"],
                ]}/>}
            />
            <T
                en={<p>
                    Summing over every occupied cell — not just the nearest one — matters when a bubble
                    starts out embedded inside a multi-cell obstacle (the raw reference path cut through it):
                    the nearest single cell alone could point back inward, but the sum of pushes from every
                    surrounding cell always points toward the obstacle's nearest edge. The last piece is{" "}
                    <strong>tangent removal</strong>, applied to the repulsion only, and it is a new term this
                    section introduces:
                </p>}
                ko={<p>
                    가장 가까운 셀 하나가 아니라 모든 점유 셀에 대해 합산하는 것은, bubble이 애초에 다중
                    셀 장애물 내부에서 시작할 때(원본 참조 경로가 그 장애물을 관통했을 때) 중요해진다.
                    가장 가까운 셀 하나만 보면 방향이 오히려 안쪽을 가리킬 수 있지만, 주변 모든 셀에서
                    오는 힘을 합치면 언제나 장애물의 가장 가까운 가장자리 쪽을 향한다. 마지막 조각은
                    반발력에만 적용하는 <strong>접선 제거</strong>다. 이 절에서 새로 도입하는 항이다:
                </p>}
            />
            <BlockMath math="\hat t_i = \frac{c_{i+1} - c_{i-1}}{\lVert c_{i+1} - c_{i-1} \rVert}, \qquad \tilde f_i = f_{c,i} + f_{r,i} - (f_{r,i} \cdot \hat t_i)\, \hat t_i"/>
            <T
                en={<Terms items={[
                    ["\\hat t_i", "unit tangent along the band at bubble i, from neighbor to neighbor (new term)"],
                    ["\\tilde f_i", "net force applied to bubble i this pass — contraction plus repulsion with its tangential component removed (new term)"],
                    ["f_{c,i},\\ f_{r,i}", "contraction and raw repulsion, as defined above"],
                ]}/>}
                ko={<Terms items={[
                    ["\\hat t_i", "bubble i 위치에서 밴드를 따르는 단위 접선 벡터, 이웃에서 이웃 방향 (새로 도입된 항)"],
                    ["\\tilde f_i", "이번 반복에서 bubble i에 적용되는 순 힘. 수축력에 접선 성분을 제거한 반발력을 더한 값 (새로 도입된 항)"],
                    ["f_{c,i},\\ f_{r,i}", "위에서 정의한 수축력과 접선 제거 전 반발력"],
                ]}/>}
            />
            <T
                en={<p>
                    Quinlan and Khatib remove the tangential part of the repulsion (and only the repulsion)
                    because it does nothing useful: pushed along the band instead of off it, it just slides
                    bubbles into each other. The contraction force's tangential component is kept for exactly
                    the opposite reason — it is what spaces bubbles evenly along the chain.
                </p>}
                ko={<p>
                    Quinlan과 Khatib가 반발력의(오직 반발력만의) 접선 성분을 제거하는 이유는, 그 성분이
                    아무 쓸모가 없기 때문이다. 밴드 밖으로 미는 대신 밴드를 따라 밀면 bubble들이 서로를
                    향해 미끄러져 들어갈 뿐이다. 수축력의 접선 성분은 정반대 이유로 남겨 둔다. 그것이 바로
                    bubble들을 사슬 위에 고르게 배치해 주는 힘이기 때문이다.
                </p>}
            />
            <BandForcesFigure/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Cost: <InlineMath math="O(N \cdot \text{deform\_iterations})"/> per
                            tick.</strong> Every one of the fixed <InlineMath math="\text{deform\_iterations}"/>{" "}
                            passes touches every interior bubble once, and each touch queries the map for
                            occupied cells within the influence radius — cheap, and independent of how far the
                            band is from the goal.</li>
                        <li><strong>No optimality claim, and no completeness guarantee.</strong> The band
                            settles into <em>a</em> local force balance between tautness and clearance, not
                            the shortest or safest corridor possible — the same local-minimum caveat that
                            applies to every reactive force method in this section.</li>
                        <li><strong>Purely local corridor repair, not a planner.</strong> A band only ever
                            deforms the corridor it already has; if the reference path is blocked in a way no
                            amount of pushing can route around, the maintenance check in the next section
                            declares the band broken rather than pretending it found a way through.</li>
                        <li><strong>Reacts to change for free.</strong> Because every tick re-deforms from the
                            band's own current shape rather than re-solving from scratch, a wall painted mid-
                            episode (or erased) just becomes a new set of forces on the next deformation pass
                            — no separate replanning trigger needed.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>비용: tick당 <InlineMath math="O(N \cdot \text{deform\_iterations})"/>.</strong>{" "}
                            고정된 <InlineMath math="\text{deform\_iterations}"/>회 반복마다 모든 내부
                            bubble을 한 번씩 건드리고, 매번 영향 반경 안의 점유 셀을 지도에 질의한다.
                            저렴하고, goal까지 남은 거리와 무관하다.</li>
                        <li><strong>최적성도, 완전성도 보장하지 않는다.</strong> 밴드는 팽팽함과 clearance
                            사이의 <em>어떤</em> 국소 힘 균형에 안착할 뿐, 가능한 가장 짧거나 안전한
                            통로가 아니다. 이 절의 모든 반응형 힘 기반 방법이 공유하는 local minimum
                            한계와 같다.</li>
                        <li><strong>순전히 국소적인 통로 수리이지 planner가 아니다.</strong> 밴드는 이미
                            가진 통로만 변형할 뿐이다. 참조 경로가 아무리 밀어도 돌아갈 수 없게 막혀
                            있다면, 다음 절의 유효성 검사가 "찾은 척"하지 않고 밴드를 broken으로
                            선언한다.</li>
                        <li><strong>변화에 별도 대응 없이 그냥 반응한다.</strong> 매 tick 처음부터 다시
                            푸는 대신 밴드 자신의 현재 모양에서부터 다시 변형하므로, 에피소드 중간에 벽을
                            그리거나 지우면 다음 변형 반복에서 그저 새로운 힘 하나로 반영될 뿐이다. 별도의
                            재계획 트리거가 필요 없다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The only state that survives between ticks is the band itself — the list of bubbles. Every
                    tick re-pins the endpoints, deforms the interior a fixed number of times, and repairs the
                    bubble spacing before extracting a command from whatever shape the band ended up in.
                </p>}
                ko={<p>
                    tick 사이에 남는 상태는 밴드 자신, 곧 bubble 목록뿐이다. 매 tick 양 끝점을 다시
                    고정하고, 내부를 고정 횟수만큼 변형한 뒤, bubble 간격을 수리하고 나서, 그 결과 모양의
                    밴드에서 명령을 뽑는다.
                </p>}
            />
            <Pseudocode code={`if band is empty:                                                    # 1
    band ← resample(reference_path, bubble_spacing)                  # 2
    band ← [robot] + band + [goal]
    for k in 1..repair_iterations: deform_once(band)                 # 3
    if not maintain(band): return broken()                           # 4
while len(band) > 2 and dist(robot, band[1].c) <= band[1].rho:        # 5
    pop_front(band)
band[0].c ← robot_xy                                                 # 6
for k in 1..deform_iterations:                                        # 7
    deform_once(band)                                                 # 8
if not maintain(band): return broken()                                # 9
emit(band_updated)                                                    # 10
target ← point_at_arclength(band, lookahead_distance)                 # 11
alpha ← wrap(atan2(target_y - y, target_x - x) - theta)
v ← v_max * max(cos(alpha), 0)                                        # 12
omega ← clamp(heading_gain * alpha, -omega_max, omega_max)
return (v, omega)`}/>
            <T
                en={<ol>
                    <li>No band survives from last tick — first tick, right after <code>reset()</code>, or the
                        previous tick just broke — so this tick starts by building one from scratch.</li>
                    <li>Resample the reference path at even arc-length spacing and bracket it with the robot's
                        current position in front and the goal behind.</li>
                    <li><strong>The pitfall:</strong> if the raw reference path cuts through an obstacle, the
                        freshly resampled band starts with a bubble at zero clearance, embedded in it. Running
                        the maintenance check immediately would declare it broken before it ever had a chance
                        to move — so a fixed number of repair passes runs first, giving the repulsion sum time
                        to push that bubble out (its displacement is floored, not just capped, so it never
                        freezes at zero step).</li>
                    <li>Only now does the validity check run. If a bubble is still below the minimum radius
                        after repair, the obstruction is real, not just an initialization artifact — hand off
                        to the broken-band path.</li>
                    <li>Front-pruning: drop bubbles the robot has already walked past — its position sits
                        inside the next bubble's own clearance disc. This never touches index 0 directly; it
                        only ever removes bubble 1 and shifts everything down, so the anchor always survives
                        to be re-pinned next.</li>
                    <li>Re-pin the anchor to the pose the robot actually reached this tick, not the pose it
                        was commanded toward.</li>
                    <li>Deform a fixed number of times — no early exit, even if the band looks settled after
                        one pass. A fixed iteration count is what keeps this bit-identical across languages.</li>
                    <li>Each pass computes every bubble's force from a snapshot of the band as the pass began,
                        then applies every displacement together — so the order bubbles happen to be visited
                        in never changes what force any of them feel.</li>
                    <li><strong>The other pitfall:</strong> deformation alone can leave bubbles too far apart
                        (gap grew past what overlap can guarantee) or leave one below minimum radius (painted
                        a wall across the band mid-episode) — maintenance inserts, deletes, and re-validates
                        before anything downstream trusts the band's shape.</li>
                    <li>Only once the band is valid does it get published to the trace, so a viewer replaying
                        the episode never sees a broken half-shape mid-repair.</li>
                    <li>The command target is a point on the band's own polyline, not on the original
                        reference path — a fixed arc-length ahead of the anchor.</li>
                    <li>Plain proportional heading control drives toward it. The band is a whole curve
                        re-deformed every tick, not a progress-indexed path, so there is no lookahead-circle
                        intersection to compute here the way the tracking family does it.</li>
                </ol>}
                ko={<ol>
                    <li>지난 tick에서 살아남은 밴드가 없다. 첫 tick이거나, <code>reset()</code> 직후이거나,
                        직전 tick이 방금 broken됐다. 그래서 이번 tick은 처음부터 밴드를 다시 짓는다.</li>
                    <li>참조 경로를 균등 arc-length 간격으로 재샘플하고, 앞에는 로봇의 현재 위치를, 뒤에는
                        goal을 붙인다.</li>
                    <li><strong>함정.</strong> 원본 참조 경로가 장애물을 관통한다면, 갓 재샘플된 밴드는
                        clearance 0인 bubble을 그 안에 품은 채 시작한다. 곧바로 유효성 검사를 돌리면
                        움직여 볼 기회조차 없이 broken으로 선언될 것이다. 그래서 고정 횟수의 repair 반복을
                        먼저 돌려, 반발력 합산이 그 bubble을 밀어낼 시간을 준다(이동량이 그냥 상한만
                        있는 게 아니라 하한도 있어, step 0에 영원히 얼어붙지 않는다).</li>
                    <li>이제야 유효성 검사를 한다. repair 뒤에도 어떤 bubble이 여전히 최소 반경 미만이면,
                        그것은 초기화 부작용이 아니라 진짜 봉쇄다. broken 처리 경로로 넘긴다.</li>
                    <li>front-pruning. 로봇이 이미 지나친 bubble을 버린다. 로봇 위치가 다음 bubble
                        자신의 clearance 원 안에 들어와 있으면 지운다. 이 과정은 index 0을 직접 건드리지
                        않는다. 항상 bubble 1만 지우고 나머지를 앞으로 당길 뿐이라, 앵커는 언제나 다음
                        재고정을 받을 수 있게 남는다.</li>
                    <li>앵커를 명령했던 pose가 아니라 로봇이 이번 tick 실제로 도달한 pose로 다시
                        고정한다.</li>
                    <li>고정 횟수만큼 변형한다. 한 번 돌려 보고 안정돼 보여도 조기 종료하지 않는다. 고정
                        반복 횟수야말로 언어 간 bit-identical을 지키는 핵심이다.</li>
                    <li>매 반복은 그 반복이 시작될 때의 밴드 스냅샷으로 모든 bubble의 힘을 계산한 다음,
                        모든 변위를 한꺼번에 적용한다. 그래서 bubble이 우연히 어떤 순서로 방문되든 서로가
                        느끼는 힘은 절대 달라지지 않는다.</li>
                    <li><strong>또 다른 함정.</strong> 변형만으로는 bubble 사이가 overlap이 보장할 수
                        없을 만큼 벌어지거나(gap 확대), 어떤 bubble이 최소 반경 아래로 떨어질 수 있다(예:
                        에피소드 중간에 밴드 위에 벽을 그린 경우). maintenance가 삽입·삭제·재검증까지
                        마쳐야 이후 어느 단계도 밴드 모양을 믿고 쓸 수 있다.</li>
                    <li>밴드가 유효할 때만 trace로 방출한다. 그래야 재생을 보는 사람이 repair 도중의
                        깨진 반쪽 모양을 볼 일이 없다.</li>
                    <li>명령 목표점은 원본 참조 경로가 아니라 밴드 자신의 폴리라인 위 점이다. 앵커에서
                        고정 arc-length만큼 앞선 지점이다.</li>
                    <li>단순 비례 heading 제어로 그 점을 향해 조향한다. 밴드는 매 tick 통째로 재변형되는
                        곡선이지 진행 index를 가진 경로가 아니므로, tracking 계열이 쓰는 lookahead-원
                        교차 계산은 여기서 필요 없다.</li>
                </ol>}
            />

            <Proof title={t(
                "Derivation (contraction is gradient descent on band length)",
                "유도 (수축력은 밴드 길이의 gradient descent다)",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Define the band's total polyline length as a function of
                            every bubble center:
                        </p>
                        <BlockMath math="L(c_0, \ldots, c_{N-1}) = \sum_{k=0}^{N-2} \lVert c_{k+1} - c_k \rVert"/>
                        <Terms items={[
                            ["L", "total arc length of the band's center polyline"],
                            ["c_0, \\ldots, c_{N-1}", "every bubble center, endpoints included"],
                        ]}/>
                        <p>
                            For an interior bubble <InlineMath math="i"/>, only two terms of the sum contain{" "}
                            <InlineMath math="c_i"/>: the segment behind it and the segment ahead of it.
                            Differentiating each with the standard identity{" "}
                            <InlineMath math="\partial \lVert v \rVert / \partial v = v / \lVert v \rVert"/>:
                        </p>
                        <BlockMath math="\frac{\partial L}{\partial c_i} = \frac{c_i - c_{i-1}}{\lVert c_i - c_{i-1} \rVert} + \frac{c_i - c_{i+1}}{\lVert c_{i+1} - c_i \rVert}"/>
                        <Terms items={[
                            ["\\partial L / \\partial c_i", "gradient of total band length with respect to bubble i's center"],
                            ["c_{i-1},\\ c_i,\\ c_{i+1}", "as above: bubble i and its two neighbors"],
                        ]}/>
                        <p>
                            Negate and rewrite each term with a minus sign folded into the numerator instead
                            of the denominator:
                        </p>
                        <BlockMath math="-\frac{\partial L}{\partial c_i} = \frac{c_{i-1} - c_i}{\lVert c_{i-1} - c_i \rVert} + \frac{c_{i+1} - c_i}{\lVert c_{i+1} - c_i \rVert}"/>
                        <Terms items={[
                            ["-\\partial L / \\partial c_i", "negative gradient of band length — the direction that shortens it fastest"],
                        ]}/>
                        <p>
                            The right-hand side is exactly the bracketed sum in the contraction force
                            definition. So:
                        </p>
                        <BlockMath math="f_{c,i} = k_{\text{contraction}} \left( -\frac{\partial L}{\partial c_i} \right)"/>
                        <Terms items={[
                            ["f_{c,i}", "the contraction force from the definition above"],
                            ["k_{\\text{contraction}}", "contraction gain, playing the role of a gradient-descent step size"],
                        ]}/>
                        <p>
                            Applying <InlineMath math="f_{c,i}"/> to every interior bubble is literally one
                            step of gradient descent on the band's total length, with the two endpoints held
                            fixed as boundary conditions. That is the precise sense in which contraction "keeps
                            the band taut": it is minimizing arc length, the same quantity a stretched physical
                            band would minimize on its own.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 밴드의 전체 폴리라인 길이를 모든 bubble 중심의 함수로
                            정의한다:
                        </p>
                        <BlockMath math="L(c_0, \ldots, c_{N-1}) = \sum_{k=0}^{N-2} \lVert c_{k+1} - c_k \rVert"/>
                        <Terms items={[
                            ["L", "밴드 중심 폴리라인의 전체 arc length"],
                            ["c_0, \\ldots, c_{N-1}", "끝점을 포함한 모든 bubble 중심"],
                        ]}/>
                        <p>
                            내부 bubble <InlineMath math="i"/>에 대해서는 합의 항 중 딱 둘만{" "}
                            <InlineMath math="c_i"/>를 포함한다. 바로 뒤 구간과 바로 앞 구간이다. 표준
                            항등식 <InlineMath math="\partial \lVert v \rVert / \partial v = v / \lVert v
                            \rVert"/>로 각각 미분하면:
                        </p>
                        <BlockMath math="\frac{\partial L}{\partial c_i} = \frac{c_i - c_{i-1}}{\lVert c_i - c_{i-1} \rVert} + \frac{c_i - c_{i+1}}{\lVert c_{i+1} - c_i \rVert}"/>
                        <Terms items={[
                            ["\\partial L / \\partial c_i", "bubble i 중심에 대한 밴드 전체 길이의 gradient"],
                            ["c_{i-1},\\ c_i,\\ c_{i+1}", "위와 동일: bubble i와 그 양 이웃"],
                        ]}/>
                        <p>
                            부호를 뒤집고, 마이너스 부호를 분모가 아니라 분자 안으로 접어 넣어 다시 쓴다:
                        </p>
                        <BlockMath math="-\frac{\partial L}{\partial c_i} = \frac{c_{i-1} - c_i}{\lVert c_{i-1} - c_i \rVert} + \frac{c_{i+1} - c_i}{\lVert c_{i+1} - c_i \rVert}"/>
                        <Terms items={[
                            ["-\\partial L / \\partial c_i", "밴드 길이의 음의 gradient. 길이를 가장 빨리 줄이는 방향"],
                        ]}/>
                        <p>
                            우변은 수축력 정의의 대괄호 안 합과 정확히 같다. 따라서:
                        </p>
                        <BlockMath math="f_{c,i} = k_{\text{contraction}} \left( -\frac{\partial L}{\partial c_i} \right)"/>
                        <Terms items={[
                            ["f_{c,i}", "위에서 정의한 수축력"],
                            ["k_{\\text{contraction}}", "수축력 게인. gradient descent의 step 크기 역할을 한다"],
                        ]}/>
                        <p>
                            모든 내부 bubble에 <InlineMath math="f_{c,i}"/>를 적용하는 것은, 양 끝점을
                            경계 조건으로 고정한 채 밴드 전체 길이에 대한 gradient descent를 정확히 한
                            step 밟는 것이다. "수축력이 밴드를 팽팽하게 유지한다"는 말의 정확한 의미가
                            바로 이것이다. 늘어난 물리적 고무줄이 스스로 최소화하는 것과 같은 양, arc
                            length를 최소화하고 있는 것이다.
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs Elastic Bands live in your browser. The translucent chain of
                    bubbles is the band as it stood at the end of this tick's deformation — watch it swell
                    around obstacles and pinch through gaps. Paint a wall across the band mid-replay and it
                    reacts on the very next tick; erase it and the band recovers.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 Elastic Bands를 라이브로 실행한다. 반투명한 bubble
                    사슬이 이번 tick 변형이 끝난 시점의 밴드다. 장애물 주변에서 부풀고 틈 사이에서
                    조여드는 모습을 보라. 재생 도중 밴드 위에 벽을 그리면 바로 다음 tick부터 반응하고,
                    지우면 다시 회복한다.
                </p>}
            />
            <ElasticBandsSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below follows the algorithm above line for line: repair-then-validate
                    initialization, front-pruning, Jacobi-style deformation, and overlap maintenance. The code
                    is the actual repository source, not an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 거의 그대로 옮긴 것이다. repair 후 검증하는 초기화,
                    front-pruning, Jacobi 방식 변형, overlap maintenance까지 그대로다. 아래 코드는
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
                                name: "python/navigation/local_planning/band/elastic_bands.py",
                                code: elasticBandsPy,
                                href: `${REPO}/python/navigation/local_planning/band/elastic_bands.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/band/elastic_bands.hpp",
                                code: elasticBandsHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/band/elastic_bands.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/band/elastic_bands.cpp",
                                code: elasticBandsCpp,
                                href: `${REPO}/cpp/src/local_planning/band/elastic_bands.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "Repair-then-validate initialization, front-pruning, Jacobi deformation, and overlap maintenance, embedded from the repository sources",
                    "repair 후 검증하는 초기화, front-pruning, Jacobi 변형, overlap maintenance. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. Quinlan, O. Khatib,{" "}
                    <a href="https://doi.org/10.1109/ROBOT.1993.291936" target="_blank" rel="noopener noreferrer">
                        <em>Elastic Bands: Connecting Path Planning and Control</em>
                    </a>, Proceedings of IEEE International Conference on Robotics and Automation (ICRA), 1993,
                    pp. 802–807.
                </li>
            </ol>
        </>
    )
}

export default ElasticBands
