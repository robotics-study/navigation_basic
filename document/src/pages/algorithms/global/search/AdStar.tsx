import {T, useTr} from "../../../../libs/i18n";
import {InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import AdStarSandbox from "../../../../components/panels/global/ad_star/AdStarSandbox";
import FamilyMap from "../../../../components/panels/global/ad_star/FamilyMap";
import adPy from "../../../../../../python/navigation/global_planning/search/ad_star.py?raw";
import adCpp from "../../../../../../cpp/src/global_planning/search/ad_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const AdStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The last three pages solved two different hard problems: ARA* buys a usable
                    answer <em>now</em> and improves it, D* Lite survives a map that turns out to
                    be wrong. A real robot usually needs both at once — it is driving through an
                    unknown building <em>and</em> its control loop wants a plan this cycle. AD*
                    (Likhachev, Ferguson, Gordon, Stentz &amp; Thrun, 2005) is the synthesis, and
                    the closing piece of this incremental/anytime arc.
                </p>}
                ko={<p>
                    앞의 세 페이지는 서로 다른 두 난제를 풀었다. ARA*는 쓸 만한 답을{" "}
                    <em>지금</em> 사고 나서 개선하고, D* Lite는 지도가 틀렸다는 사실을 버텨 낸다.
                    실제 로봇에게는 대개 둘이 동시에 필요하다. 모르는 건물을 주행하는{" "}
                    <em>동시에</em> 제어 루프는 이번 주기의 계획을 요구한다. AD*(Likhachev,
                    Ferguson, Gordon, Stentz &amp; Thrun, 2005)가 그 종합이고, 이
                    incremental/anytime 계열의 마무리다.
                </p>}
            />

            <h2>{t("Two Axes, One Planner", "두 축, 하나의 planner")}</h2>
            <FamilyMap/>
            <T
                en={<>
                    <p>
                        AD* is D* Lite's backward, repairable search with ARA*'s two devices
                        grafted on:
                    </p>
                    <ul>
                        <li><strong>ε-inflated keys, but only for over-consistent vertices.</strong>{" "}
                            A vertex whose cost estimate just <em>improved</em>{" "}
                            (<InlineMath math="g > rhs"/>) may ride an inflated heuristic — that is
                            what makes the first plan fast. A vertex whose cost{" "}
                            <em>went up</em> (an under-consistent one, typically next to a newly
                            discovered wall) keeps an un-inflated key, because cost increases must
                            propagate on an admissible priority or the repair would be wrong.</li>
                        <li><strong>INCONS between passes.</strong> Exactly as in ARA*: improvements
                            landing on already-expanded vertices wait for the next reopen, where
                            OPEN ∪ INCONS is re-keyed under the current ε.</li>
                    </ul>
                    <p>
                        The schedule ties it together: tighten ε toward 1 while stationary, step
                        only when the plan is belief-optimal, and when the sensor reveals a wall,
                        re-inflate ε to grab a quick provisional plan before tightening again.
                    </p>
                </>}
                ko={<>
                    <p>
                        AD*는 D* Lite의 backward 수리 탐색에 ARA*의 장치 두 개를 이식한 것이다:
                    </p>
                    <ul>
                        <li><strong>ε-팽창 키, 단 over-consistent 정점에만.</strong> 비용 추정이
                            방금 <em>좋아진</em> 정점(<InlineMath math="g > rhs"/>)은 부풀린
                            heuristic을 타도 된다. 첫 계획이 빨리 나오는 이유다. 비용이{" "}
                            <em>오른</em> 정점(under-consistent, 보통 새로 발견된 벽 옆)은 팽창
                            없는 키를 유지한다. 인상은 admissible한 우선순위로 전파돼야 수리가
                            틀리지 않기 때문이다.</li>
                        <li><strong>패스 사이의 INCONS.</strong> ARA*와 정확히 같다. 이미 확장된
                            정점에 떨어진 개선은 다음 reopen을 기다리고, 그때 OPEN ∪ INCONS가
                            현재 ε 기준으로 재키잉된다.</li>
                    </ul>
                    <p>
                        스케줄이 전체를 묶는다. 서 있는 동안 ε을 1로 조이고, 계획이 belief-최적일
                        때만 한 걸음 움직이고, 센서가 벽을 드러내면 ε을 다시 올려 임시 계획부터
                        빨리 쥔 뒤 도로 조인다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Anytime</strong>: every published plan is{" "}
                        <InlineMath math="\varepsilon"/>-suboptimal for the current belief, with
                        the current ε as the bound.</li>
                    <li><strong>Incremental</strong>: discoveries trigger local repair on the
                        g/rhs field, not a restart — D* Lite's economics carry over.</li>
                    <li><strong>Belief-optimal motion</strong>: the robot steps only at{" "}
                        <InlineMath math="\varepsilon = 1"/>, so the executed trajectory matches
                        D* Lite's; what AD* buys is having usable plans <em>earlier</em> along the
                        way.</li>
                    <li><strong>Cost</strong>: worst case remains a full search; the ε schedule
                        adds bounded rework (each reopen re-keys OPEN ∪ INCONS).</li>
                </ul>}
                ko={<ul>
                    <li><strong>Anytime</strong>: 발표되는 모든 계획이 현재 believe에 대해{" "}
                        <InlineMath math="\varepsilon"/>-준최적이고, 그때의 ε이 한계값이다.</li>
                    <li><strong>Incremental</strong>: 발견은 재시작이 아니라 g/rhs 장의 국소
                        수리를 부른다. D* Lite의 경제성이 그대로 이어진다.</li>
                    <li><strong>Belief-최적 이동</strong>: 로봇은{" "}
                        <InlineMath math="\varepsilon = 1"/>에서만 걸음을 옮기므로 실행 궤적은
                        D* Lite와 같다. AD*가 사는 것은 그 과정에서 쓸 만한 계획을{" "}
                        <em>더 일찍</em> 쥐고 있다는 점이다.</li>
                    <li><strong>비용</strong>: 최악은 여전히 전체 탐색이고, ε 스케줄은 유계의
                        재작업(reopen마다 OPEN ∪ INCONS 재키잉)을 더한다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Everything from D* Lite is here — backward g/rhs, <InlineMath math="k_m"/>,
                    move–sense–repair — with the key function and the outer loop changed:
                </p>}
                ko={<p>
                    D* Lite의 모든 것(backward g/rhs, <InlineMath math="k_m"/>,
                    move–sense–repair)이 그대로 있고, key 함수와 바깥 루프가 달라진다:
                </p>}
            />
            <Pseudocode code={`key(s):                                                     # 1
    if g[s] > rhs[s]:  return [rhs[s] + ε·h(robot, s) + k_m,  rhs[s]]
    else:              return [g[s]  +   h(robot, s) + k_m,  g[s]]

update_state(u):                                            # 2
    recompute rhs[u];  remove u from OPEN and INCONS
    if g[u] ≠ rhs[u]:
        if u not in CLOSED:  insert u into OPEN with key(u)
        else:                INCONS ← INCONS ∪ {u}

main:
    rhs[goal] ← 0;  insert goal;  sense();  compute_or_improve();  publish
    while robot ≠ goal:
        if ε > 1:                                           # 3
            ε ← max(1, ε − step);  reopen(OPEN ∪ INCONS);  compute_or_improve();  publish
        else:                                               # 4
            robot ← argmin over successors of (c + g);  sense()
            if new walls found:                             # 5
                k_m += h(previous robot, robot)
                update_state(each vertex next to a new wall)
                ε ← ε₀;  reopen();  compute_or_improve();  publish`}/>
            <T
                en={<ol>
                    <li>The asymmetric key is AD*'s signature: inflation applies only to
                        good-news (over-consistent) vertices; bad news travels at admissible
                        priority.</li>
                    <li>D* Lite's vertex update plus the ARA* refinement — inconsistent states
                        that were already expanded park in INCONS instead of re-entering OPEN.</li>
                    <li>While there is thinking time, behave like ARA*: tighten, reopen, repair,
                        publish a better-bounded plan.</li>
                    <li>Once the plan is belief-optimal, behave like D* Lite: one greedy step
                        down the cost field.</li>
                    <li>On a discovery, both machines at once: shift <InlineMath math="k_m"/>,
                        mark damage, and jump ε back up so a provisional plan arrives fast before
                        the schedule tightens it again.</li>
                </ol>}
                ko={<ol>
                    <li>비대칭 key가 AD*의 서명이다. 팽창은 좋은 소식(over-consistent)에만
                        적용되고, 나쁜 소식은 admissible한 우선순위로 이동한다.</li>
                    <li>D* Lite의 vertex 갱신에 ARA*식 보강을 더한 것. 이미 확장된 inconsistent
                        상태는 OPEN 대신 INCONS에 대기한다.</li>
                    <li>생각할 시간이 있는 동안은 ARA*처럼 군다. 조이고, reopen하고, 수리하고, 더
                        좋은 한계의 계획을 발표한다.</li>
                    <li>계획이 belief-최적이 되면 D* Lite처럼 군다. 비용장을 따라 greedy로 한
                        걸음.</li>
                    <li>발견이 있으면 두 장치가 동시에 돈다. <InlineMath math="k_m"/>을 밀고,
                        손상을 표시하고, ε을 도로 올려 임시 계획을 빨리 얻은 뒤 스케줄이 다시
                        조인다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs the same unknown-map problems as the D* Lite page. The number
                    to watch is "first solution after N expansions": with{" "}
                    <InlineMath math="\varepsilon_0 = 2.5"/> the first usable plan lands far
                    sooner than D* Lite's belief-optimal one, and the publication counter shows a
                    plan being re-issued around every discovery. The replay below is the
                    repository demo on the trap map — repair bursts and re-publications visible
                    between robot steps.
                </p>}
                ko={<p>
                    sandbox는 D* Lite 페이지와 같은 미지 지도 문제를 돈다. 볼 수치는 "첫 해까지
                    확장 N"이다. <InlineMath math="\varepsilon_0 = 2.5"/>면 첫 계획이 D* Lite의
                    belief-최적 계획보다 훨씬 먼저 나오고, 발표 카운터는 발견 때마다 계획이 다시
                    발행되는 것을 보여 준다. 아래 replay는 trap 맵에서의 저장소 demo다. 로봇 걸음
                    사이의 수리 파동과 재발표가 보인다.
                </p>}
            />
            <AdStarSandbox/>
            <TraceReplay algo="ad_star" maps={["dstar_trap01", "maze01"]} label={t(
                "Real traces from the repository's AD* demo — anytime re-publications layered on the D* Lite drive",
                "저장소 AD* demo의 실제 trace. D* Lite식 주행 위에 anytime 재발표가 겹쳐 보인다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation is the D* Lite loop with the asymmetric key, CLOSED/INCONS
                    bookkeeping, and the ε schedule woven in — embedded below in full.
                </p>}
                ko={<p>
                    구현은 D* Lite 루프에 비대칭 key, CLOSED/INCONS 관리, ε 스케줄을 짜 넣은
                    것이다. 전체를 아래에 embed했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/ad_star.py",
                            code: adPy,
                            href: `${REPO}/python/navigation/global_planning/search/ad_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/ad_star.cpp",
                            code: adCpp,
                            href: `${REPO}/cpp/src/global_planning/search/ad_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete AD* implementation, embedded from the repository sources",
                    "AD* 전체 구현. 저장소 소스를 그대로 embed한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    M. Likhachev, D. Ferguson, G. Gordon, A. Stentz, S. Thrun,{" "}
                    <a href="https://cdn.aaai.org/ICAPS/2005/ICAPS05-027.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>Anytime Dynamic A*: An Anytime, Replanning Algorithm</em>
                    </a>,
                    International Conference on Automated Planning and Scheduling (ICAPS), 2005.
                </li>
                <li>
                    S. Koenig, M. Likhachev,{" "}
                    <a href="https://cdn.aaai.org/AAAI/2002/AAAI02-072.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>D* Lite</em>
                    </a>,
                    AAAI Conference on Artificial Intelligence, 2002.
                </li>
                <li>
                    M. Likhachev, G. Gordon, S. Thrun,{" "}
                    <a href="https://www.cs.cmu.edu/~maxim/files/aranips03.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>ARA*: Anytime A* with Provable Bounds on Sub-Optimality</em>
                    </a>,
                    Advances in Neural Information Processing Systems (NIPS), 2003.
                </li>
            </ol>
        </>
    )
}

export default AdStar
