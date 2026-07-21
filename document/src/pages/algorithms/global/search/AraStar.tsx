import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import AraStarSandbox from "../../../../components/panels/global/ara_star/AraStarSandbox";
import araPy from "../../../../../../python/navigation/global_planning/search/ara_star.py?raw";
import araCpp from "../../../../../../cpp/src/global_planning/search/ara_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const AraStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    A planner on a robot rarely gets to say "wait until I'm done". Control loops
                    have deadlines, and the honest question is not "what is the optimal path?" but
                    "what is the best path you can hand me <em>right now</em> — and can you keep
                    improving it if I give you more time?" ARA* (Likhachev, Gordon &amp; Thrun,
                    2003) answers exactly that: a sequence of weighted-A* searches that publishes
                    a bounded-suboptimal path almost immediately, then tightens it toward the
                    optimum, reusing everything it has already computed.
                </p>}
                ko={<p>
                    로봇 위의 planner에게 "끝날 때까지 기다려 달라"는 말은 사치다. 제어 루프에는
                    마감이 있고, 정직한 질문은 "최적 경로가 무엇인가"가 아니라 "<em>지금 당장</em>{" "}
                    줄 수 있는 최선의 경로는 무엇이고, 시간을 더 주면 계속 개선할 수 있는가"다.
                    ARA*(Likhachev, Gordon &amp; Thrun, 2003)는 정확히 그 질문에 답한다. weighted
                    A* 탐색을 연달아 돌리며 준최적 보장이 있는 경로를 거의 즉시 발표하고, 이미
                    계산한 것을 전부 재사용하면서 최적을 향해 조여 간다.
                </p>}
            />

            <h2>{t("From Weighted A* to Anytime", "Weighted A*에서 Anytime으로")}</h2>
            <T
                en={<>
                    <p>
                        Recall the dial from the A* page: ordering the frontier by{" "}
                        <InlineMath math="f = g + \varepsilon h"/> with{" "}
                        <InlineMath math="\varepsilon > 1"/> makes the search greedier — far fewer
                        expansions, and the answer costs at most <InlineMath math="\varepsilon"/>{" "}
                        times the optimum. That suggests a schedule: search with a large{" "}
                        <InlineMath math="\varepsilon_0"/> to get a usable path fast, then rerun
                        with <InlineMath math="\varepsilon_0 - 0.5"/>, then lower, until{" "}
                        <InlineMath math="\varepsilon = 1"/> delivers the optimum. Runnable, but
                        wasteful: each rerun rediscovers nearly everything the previous one knew.
                    </p>
                    <p>
                        ARA*'s contribution is making the reruns share state. All iterations keep
                        one <InlineMath math="g"/> table and one search tree. When an iteration
                        finds a shorter route to a state it has <em>already expanded</em>, it does
                        not re-expand it now — the state is parked in a set called{" "}
                        <strong>INCONS</strong>. The next iteration starts its queue from
                        OPEN ∪ INCONS with keys recomputed under the smaller{" "}
                        <InlineMath math="\varepsilon"/>. Later iterations therefore expand only
                        where the tightened heuristic actually changes decisions, which is
                        typically a small fraction of the map.
                    </p>
                </>}
                ko={<>
                    <p>
                        A* 페이지의 다이얼을 떠올려 보라. frontier를{" "}
                        <InlineMath math="f = g + \varepsilon h"/>,{" "}
                        <InlineMath math="\varepsilon > 1"/>로 정렬하면 탐색이 greedy해져 확장이
                        크게 줄고, 답의 비용은 최적의 <InlineMath math="\varepsilon"/>배를 넘지
                        않는다. 그렇다면 스케줄이 자연스럽다. 큰{" "}
                        <InlineMath math="\varepsilon_0"/>로 먼저 쓸 만한 경로를 빨리 얻고,{" "}
                        <InlineMath math="\varepsilon_0 - 0.5"/>로, 또 그보다 작게, 결국{" "}
                        <InlineMath math="\varepsilon = 1"/>에서 최적을 얻을 때까지 다시 돌리는
                        것이다. 동작은 하지만 낭비다. 매 재실행이 직전 실행이 알던 것을 거의
                        전부 다시 발견한다.
                    </p>
                    <p>
                        ARA*의 기여는 그 재실행들이 상태를 공유하게 만든 것이다. 모든 반복이{" "}
                        <InlineMath math="g"/> 테이블 하나와 탐색 트리 하나를 같이 쓴다. 어떤
                        반복이 <em>이미 확장한</em> 상태로 가는 더 짧은 길을 찾으면 지금 재확장하지
                        않고 <strong>INCONS</strong>라는 집합에 담아 둔다. 다음 반복은
                        OPEN ∪ INCONS를 작아진 <InlineMath math="\varepsilon"/> 기준으로 키를 다시
                        계산해 큐를 연다. 그래서 뒤 반복들은 조여진 heuristic이 실제로 결정을
                        바꾸는 곳만 확장하고, 그것은 보통 지도의 작은 일부다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Anytime</strong>: a usable path appears after the first (cheap)
                        iteration; interrupt at any point and you hold the best path so far.</li>
                    <li><strong>Bounded suboptimality at every publication</strong>: the path
                        published at inflation <InlineMath math="\varepsilon"/> costs at most{" "}
                        <InlineMath math="\varepsilon \cdot C^*"/>; the final iteration
                        (<InlineMath math="\varepsilon = 1"/>) is optimal.</li>
                    <li><strong>Monotone improvement</strong>: published costs never increase
                        across iterations.</li>
                    <li><strong>Reuse</strong>: total work over the whole schedule is far below
                        running weighted A* from scratch per <InlineMath math="\varepsilon"/> —
                        the sandbox below puts numbers on it. Worst case still{" "}
                        <InlineMath math="O(b^d)"/>, like everything A*-shaped.</li>
                </ul>}
                ko={<ul>
                    <li><strong>Anytime</strong>: 값싼 첫 반복이 끝나는 즉시 쓸 만한 경로가
                        생기고, 어느 시점에 중단해도 그때까지의 최선 경로를 쥐고 있다.</li>
                    <li><strong>발표마다 준최적 한계 보장</strong>: 팽창{" "}
                        <InlineMath math="\varepsilon"/>에서 발표된 경로의 비용은 최대{" "}
                        <InlineMath math="\varepsilon \cdot C^*"/>다. 마지막 반복
                        (<InlineMath math="\varepsilon = 1"/>)은 최적이다.</li>
                    <li><strong>단조 개선</strong>: 발표되는 비용은 반복이 진행되며 절대 늘지
                        않는다.</li>
                    <li><strong>재사용</strong>: 스케줄 전체의 총 작업량이{" "}
                        <InlineMath math="\varepsilon"/>마다 weighted A*를 처음부터 돌리는 것보다
                        훨씬 적다. 아래 sandbox가 그 수치를 보여 준다. 최악의 경우는 여전히{" "}
                        <InlineMath math="O(b^d)"/>로, A* 계열의 숙명이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One search state (<InlineMath math="g"/>, parents, OPEN, CLOSED, INCONS)
                    threads through every iteration. <code>improve_path</code> is weighted A* with
                    two twists: its stopping test compares the goal against the queue's best key,
                    and improved-but-closed states go to INCONS instead of the queue:
                </p>}
                ko={<p>
                    탐색 상태 한 벌(<InlineMath math="g"/>, 부모, OPEN, CLOSED, INCONS)이 모든
                    반복을 관통한다. <code>improve_path</code>는 weighted A*에 두 가지 변형을 더한
                    것이다. 종료 검사가 goal을 큐의 최소 키와 비교하고, 개선됐지만 이미 닫힌
                    상태는 큐 대신 INCONS로 보낸다:
                </p>}
            />
            <Pseudocode code={`improve_path(ε):
    while OPEN not empty and g[goal] > min key in OPEN:            # 1
        s ← pop_min(OPEN);  CLOSED ← CLOSED ∪ {s}
        for each neighbor n' with edge cost c(s, n'):
            if g[s] + c(s, n') < g[n']:
                g[n'] ← g[s] + c(s, n');  parent[n'] ← s
                if n' not in CLOSED:
                    push n' into OPEN with key g[n'] + ε·h(n')
                else:
                    INCONS ← INCONS ∪ {n'}                         # 2

main:
    ε ← ε₀;  g[start] ← 0;  push start
    loop:
        improve_path(ε)
        publish path to goal                                       # 3
        if ε = 1: stop
        ε ← max(1, ε − step)
        OPEN ← OPEN ∪ INCONS, re-keyed with new ε;  INCONS ← ∅     # 4
        CLOSED ← ∅`}/>
            <T
                en={<ol>
                    <li>Stop as soon as the goal's cost is no worse than the best key in OPEN —
                        expanding further cannot help this iteration, and this test is exactly
                        what makes the <InlineMath math="\varepsilon"/>-bound provable.</li>
                    <li>The core trick: an improvement that lands on an already-expanded state is
                        deferred. Re-expanding it now would cascade; the next iteration will
                        handle it with a tighter <InlineMath math="\varepsilon"/> anyway.</li>
                    <li>Each publication comes with the guarantee "within{" "}
                        <InlineMath math="\varepsilon"/> of optimal" — the caller can act on it
                        or wait for the next one.</li>
                    <li>Between iterations, nothing is thrown away: the frontier plus the deferred
                        states become the new frontier, re-prioritized under the smaller{" "}
                        <InlineMath math="\varepsilon"/>. Clearing CLOSED permits re-expansion
                        where the improvements demand it.</li>
                </ol>}
                ko={<ol>
                    <li>goal의 비용이 OPEN의 최소 키 이하가 되는 즉시 멈춘다. 더 확장해도 이번
                        반복에는 도움이 안 되고, 이 검사가 바로{" "}
                        <InlineMath math="\varepsilon"/>-한계를 증명 가능하게 만드는 장치다.</li>
                    <li>핵심 트릭. 이미 확장된 상태에 떨어진 개선은 미룬다. 지금 재확장하면
                        연쇄가 일어나는데, 어차피 다음 반복이 더 조여진{" "}
                        <InlineMath math="\varepsilon"/>으로 처리한다.</li>
                    <li>발표마다 "최적의 <InlineMath math="\varepsilon"/>배 이내"라는 보장이
                        붙는다. 호출자는 그것으로 움직여도 되고 다음 발표를 기다려도 된다.</li>
                    <li>반복 사이에 버리는 것이 없다. frontier와 미뤄 둔 상태들이 작아진{" "}
                        <InlineMath math="\varepsilon"/> 기준으로 재정렬되어 새 frontier가 된다.
                        CLOSED를 비우는 것은 개선이 요구하는 곳의 재확장을 허용하기 위해서다.</li>
                </ol>}
            />

            <h2>{t("Why the Bound Holds", "한계가 성립하는 이유")}</h2>
            <T
                en={<p>
                    The publication guarantee needs no global argument — it falls out of the
                    stopping test and heuristic admissibility in three lines.
                </p>}
                ko={<p>
                    발표 보장에는 전역 논증이 필요 없다. 종료 검사와 heuristic의 admissibility에서
                    세 줄로 떨어진다.
                </p>}
            />
            <Proof title={t("Theorem (ε-suboptimality at publication)", "정리 (발표 시점의 ε-준최적성)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> <InlineMath math="h"/> admissible. When{" "}
                            <code>improve_path(ε)</code> stops, every queued state{" "}
                            <InlineMath math="s"/> satisfies{" "}
                            <InlineMath math="g(\text{goal}) \le g(s) + \varepsilon h(s)"/>. Take
                            any optimal path and let <InlineMath math="s"/> be its first state
                            still in OPEN (one always exists, with{" "}
                            <InlineMath math="g(s) = g^*(s)"/>):
                        </p>
                        <BlockMath math="g(\text{goal}) \;\le\; g^*(s) + \varepsilon h(s) \;\overset{\text{admissible}}{\le}\; \varepsilon\bigl(g^*(s) + h^*(s)\bigr) \;=\; \varepsilon\, C^*"/>
                        <Terms items={[
                            ["\\varepsilon", <>the current heuristic inflation (<InlineMath math="\\varepsilon \\ge 1"/>); the frontier is ordered by <InlineMath math="g + \\varepsilon h"/></>],
                            ["g(\\text{goal})", "cost of the path being published"],
                            ["g^*(s),\\ h^*(s)", <>true optimal cost start→<InlineMath math="s"/> and true remaining cost <InlineMath math="s"/>→goal; admissibility gives <InlineMath math="h \\le h^*"/></>],
                            ["C^*", <>optimal start→goal cost, <InlineMath math="g^*(s) + h^*(s)"/> along the optimal path</>],
                        ]}/>
                        <p>
                            Publishing at inflation <InlineMath math="\varepsilon"/> therefore
                            guarantees cost at most <InlineMath math="\varepsilon C^*"/>.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> <InlineMath math="h"/>는 admissible.{" "}
                            <code>improve_path(ε)</code>가 멈추면 큐의 모든 상태{" "}
                            <InlineMath math="s"/>가{" "}
                            <InlineMath math="g(\text{goal}) \le g(s) + \varepsilon h(s)"/>를
                            만족한다. 임의의 최적 경로에서 아직 OPEN에 남아 있는 첫 상태를{" "}
                            <InlineMath math="s"/>라 하면(항상 존재하고{" "}
                            <InlineMath math="g(s) = g^*(s)"/>다):
                        </p>
                        <BlockMath math="g(\text{goal}) \;\le\; g^*(s) + \varepsilon h(s) \;\overset{\text{admissible}}{\le}\; \varepsilon\bigl(g^*(s) + h^*(s)\bigr) \;=\; \varepsilon\, C^*"/>
                        <Terms items={[
                            ["\\varepsilon", <>현재의 heuristic 팽창 계수 (<InlineMath math="\\varepsilon \\ge 1"/>). frontier는 <InlineMath math="g + \\varepsilon h"/>로 정렬된다</>],
                            ["g(\\text{goal})", "지금 발표하는 경로의 비용"],
                            ["g^*(s),\\ h^*(s)", <>시작→<InlineMath math="s"/> 참 최적 비용과 <InlineMath math="s"/>→목표 참 잔여 비용. admissible 이란 <InlineMath math="h \\le h^*"/>라는 뜻</>],
                            ["C^*", <>최적 시작→목표 비용. 최적 경로 위에서 <InlineMath math="g^*(s) + h^*(s)"/>와 같다</>],
                        ]}/>
                        <p>
                            따라서 팽창 <InlineMath math="\varepsilon"/>에서 발표한 경로의 비용은
                            최대 <InlineMath math="\varepsilon C^*"/>다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    Play the sandbox and watch the red path get published, then shorten as{" "}
                    <InlineMath math="\varepsilon"/> tightens (the readout counts solutions). The
                    per-iteration counters tell the reuse story: the first iteration does nearly
                    all the expanding, and later ones add a handful of nodes each — compare the
                    total against rerunning weighted A* from scratch at every ε. A larger{" "}
                    <InlineMath math="\varepsilon_0"/> makes the first answer faster and the gap
                    bigger.
                </p>}
                ko={<p>
                    sandbox를 재생하면 빨간 경로가 발표된 뒤 <InlineMath math="\varepsilon"/>이
                    조여지며 짧아지는 것이 보인다(readout이 해 개수를 센다). 반복별 카운터가
                    재사용의 핵심을 말해 준다. 확장은 첫 반복이 거의 다 하고, 뒤 반복들은 각각 몇
                    개씩만 더한다. 그 합을 매 ε마다 weighted A*를 처음부터 돌린 값과 비교해 보라.{" "}
                    <InlineMath math="\varepsilon_0"/>를 키우면 첫 답이 더 빨라지고 격차도
                    커진다.
                </p>}
            />
            <AraStarSandbox/>
            <TraceReplay algo="ara_star" maps={["wastar_greedy01", "maze01"]} label={t(
                "Real traces from the repository's ARA* demo — successive publications visible as the path snaps to shorter routes",
                "저장소 ARA* demo의 실제 trace. 경로가 더 짧은 길로 바뀌는 순간들이 연속 발표로 보인다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation is a single search state shared across iterations —{" "}
                    <code>improve_path</code> plus the ε-schedule loop, exactly as in the
                    pseudocode. Embedded below from the repository sources.
                </p>}
                ko={<p>
                    구현은 반복들이 공유하는 탐색 상태 한 벌이다. <code>improve_path</code>와
                    ε-스케줄 루프가 수도코드 그대로 들어 있다. 저장소 소스를 아래에 embed했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/ara_star.py",
                            code: araPy,
                            href: `${REPO}/python/navigation/global_planning/search/ara_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/ara_star.cpp",
                            code: araCpp,
                            href: `${REPO}/cpp/src/global_planning/search/ara_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete ARA* implementation, embedded from the repository sources",
                    "ARA* 전체 구현. 저장소 소스를 그대로 embed한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    M. Likhachev, G. Gordon, S. Thrun,{" "}
                    <a href="https://www.cs.cmu.edu/~maxim/files/aranips03.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>ARA*: Anytime A* with Provable Bounds on Sub-Optimality</em>
                    </a>,
                    Advances in Neural Information Processing Systems (NIPS), 2003.
                </li>
                <li>
                    I. Pohl,{" "}
                    <a href="https://doi.org/10.1016/0004-3702(70)90007-X" target="_blank"
                       rel="noopener noreferrer">
                        <em>Heuristic Search Viewed as Path Finding in a Graph</em>
                    </a>,
                    Artificial Intelligence, 1970.
                </li>
                <li>
                    E. A. Hansen, R. Zhou,{" "}
                    <a href="https://doi.org/10.1613/jair.2096" target="_blank"
                       rel="noopener noreferrer">
                        <em>Anytime Heuristic Search</em>
                    </a>,
                    Journal of Artificial Intelligence Research, 2007.
                </li>
            </ol>
        </>
    )
}

export default AraStar
