import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import LpaStarSandbox from "../../../../components/panels/global/lpa_star/LpaStarSandbox";
import lpaPy from "../../../../../../python/navigation/global_planning/search/lpa_star.py?raw";
import lpaCpp from "../../../../../../cpp/src/global_planning/search/lpa_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const LpaStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    A* assumes the map is known and fixed. D* Lite handled the case where the
                    map is unknown. LPA* (Koenig &amp; Likhachev, 2002; journal version Koenig,
                    Likhachev &amp; Furcy, 2004) handles the mirror setting: the map is fully
                    known from round 0 — what changes are the costs themselves. A door closes, a
                    corridor opens, and the planner is <em>told</em> about it. Re-running A* on
                    every change throws away an entire search that was mostly still correct.
                    LPA* keeps its g/rhs estimates alive between planning runs and repairs only
                    the region the change actually invalidates.
                </p>}
                ko={<p>
                    A*는 지도가 알려져 있고 고정되어 있다고 가정했다. D* Lite는 지도를 모르는
                    경우를 다뤘다. LPA*(Koenig &amp; Likhachev, 2002, 저널 버전은 Koenig,
                    Likhachev &amp; Furcy, 2004)은 그 반대편 설정을 다룬다. 지도는 round 0부터
                    완전히 알려져 있고, 바뀌는 것은 cost 자체다. 문이 닫히고 통로가 열리고,
                    planner는 그것을 <em>알려받는다</em>. 변경마다 A*를 다시 돌린다면 대부분
                    여전히 옳았던 탐색 전체를 버리는 일이다. LPA*는 g/rhs 추정치를 계획 실행
                    사이에 살려 두고, 변경이 실제로 무효화한 영역만 수리한다.
                </p>}
            />

            <h2>{t("When the Known Map Changes", "알고 있는 지도가 바뀔 때")}</h2>
            <T
                en={<>
                    <p>
                        The setting: a pre-mapped facility. The robot knows every wall from the
                        first round — there is nothing to sense. What changes is occupancy itself,
                        communicated from outside: a warehouse aisle closes for maintenance, a
                        door opens. The naive strategy works — re-run A* on the new map — but one
                        changed cell invalidates only the part of the previous search whose
                        estimates passed through it. Everything else was already correct, and
                        recomputing it is pure waste.
                    </p>
                    <p>
                        The observation that makes repair possible: a cost change touches{" "}
                        <em>one cell's</em> incident edges, so only the affected vertices' lookahead
                        changes. LPA* keeps exactly those vertices in a queue and re-propagates
                        from them. And unlike D* Lite, both endpoints are fixed — this planner
                        never moves its root. Re-rooting is D* Lite's job; here the search runs{" "}
                        <em>forward</em> from the start toward a goal that never moves.
                    </p>
                </>}
                ko={<>
                    <p>
                        설정은 이렇다. 미리 지도를 그려 둔 시설. 로봇은 첫 라운드부터 모든 벽을
                        알고 있고, 감지할 것은 아무것도 없다. 바뀌는 것은 점유 자체이고 외부에서
                        전달된다. 창고 통로가 정비로 막히고 문이 열린다. 순진한 전략도 동작은 한다.
                        바뀐 지도로 A*를 다시 돌리는 것이다. 다만 셀 하나가 바뀐 사건은 이전 탐색
                        중 <em>그 셀을 지나간 추정치</em>만 무효화한다. 나머지는 이미 옳았는데
                        그것을 다시 계산하는 것은 순수한 낭비다.
                    </p>
                    <p>
                        수리를 가능하게 하는 관찰: cost 변경은 <em>한 셀</em>의 인접 간선에만
                        닿고, 그래서 영향받은 vertex의 lookahead만 바뀐다. LPA*는 정확히 그
                        vertex들을 큐에 담아 거기서 다시 전파한다. 그리고 D* Lite와 달리 양끝이
                        고정이다. 이 planner는 탐색 뿌리를 옮기지 않는다. 뿌리를 옮기는 일은 D*
                        Lite의 몫이고, 여기서는 탐색이 start에서 goal로 <em>앞으로</em>
                        진행하며 goal은 절대 움직이지 않는다.
                    </p>
                </>}
            />

            <h2>{t("Forward Search and the g/rhs Pair", "Forward 탐색과 g/rhs")}</h2>
            <T
                en={<>
                    <p>
                        Two design choices carry the whole algorithm, and they are exact mirrors
                        of D* Lite's.
                    </p>
                    <p>
                        <strong>Search forward from the fixed start.</strong>{" "}
                        <InlineMath math="g(s)"/> estimates the cost <em>from the start</em> to{" "}
                        <InlineMath math="s"/>, and the lookahead reads predecessors instead of
                        successors:
                    </p>
                </>}
                ko={<>
                    <p>
                        설계 선택 두 개가 알고리즘 전체를 떠받치고, 둘 다 D* Lite의 정확한
                        거울상이다.
                    </p>
                    <p>
                        <strong>고정된 start에서 앞으로 탐색한다.</strong>{" "}
                        <InlineMath math="g(s)"/>는 <em>start</em>에서 <InlineMath math="s"/>까지
                        비용의 추정치이고, lookahead는 successor가 아니라 predecessor를 읽는다.
                    </p>
                </>}
            />
            <BlockMath math="rhs(s) \;=\; \min_{p \in pred(s)}\bigl(c(p, s) + g(p)\bigr), \qquad rhs(\text{start}) = 0"/>
            <Terms items={[
                ["g(s)", <>시작점에서 <InlineMath math="s"/>까지 비용의 확정 추정치 (탐색이 forward라 g는 goal까지가 아니라 시작점부터의 비용이다)</>],
                ["rhs(s)", <><strong>새로 추가된 항</strong>: one-step lookahead. predecessor를 거치는 최선 비용으로, 간선이 바뀌면 즉시 다시 계산된다</>],
                ["pred(s)", <><InlineMath math="s"/>의 grid 이웃 (격자는 무향이라 successor와 predecessor가 같다)</>],
                ["c(p, s)", <>간선 비용 (셀이 막히면 ∞)</>],
            ]}/>
            <T
                en={<>
                    <p>
                        A vertex is <em>consistent</em> when <InlineMath math="g(s) = rhs(s)"/>.
                        When a cell flips, the affected vertices' <InlineMath math="rhs"/> changes
                        immediately (it is a local minimum over neighbors), they become
                        inconsistent, and only inconsistent vertices queue for repair. The second
                        choice: the heuristic's reference is the <em>fixed goal</em>, so keys stay
                        comparable across replans without any offset — D* Lite needed{" "}
                        <InlineMath math="k_m"/> only because its search root moved with the
                        robot. Here nothing moves, and no bookkeeping has to compensate for it.
                    </p>
                </>}
                ko={<>
                    <p>
                        <InlineMath math="g(s) = rhs(s)"/>인 vertex를 <em>consistent</em>하다고
                        한다. 셀이 뒤집히면 영향받은 vertex의 <InlineMath math="rhs"/>가 즉시
                        바뀌고(이웃에 대한 국소 최솟값이므로), 그들은 inconsistent가 되며,{" "}
                        <InlineMath math="U"/>에는 inconsistent한 vertex만 들어간다. 두 번째 선택:
                        heuristic의 기준점이 <em>고정된 goal</em>이므로 키는 replan 내내 비교
                        가능하고, 오프셋이 아예 필요 없다. D* Lite가 <InlineMath math="k_m"/>을
                        둔 이유는 탐색 뿌리가 로봇과 함께 움직였기 때문이고, 여기서는 아무것도
                        움직이지 않으므로 보상 장부도 필요 없다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Exact per round</strong>: when a repair burst ends,{" "}
                        <InlineMath math="g(\text{goal})"/> is exactly the shortest distance under
                        the current map — the same answer A* would give if restarted, with no
                        approximation anywhere.</li>
                    <li><strong>Incremental</strong>: repair cost scales with the region the flip
                        actually influences. Worst case (a change that invalidates everything)
                        degrades to a full A*; typical single-cell flips touch a small patch.</li>
                    <li><strong>The map is known</strong>: this is what separates LPA* from D* Lite
                        in kind, not degree. Costs are communicated, never sensed — so there is no
                        belief, and every round's answer is optimal for the real map, not just for
                        what has been discovered.</li>
                    <li><strong>Memory</strong> <InlineMath math="O(V)"/> for{" "}
                        <InlineMath math="g/rhs"/>; the queue holds only inconsistent vertices.</li>
                </ul>}
                ko={<ul>
                    <li><strong>라운드마다 정확하다</strong>: 수리 burst가 끝나면{" "}
                        <InlineMath math="g(\text{goal})"/>은 현재 지도에서의 최단 거리와 정확히
                        같다. A*를 다시 돌려도 나오는 그 답이고, 근사는 어디에도 없다.</li>
                    <li><strong>Incremental</strong>: 수리 비용은 반전이 실제로 영향을 준 영역에
                        비례한다. 최악의 경우(모든 것을 무효화하는 변경)는 A* 전체 재실행으로
                        퇴화하지만, 보통의 셀 하나 반전은 작은 조각만 건드린다.</li>
                    <li><strong>지도는 알려져 있다</strong>: 이것이 LPA*를 D* Lite와 정도가 아니라
                        종류로 가르는 지점이다. cost는 감지하는 것이 아니라 전달받고, 그래서 belief가
                        없고, 모든 라운드의 답은 발견된 것에 대한 최적이지 실제 지도에 대한 최적이다.</li>
                    <li><strong>메모리</strong>는 <InlineMath math="g/rhs"/>에 <InlineMath math="O(V)"/>.
                        큐에는 inconsistent한 vertex만 들어간다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The state is the pair <InlineMath math="g/rhs"/>, a priority queue{" "}
                    <InlineMath math="U"/> of inconsistent vertices, and the fixed endpoints. Three
                    routines cooperate: a key that orders repairs, a vertex update that detects
                    inconsistency, and a bounded best-first pass that restores consistency toward
                    the goal. The main loop is just “wait for cost changes, UpdateVertex, repair”:
                </p>}
                ko={<p>
                    상태는 <InlineMath math="g/rhs"/> 쌍, inconsistent vertex를 담는 우선순위 큐{" "}
                    <InlineMath math="U"/>, 그리고 고정된 양끝점이다. 루틴 세 개가 맞물린다. 수리
                    순서를 정하는 key, inconsistency를 감지하는 vertex 갱신, goal 방향으로
                    consistency를 복원하는 한정된 best-first 패스. 메인 루프는 그냥 “cost 변경을
                    기다리고, UpdateVertex 하고, 수리한다”다:
                </p>}
            />
            <Pseudocode code={`key(s) = [min(g[s], rhs[s]) + h(goal, s),  min(g[s], rhs[s])]      # 1

update_vertex(u):                                                            # 2
    if u ≠ start:  rhs[u] ← min over predecessors p of (c(p, u) + g[p])
    remove u from U
    if g[u] ≠ rhs[u]:  insert u into U with key(u)

compute_shortest_path():                                                     # 3
    while top_key(U) < key(goal)  or  rhs[goal] ≠ g[goal]:
        u ← pop_min(U)
        if k_old < key(u):   re-insert u with the new key   (dead code here — see 5)
        else if g[u] > rhs[u]:  g[u] ← rhs[u];  update_vertex(each successor of u)
        else:                   g[u] ← ∞;        update_vertex(u and each successor)

main:                                                                        # 4
    seed the model from the known map;  rhs[start] ← 0;  insert start
    compute_shortest_path()                                                  # 5
    forever, on each communicated flip of cell u:                            # 6
        update_vertex(u);  update_vertex(each passable neighbour of u)
        compute_shortest_path()`}/>
            <T
                en={<ol>
                    <li>The key mirrors A*'s <InlineMath math="f"/>: estimated total cost through{" "}
                        <InlineMath math="s"/>, heuristic measured to the fixed <em>goal</em>. The
                        reference never moves, so keys computed before a change stay comparable
                        after it — no offset term exists.</li>
                    <li>Recompute the one-step lookahead over passable predecessors. If it
                        disagrees with <InlineMath math="g"/>, the vertex queues for repair; if
                        they agree, it leaves the queue. A newly blocked cell is popped and never
                        re-enters while blocked — its stale finite values stay stored but
                        unreachable.</li>
                    <li>Expand in key order until the goal itself is locally consistent{" "}
                        <em>and</em> no live key undercuts <InlineMath math="\text{key}(\text{goal})"/>.
                        An over-consistent vertex (<InlineMath math="g > rhs"/>) accepts the better
                        value; an under-consistent one is reset to <InlineMath math="\infty"/> first
                        so the raise propagates.</li>
                    <li>Initialize once: every <InlineMath math="g"/> starts infinite, the start's{" "}
                        <InlineMath math="rhs"/> pins to 0, and the model seeds from the known map.
                        Later rounds skip all of this — that is the whole point.</li>
                    <li>The key-invariant branch (re-inserting a popped vertex under its new key)
                        is dead code in this variant: without an offset, a popped key can only be
                        stale from the pop itself. It stays verbatim from the paper's pseudocode,
                        provably inert.</li>
                    <li>A communicated flip flips the cell in the model and UpdateVertex's exactly
                        that cell and its passable neighbours — the vertices whose predecessor set
                        actually changed — then re-runs the bounded best-first pass. A no-op flip
                        (already in that state) changes no edge cost, so it updates nothing.</li>
                </ol>}
                ko={<ol>
                    <li>key는 A*의 <InlineMath math="f"/>와 같은 꼴이다. <InlineMath math="s"/>를
                        지나는 총 비용 추정에, heuristic은 고정된 <em>goal</em>까지 재고. 기준점이
                        움직이지 않으니 변경 전에 계산된 키가 변경 후에도 비교 가능하고,{" "}
                        <InlineMath math="k_m"/> 같은 오프셋 항은 아예 존재하지 않는다.</li>
                    <li>passable predecessor에 대해 one-step lookahead를 다시 계산한다.{" "}
                        <InlineMath math="g"/>와 다르면 수리 대기열에 넣고, 같으면 큐에서 뺀다.
                        막힌 셀로 바뀌면 큐에서 빠지고, 막혀 있는 동안 다시 들어가지 않는다. 낡은
                        유한값은 저장된 채로 도달 불가가 된다.</li>
                    <li>goal 자체가 locally consistent하고 <InlineMath math="\text{key}(\text{goal})"/>을
                        밑도는 살아 있는 키가 남지 않을 때까지 key 순서로 확장한다. over-consistent
                        (<InlineMath math="g > rhs"/>)면 더 나은 값을 받아들이고, under-consistent면
                        먼저 <InlineMath math="\infty"/>로 올려서 인상이 전파되게 한다.</li>
                    <li>초기화는 한 번뿐이다. 모든 <InlineMath math="g"/>는 ∞로 시작하고 start의{" "}
                        <InlineMath math="rhs"/>만 0에 고정되고, 모델은 알려진 지도에서 시드된다.
                        이후 라운드는 이 전부를 건너뛴다. 그게 전부다.</li>
                    <li>key-invariant 분기(popped를 새 key로 재삽입하는 것)는 이 변형에서 dead
                        code다. 오프셋이 없으니 popped key는 pop 자체로 낡은 것일 수뿐 없다. 논문
                        pseudocode 그대로 두고 있고, 증명된 상태로 무해하다.</li>
                    <li>전달된 반전은 모델에서 셀을 뒤집고, predecessor 집합이 실제로 바뀐
                        정확히 그 셀과 통과 가능 이웃들만 UpdateVertex한 뒤 한정된 best-first
                        패스를 다시 돌린다. no-op 반전(이미 그 상태)은 간선 비용을
                        바꾸지 않으니 아무것도 갱신하지 않는다.</li>
                </ol>}
            />

            <h2>{t("Why the Repaired g Is Optimal", "수리된 g가 최적인 이유")}</h2>
            <Proof title={t("Invariant (queue = inconsistent set)", "불변식 (큐 = inconsistent 집합)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> The queue holds exactly the inconsistent
                            vertices, and every mutation of <InlineMath math="g"/> or{" "}
                            <InlineMath math="rhs"/> passes through <code>update_vertex</code>,
                            which ends by restoring:
                        </p>
                        <BlockMath math="u \in U \;\Longleftrightarrow\; g(u) \ne rhs(u)"/>
                        <Terms items={[
                            ["U", "priority queue. inconsistent한 vertex 정확히 그들만 들어 있다"],
                            ["g(u),\ rhs(u)", <>start부터 비용의 확정 추정치 vs one-step lookahead. 같으면 <InlineMath math="u"/>는 consistent (확정이고 올바름)</>],
                        ]}/>
                        <p>
                            <strong>Claim.</strong> When a burst ends, <InlineMath math="g(\text{goal})"/>{" "}
                            equals the true shortest distance <InlineMath math="d^{*}"/> under the
                            current model. Suppose not:{" "}
                            <InlineMath math="g(\text{goal}) > d^{*}"/> on some true shortest path{" "}
                            <InlineMath math="P = v_0, \dots, v_k"/> (<InlineMath math="v_0 = \text{start}"/>,{" "}
                            <InlineMath math="v_k = \text{goal}"/>).
                        </p>
                        <p>
                            Induct along <InlineMath math="P"/>. <InlineMath math="v_1"/> cannot be
                            inconsistent at termination: it would sit live in <InlineMath math="U"/>{" "}
                            with key
                        </p>
                        <BlockMath math="\min\bigl(g(v_1), rhs(v_1)\bigr) + h(v_1, \text{goal}) \;\le\; rhs(v_1) + h(v_1, \text{goal}) \;\le\; \bigl(c(\text{start}, v_1) + g(\text{start})\bigr) + d^{*}(v_1 \to \text{goal}) = d^{*} < g(\text{goal})"/>
                        <Terms items={[
                            ["h(v_1,\ \text{goal})", <>goal까지의 octile heuristic. 최단 거리에 대한 허용 가능(admissible)한 underestimate</>],
                            ["d^{*}(\cdot \to \text{goal})", <>현재 모델에서의 참 최단 거리. P가 최단 경로라 접두부는 최적이고, 그래서 등호가 성립한다</>],
                        ]}/>
                        <p>
                            — a live key strictly under <InlineMath math="\text{key}(\text{goal}) = (g(\text{goal}), g(\text{goal}))"/>,
                            contradicting termination. So every vertex of <InlineMath math="P"/> is
                            consistent, each hop satisfies{" "}
                            <InlineMath math="c(v_{i-1}, v_i) + g(v_{i-1}) = g(v_i)"/>, and the
                            equalities telescope to <InlineMath math="g(\text{goal}) \le \operatorname{cost}(P) = d^{*}"/>.
                            The extracted path has cost exactly <InlineMath math="g(\text{goal})"/>,
                            so both sides are equal — and the same chain shows the extraction walk
                            can never get stuck. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 큐에는 inconsistent vertex 정확히 그들만 들어
                            있고, <InlineMath math="g"/>나 <InlineMath math="rhs"/>의 모든 변경은{" "}
                            <code>update_vertex</code>를 거치고, 그 끝은 항상 다음을 복원한다:
                        </p>
                        <BlockMath math="u \in U \;\Longleftrightarrow\; g(u) \ne rhs(u)"/>
                        <Terms items={[
                            ["U", "priority queue. inconsistent한 vertex 정확히 그들만 들어 있다"],
                            ["g(u),\ rhs(u)", <>start부터 비용의 확정 추정치 vs one-step lookahead. 같으면 <InlineMath math="u"/>는 consistent (확정이고 올바름)</>],
                        ]}/>
                        <p>
                            <strong>주장.</strong> burst가 끝나는 순간 <InlineMath math="g(\text{goal})"/>은
                            현재 모델에서의 참 최단 거리 <InlineMath math="d^{*}"/>와 같다. 반대로{" "}
                            <InlineMath math="g(\text{goal}) > d^{*}"/>라고 하자. 어떤 참 최단 경로{" "}
                            <InlineMath math="P = v_0, \dots, v_k"/>(<InlineMath math="v_0 = \text{start}"/>,{" "}
                            <InlineMath math="v_k = \text{goal}"/>)에 대해 그렇다는 가정이다.
                        </p>
                        <p>
                            <InlineMath math="P"/>를 따라 귀납한다. <InlineMath math="v_1"/>이 종료
                            시점에 inconsistent할 수 없다. 그러면 key가
                        </p>
                        <BlockMath math="\min\bigl(g(v_1), rhs(v_1)\bigr) + h(v_1, \text{goal}) \;\le\; rhs(v_1) + h(v_1, \text{goal}) \;\le\; \bigl(c(\text{start}, v_1) + g(\text{start})\bigr) + d^{*}(v_1 \to \text{goal}) = d^{*} < g(\text{goal})"/>
                        <Terms items={[
                            ["h(v_1,\ \text{goal})", <>goal까지 octile heuristic. 최단 거리에 대한 허용 가능(admissible)한 underestimate</>],
                            ["d^{*}(\cdot \to \text{goal})", <>현재 모델의 참 최단 거리. P가 최단 경로라 접두부가 최적이고, 그래서 등호가 성립한다</>],
                        ]}/>
                        <p>
                            을 가져서 <InlineMath math="\text{key}(\text{goal}) = (g(\text{goal}), g(\text{goal}))"/>을
                            엄밀히 밑도는 살아 있는 키가 남아 종료 조건에 모순이다. 그래서{" "}
                            <InlineMath math="P"/>의 모든 vertex가 consistent하고, 각 hop이{" "}
                            <InlineMath math="c(v_{i-1}, v_i) + g(v_{i-1}) = g(v_i)"/>를 만족하고,
                            등식들이 telescope되어 <InlineMath math="g(\text{goal}) \le \operatorname{cost}(P) = d^{*}"/>.
                            추출된 경로의 비용은 정확히 <InlineMath math="g(\text{goal})"/>이므로
                            양변이 같아지고, 같은 사슬은 추출 walk이 막힐 수 없음을도 보인다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    In the sandbox the map is fully known from round 0 — there are no ghosts. Click
                    a cell to <em>communicate</em> its cost change: the wall appears at its event,
                    the repair burst expands locally around it, and the path re-routes through the
                    other gate. Flip the same cell back and the straight crossing returns with a
                    tiny burst. The counter compares LPA*'s cumulative repair expansions against
                    rerunning A* from scratch on every change — on this map, one cell at a time is
                    orders of magnitude cheaper.
                </p>}
                ko={<p>
                    sandbox에서 지도는 round 0부터 전부 알려져 있고 흐린 벽도 없다. 셀을 클릭하면
                    그 셀의 cost 변경이 <em>전달</em>된다. 벽은 자기 이벤트 시점에 나타나고, 수리
                    burst는 그 주변을 국소적으로 확장하고, 경로는 다른 게이트로 다시 잡힌다. 같은
                    셀을 되돌리면 tiny burst와 함께 직선 통행이 돌아온다. 카운터는 LPA*의 누적 수리
                    확장 수를 변경마다 A*를 처음부터 돌렸을 때와 비교한다. 이 지도에서는 한 번에
                    셀 하나가 배수 단위로 싸다.
                </p>}
            />
            <LpaStarSandbox/>
            <TraceReplay algo="lpa_star" maps={["lpa_replan01"]} label={t(
                "Real trace from the repository's LPA* demo: round 0 crosses through the open gate, revision 1 walls it off and the repair re-routes over the top, revision 2 frees the cell back",
                "저장소 LPA* demo의 실제 trace. round 0은 열린 게이트로 횡단하고, revision 1이 그 통로를 막자 수리가 위쪽 게이트로 경로를 다시 잡고, revision 2가 셀을 되돌린다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    Unlike Dijkstra and A*, LPA* does not ride the shared best-first core — state
                    survives across plan() calls, the model is seeded from ground truth once, and
                    cost changes arrive through an explicit revision channel. The full implementation
                    is embedded below; both languages are bit-identical move for move, so a demo
                    trace replays identically everywhere.
                </p>}
                ko={<p>
                    Dijkstra, A*와 달리 LPA*는 공유 best-first 코어를 쓰지 않는다. 상태가 plan()
                    호출 사이에 살아남고, 모델은 ground truth에서 한 번 시드되고, cost 변경은 명시적
                    revision 채널로 도착한다. 전체 구현을 아래에 embed했다. 두 언어는 move 단위로
                    bit-identical이라 demo trace가 어디서나 동일하게 재생된다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/lpa_star.py",
                            code: lpaPy,
                            href: `${REPO}/python/navigation/global_planning/search/lpa_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/lpa_star.cpp",
                            code: lpaCpp,
                            href: `${REPO}/cpp/src/global_planning/search/lpa_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete LPA* implementation, embedded from the repository sources",
                    "LPA* 전체 구현. 저장소 소스를 그대로 embed한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. Koenig, M. Likhachev,{" "}
                    <a href="https://cdn.aaai.org/AAAI/2002/AAAI02-072.pdf" target="_blank"
                       rel="noopener noreferrer">
                        <em>D* Lite</em>
                    </a>,
                    AAAI Conference on Artificial Intelligence, 2002.
                </li>
                <li>
                    S. Koenig, M. Likhachev, D. Furcy,{" "}
                    <a href="https://doi.org/10.1016/j.artint.2003.12.001" target="_blank"
                       rel="noopener noreferrer">
                        <em>Lifelong Planning A*</em>
                    </a>,
                    Artificial Intelligence, 2004.
                </li>
            </ol>
        </>
    )
}

export default LpaStar
