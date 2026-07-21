import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import DStarLiteSandbox from "../../../../components/panels/global/dstar_lite/DStarLiteSandbox";
import BackwardField from "../../../../components/panels/global/dstar_lite/BackwardField";
import dstarPy from "../../../../../../python/navigation/global_planning/search/dstar_lite.py?raw";
import dstarCpp from "../../../../../../cpp/src/global_planning/search/dstar_lite.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const DstarLite = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    A*, Dijkstra, and BFS all assume the map is known and fixed. Real robots get
                    neither: the map is partial, and the world contradicts it mid-drive. D* Lite
                    (Koenig &amp; Likhachev, 2002) is the standard answer — an incremental planner
                    that repairs its previous search instead of restarting it, built for the loop
                    every mobile robot actually lives in: move, sense, replan.
                </p>}
                ko={<p>
                    A*, Dijkstra, BFS는 모두 지도가 알려져 있고 변하지 않는다고 가정한다. 실제
                    로봇에게는 둘 다 사치다. 지도는 불완전하고, 세계는 주행 중에 그 지도를
                    배반한다. D* Lite(Koenig &amp; Likhachev, 2002)는 그 상황의 표준 답이다.
                    이전 탐색을 버리고 다시 시작하는 대신 수리하는 incremental planner로, 모든
                    모바일 로봇이 실제로 사는 루프인 move, sense, replan을 위해 설계됐다.
                </p>}
            />

            <h2>{t("Planning Without a Map", "지도 없이 계획하기")}</h2>
            <T
                en={<>
                    <p>
                        The setting: the robot knows only the grid's size, assumes every cell free,
                        and carries a sensor that reveals true occupancy within a small radius. It
                        must reach the goal anyway. The naive strategy works — plan with A* under
                        the current belief, drive, and whenever the sensor contradicts the belief,
                        throw the plan away and run A* again — but on a long corridor of
                        discoveries you pay a full search per wall.
                    </p>
                    <p>
                        The observation that makes repair possible: a newly discovered wall
                        invalidates only the part of the search that <em>passed through it</em>.
                        Most of the cost-to-goal information computed last time is still correct.
                        D* Lite keeps exactly that information alive between replans.
                    </p>
                </>}
                ko={<>
                    <p>
                        설정은 이렇다. 로봇은 격자 크기만 알고 모든 셀이 비어 있다고 가정하며,
                        작은 반경 안의 실제 점유를 알려 주는 센서를 갖고 있다. 그래도 goal에
                        도착해야 한다. 순진한 전략도 동작은 한다. 현재 believe하는 지도로 A*를
                        돌리고, 주행하다 센서가 그 믿음과 어긋나면 계획을 버리고 A*를 다시
                        돌리는 것이다. 다만 발견이 이어지는 긴 복도에서는 벽 하나마다 전체 탐색
                        비용을 다시 낸다.
                    </p>
                    <p>
                        수리를 가능하게 하는 관찰은 하나다. 새로 발견된 벽은 탐색 결과 중{" "}
                        <em>그 벽을 지나던 부분</em>만 무효화한다. 지난번에 계산한 goal까지의
                        비용 정보 대부분은 여전히 옳다. D* Lite는 정확히 그 정보를 replan 사이에
                        살려 둔다.
                    </p>
                </>}
            />

            <h2>{t("Backward Search and the g/rhs Pair", "Backward 탐색과 g/rhs")}</h2>
            <T
                en={<>
                    <p>
                        Two design choices carry the whole algorithm.
                    </p>
                    <p>
                        <strong>Search backward, from the goal.</strong>{" "}
                        <InlineMath math="g(s)"/> estimates the cost from <InlineMath math="s"/>{" "}
                        <em>to the goal</em>, not from the start. The robot is the thing that
                        moves; the goal stays put. A cost-to-goal field survives every robot step
                        unchanged — only discovered walls can break it:
                    </p>
                </>}
                ko={<>
                    <p>
                        설계 선택 두 개가 알고리즘 전체를 떠받친다.
                    </p>
                    <p>
                        <strong>goal에서 뒤로 탐색한다.</strong> <InlineMath math="g(s)"/>는
                        시작점이 아니라 <InlineMath math="s"/>에서 <em>goal까지</em>의 비용
                        추정치다. 움직이는 쪽은 로봇이고 goal은 제자리에 있다. goal 기준 비용장은
                        로봇이 몇 걸음을 옮겨도 그대로 유효하고, 그것을 깨뜨릴 수 있는 것은
                        발견된 벽뿐이다:
                    </p>
                </>}
            />
            <BackwardField/>
            <T
                en={<>
                    <p>
                        <strong>Keep two value functions.</strong> Alongside{" "}
                        <InlineMath math="g(s)"/> (the committed estimate), D* Lite maintains a
                        one-step lookahead:
                    </p>
                    <BlockMath math="rhs(s) \;=\; \min_{s' \in succ(s)}\bigl(c(s, s') + g(s')\bigr), \qquad rhs(\text{goal}) = 0"/>
                    <Terms items={[
                        ["g(s)", <>committed estimate of the cost <em>from <InlineMath math="s"/> to the goal</em> (the search runs backward, so g measures cost-to-goal, not cost-from-start)</>],
                        ["rhs(s)", <><strong>the new term</strong>: one-step lookahead — the best cost through any successor, recomputed instantly when an edge changes</>],
                        ["succ(s)", <>grid neighbors of <InlineMath math="s"/></>],
                        ["c(s, s')", "edge cost between neighbors (∞ once a wall is discovered)"],
                    ]}/>
                    <p>
                        A vertex is <em>consistent</em> when <InlineMath math="g(s) = rhs(s)"/>.
                        When a wall appears, the affected vertices' <InlineMath math="rhs"/>{" "}
                        change immediately (it is a local minimum over neighbors), they become
                        inconsistent, and only inconsistent vertices enter the queue. The repair
                        wave spreads exactly as far as the damage does — everywhere else,{" "}
                        <InlineMath math="g = rhs"/> and nothing needs touching. A scalar offset{" "}
                        <InlineMath math="k_m"/> accumulates the robot's movement so that queue
                        keys computed before a move remain comparable after it, without re-keying
                        the whole queue.
                    </p>
                </>}
                ko={<>
                    <p>
                        <strong>값 함수를 두 개 유지한다.</strong> 확정 추정치{" "}
                        <InlineMath math="g(s)"/> 옆에 one-step lookahead를 하나 더 둔다:
                    </p>
                    <BlockMath math="rhs(s) \;=\; \min_{s' \in succ(s)}\bigl(c(s, s') + g(s')\bigr), \qquad rhs(\text{goal}) = 0"/>
                    <Terms items={[
                        ["g(s)", <><InlineMath math="s"/>에서 <em>goal까지</em> 비용의 확정 추정치 (탐색이 backward라 g는 시작부터가 아니라 goal까지의 비용이다)</>],
                        ["rhs(s)", <><strong>새로 추가된 항</strong>: one-step lookahead. 어떤 successor를 거치는 최선 비용으로, 간선이 바뀌면 즉시 다시 계산된다</>],
                        ["succ(s)", <><InlineMath math="s"/>의 grid 이웃들</>],
                        ["c(s, s')", "이웃 간 간선 비용 (벽이 발견되면 ∞)"],
                    ]}/>
                    <p>
                        <InlineMath math="g(s) = rhs(s)"/>인 vertex를 <em>consistent</em>하다고
                        한다. 벽이 나타나면 영향을 받은 vertex들의 <InlineMath math="rhs"/>가
                        즉시 바뀌고(이웃에 대한 국소 최솟값이므로), 그 vertex들은 inconsistent가
                        되며, 큐에는 inconsistent한 vertex만 들어간다. 수리 파동은 손상이 미친
                        만큼만 퍼진다. 그 밖의 모든 곳은 <InlineMath math="g = rhs"/>라 건드릴
                        필요가 없다. 스칼라 오프셋 <InlineMath math="k_m"/>은 로봇의 이동량을
                        누적해서, 이동 전에 계산된 큐 키가 이동 후에도 비교 가능하게 만든다. 큐
                        전체를 다시 키잉하지 않아도 된다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Complete and correct under the current belief</strong>: after each
                        repair, the greedy step follows a path that is optimal for everything the
                        robot knows so far.</li>
                    <li><strong>Incremental</strong>: repair cost scales with the size of the
                        affected region, not the map. Worst case (a discovery that invalidates
                        everything) degrades to a full A*; typical discoveries touch a small
                        neighborhood.</li>
                    <li><strong>Not globally optimal in hindsight</strong>: the executed trajectory
                        is optimal per-belief, but a different first guess could have avoided a
                        trap entirely — with an unknown map no online algorithm can beat that.</li>
                    <li><strong>Memory</strong> <InlineMath math="O(V)"/> for{" "}
                        <InlineMath math="g/rhs"/>; the queue holds only inconsistent vertices.</li>
                </ul>}
                ko={<ul>
                    <li><strong>현재 believe 기준으로 완전하고 정확하다</strong>: 매 수리 후의
                        greedy 한 걸음은 로봇이 지금까지 아는 정보에 대해 최적인 경로를 따른다.</li>
                    <li><strong>Incremental</strong>: 수리 비용은 지도 크기가 아니라 영향권
                        크기에 비례한다. 최악의 경우(모든 것을 무효화하는 발견)는 A* 전체
                        재실행으로 퇴화하지만, 보통의 발견은 작은 이웃만 건드린다.</li>
                    <li><strong>사후적으로 전역 최적은 아니다</strong>: 실행된 궤적은 believe
                        기준 최적일 뿐, 처음부터 다른 길을 찍었다면 함정을 통째로 피했을 수도
                        있다. 지도를 모르는 이상 어떤 online 알고리즘도 이것을 이길 수 없다.</li>
                    <li><strong>메모리</strong>는 <InlineMath math="g/rhs"/>에{" "}
                        <InlineMath math="O(V)"/>. 큐에는 inconsistent한 vertex만 들어간다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The state is the pair <InlineMath math="g/rhs"/>, a priority queue{" "}
                    <InlineMath math="U"/> of inconsistent vertices, and the offset{" "}
                    <InlineMath math="k_m"/>. Three routines cooperate: a key that orders repairs,
                    a vertex update that detects inconsistency, and a bounded best-first pass that
                    restores consistency around the robot:
                </p>}
                ko={<p>
                    상태는 <InlineMath math="g/rhs"/> 쌍, inconsistent vertex를 담는 우선순위 큐{" "}
                    <InlineMath math="U"/>, 그리고 오프셋 <InlineMath math="k_m"/>이다. 루틴 세
                    개가 맞물린다. 수리 순서를 정하는 key, inconsistency를 감지하는 vertex
                    갱신, 로봇 주변의 consistency를 복원하는 한정된 best-first 패스다:
                </p>}
            />
            <Pseudocode code={`key(s) = [min(g[s], rhs[s]) + h(robot, s) + k_m,  min(g[s], rhs[s])]    # 1

update_vertex(u):                                                        # 2
    if u ≠ goal:  rhs[u] ← min over successors s' of (c(u, s') + g[s'])
    remove u from U
    if g[u] ≠ rhs[u]:  insert u into U with key(u)

compute_shortest_path():                                                 # 3
    while top_key(U) < key(robot)  or  rhs[robot] ≠ g[robot]:
        u ← pop_min(U)
        if g[u] > rhs[u]:  g[u] ← rhs[u];  update_vertex(each pred of u)
        else:              g[u] ← ∞;       update_vertex(u and each pred)

main:
    rhs[goal] ← 0;  insert goal;  sense();  compute_shortest_path()
    while robot ≠ goal:
        robot ← argmin over successors of (c + g)                        # 4
        sense()                                                          # 5
        if new walls found:
            k_m += h(previous robot, robot)
            update_vertex(each vertex next to a new wall)
            compute_shortest_path()`}/>
            <T
                en={<ol>
                    <li>The key mirrors A*'s <InlineMath math="f"/>: estimated total cost through{" "}
                        <InlineMath math="s"/>, heuristic measured to the <em>robot</em> (the
                        backward search's target), plus <InlineMath math="k_m"/> so keys from
                        before a move stay comparable.</li>
                    <li>Recompute the one-step lookahead. If it disagrees with{" "}
                        <InlineMath math="g"/>, the vertex is inconsistent and queues for repair;
                        if they agree, it leaves the queue.</li>
                    <li>Fix inconsistent vertices in key order, but only until the robot's own
                        value is consistent — the pass stops as soon as the robot has a reliable
                        next step, leaving the rest of the map unrepaired on purpose. An
                        over-consistent vertex (<InlineMath math="g > rhs"/>) accepts the better
                        value; an under-consistent one is reset to <InlineMath math="\infty"/>{" "}
                        first so the raise can propagate.</li>
                    <li>One greedy step down the repaired cost field.</li>
                    <li>Sense; if the belief changed, shift <InlineMath math="k_m"/>, mark the
                        damaged vertices, and repair.</li>
                </ol>}
                ko={<ol>
                    <li>key는 A*의 <InlineMath math="f"/>와 같은 꼴이다. <InlineMath math="s"/>를
                        지나는 총 비용 추정에, backward 탐색의 목적지인 <em>로봇</em>까지의
                        heuristic을 더하고, 이동 전 키와의 비교 가능성을 위해{" "}
                        <InlineMath math="k_m"/>을 더한다.</li>
                    <li>one-step lookahead를 다시 계산한다. <InlineMath math="g"/>와 다르면
                        inconsistent이므로 수리 대기열에 넣고, 같으면 큐에서 뺀다.</li>
                    <li>inconsistent vertex를 key 순서로 고치되, 로봇 자신의 값이 consistent해질
                        때까지만 고친다. 로봇이 믿을 수 있는 다음 한 걸음을 얻는 즉시 패스가
                        멈추고, 지도의 나머지는 일부러 수리하지 않은 채 둔다. over-consistent
                        (<InlineMath math="g > rhs"/>)면 더 나은 값을 받아들이고,
                        under-consistent면 먼저 <InlineMath math="\infty"/>로 올려서 인상이
                        전파되게 한다.</li>
                    <li>수리된 비용장을 따라 greedy로 한 걸음 내려간다.</li>
                    <li>감지한다. believe가 바뀌었으면 <InlineMath math="k_m"/>을 밀고, 손상된
                        vertex를 표시하고, 수리한다.</li>
                </ol>}
            />

            <h2>{t("Why Repair Is Cheap", "수리가 싼 이유")}</h2>
            <T
                en={<p>
                    The whole efficiency argument hangs on one invariant: <strong>the queue
                    contains exactly the inconsistent vertices</strong>. Consistency is local, so
                    a new wall makes only its neighbors inconsistent; the best-first pass then
                    touches a vertex only if fixing another vertex changed its lookahead. Repair
                    work is proportional to the region the discovery actually influences — and the
                    early-exit condition stops even that as soon as the robot's next step is
                    trustworthy.
                </p>}
                ko={<p>
                    효율 논증 전체가 불변식 하나에 걸려 있다. <strong>큐에는 정확히 inconsistent
                    vertex만 들어 있다</strong>는 것이다. consistency는 국소 성질이라 새 벽은 그
                    이웃만 inconsistent로 만들고, best-first 패스는 어떤 vertex를 고친 결과가
                    다른 vertex의 lookahead를 바꿨을 때만 그 vertex를 건드린다. 수리량은 발견이
                    실제로 영향을 준 영역에 비례하고, 조기 종료 조건은 로봇의 다음 걸음이 믿을
                    만해지는 순간 그마저도 멈춘다.
                </p>}
            />
            <Proof title={t("Invariant (queue = inconsistent set)", "불변식 (큐 = inconsistent 집합)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Initially only the goal is inconsistent
                            (<InlineMath math="rhs = 0 \ne g = \infty"/>) and queued. Every
                            mutation of <InlineMath math="g"/> or <InlineMath math="rhs"/> passes
                            through <code>update_vertex</code>, which ends with:
                        </p>
                        <BlockMath math="u \in U \;\Longleftrightarrow\; g(u) \ne rhs(u)"/>
                        <Terms items={[
                            ["U", "the priority queue: it holds exactly the inconsistent vertices, nothing else"],
                            ["g(u),\ rhs(u)", <>committed estimate vs one-step lookahead of cost-to-goal; equality means <InlineMath math="u"/> is consistent (settled and correct)</>],
                        ]}/>
                        <p>
                            When <InlineMath math="U"/> empties (or the early exit fires), every
                            vertex the robot's value depends on satisfies{" "}
                            <InlineMath math="g = rhs"/>, and unrolling the{" "}
                            <InlineMath math="rhs"/> definition gives the Bellman equation for
                            cost-to-goal — so <InlineMath math="g"/> is the true shortest distance
                            under the current belief. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 처음에는 goal만 inconsistent
                            (<InlineMath math="rhs = 0 \ne g = \infty"/>)이고 큐에 있다.{" "}
                            <InlineMath math="g"/>나 <InlineMath math="rhs"/>의 모든 변경은{" "}
                            <code>update_vertex</code>를 거치고, 그 끝은 항상 다음을 복원한다:
                        </p>
                        <BlockMath math="u \in U \;\Longleftrightarrow\; g(u) \ne rhs(u)"/>
                        <Terms items={[
                            ["U", "priority queue. inconsistent한 vertex 정확히 그들만 들어 있다"],
                            ["g(u),\ rhs(u)", <>goal까지 비용의 확정 추정치 vs one-step lookahead. 같으면 <InlineMath math="u"/>는 consistent (확정이고 올바름)</>],
                        ]}/>
                        <p>
                            <InlineMath math="U"/>가 비면(또는 조기 종료가 발동하면) 로봇의 값이
                            의존하는 모든 vertex에서 <InlineMath math="g = rhs"/>이고,{" "}
                            <InlineMath math="rhs"/> 정의를 풀면 goal까지 비용에 대한 Bellman
                            방정식이 된다. 즉 <InlineMath math="g"/>는 현재 believe 기준의 실제
                            최단 거리다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    In the sandbox the walls you draw are ground truth the robot cannot see —
                    they render as ghosts until its sensor finds them. The counter compares
                    D* Lite's total repair expansions against rerunning A* from scratch at every
                    discovery. On the <em>scattered</em> preset each discovery invalidates only a
                    small patch, and repair wins severalfold; switch to{" "}
                    <em>trap</em> for the worst case, where one C-shaped discovery invalidates
                    nearly everything and repair degrades to a full replan. Widening the sensor
                    finds trouble earlier on both.
                </p>}
                ko={<p>
                    sandbox에서 그리는 벽은 로봇이 볼 수 없는 실제 지도다. 센서가 찾아내기
                    전까지는 흐릿하게만 그려진다. 카운터는 D* Lite의 총 수리 확장 수와, 발견
                    때마다 A*를 처음부터 다시 돌렸을 때의 확장 수를 비교한다. <em>scattered</em>{" "}
                    프리셋에서는 발견 하나가 작은 조각만 무효화해서 수리가 몇 배 차이로 이긴다.{" "}
                    <em>trap</em>으로 바꾸면 최악 케이스가 나온다. C자 발견 하나가 거의 전부를
                    무효화해서 수리가 전체 재계획 수준으로 퇴화한다. 센서를 넓히면 어느 쪽이든
                    문제를 더 일찍 알아챈다.
                </p>}
            />
            <DStarLiteSandbox/>
            <TraceReplay algo="dstar_lite" maps={["dstar_trap01", "maze01"]} label={t(
                "Real traces from the repository's D* Lite demo — repair bursts between robot steps, walls appearing as the sensor finds them",
                "저장소 D* Lite demo의 실제 trace. 로봇 걸음 사이의 수리 파동과, 센서에 잡히는 순간 나타나는 벽들",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    Unlike Dijkstra and A*, D* Lite does not ride the shared best-first core — the
                    move→sense→repair loop, the backward search, and the g/rhs bookkeeping are its
                    own. The full implementation is embedded below.
                </p>}
                ko={<p>
                    Dijkstra, A*와 달리 D* Lite는 공유 best-first 코어를 쓰지 않는다.
                    move→sense→repair 루프, backward 탐색, g/rhs 관리가 전부 고유 구조이기
                    때문이다. 전체 구현을 아래에 embed했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/dstar_lite.py",
                            code: dstarPy,
                            href: `${REPO}/python/navigation/global_planning/search/dstar_lite.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/dstar_lite.cpp",
                            code: dstarCpp,
                            href: `${REPO}/cpp/src/global_planning/search/dstar_lite.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete D* Lite implementation, embedded from the repository sources",
                    "D* Lite 전체 구현. 저장소 소스를 그대로 embed한 것이다",
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
                <li>
                    A. Stentz,{" "}
                    <a href="https://doi.org/10.1109/ROBOT.1994.351061" target="_blank"
                       rel="noopener noreferrer">
                        <em>Optimal and Efficient Path Planning for Partially-Known
                            Environments</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation, 1994.
                </li>
            </ol>
        </>
    )
}

export default DstarLite
