import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import RrtConnectSandbox from "../../../../components/panels/global/rrt_connect/RrtConnectSandbox";
import rrtConnectPy from "../../../../../../python/navigation/global_planning/sampling/rrt_connect.py?raw";
import rrtConnectCpp from "../../../../../../cpp/src/global_planning/sampling/rrt_connect.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const RrtConnect = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Why should only the start get a tree? RRT-Connect (Kuffner &amp; LaValle,
                    2000) grows one from each end and makes them chase each other: every
                    iteration, one tree takes a single exploratory step, and the other tree{" "}
                    <em>sprints</em> greedily toward the node just created until it arrives
                    or hits a wall. The two fronts meet in the middle of the problem instead
                    of one front crossing all of it — in practice the fastest feasible-path
                    planner in this section, and the default choice when you just need{" "}
                    <em>a</em> path quickly.
                </p>}
                ko={<p>
                    트리를 시작점에만 줄 이유가 있나? RRT-Connect(Kuffner &amp; LaValle,
                    2000)는 양끝에서 하나씩 키워 서로 쫓게 만든다. 매 반복, 한 트리가 탐사
                    스텝을 하나 딛으면, 다른 트리가 방금 생긴 그 노드를 향해 도착하거나
                    벽에 막힐 때까지 greedy 하게 <em>내달린다</em>. 한 전선이 문제 전체를
                    가로지르는 대신 두 전선이 가운데에서 만난다. 실전에서 이 섹션에서 가장
                    빠른 feasible planner이고, "일단 경로 하나"가 급할 때의 기본값이다.
                </p>}
            />

            <h2>{t("Extend Meets Connect", "EXTEND와 CONNECT가 만난다")}</h2>
            <T
                en={<>
                    <p>
                        The asymmetry is deliberate. EXTEND is RRT's cautious step: one{" "}
                        <InlineMath math="\eta"/> toward a uniform sample, keeping the
                        Voronoi-bias exploration. CONNECT is pure greed: repeat EXTEND toward
                        a <em>fixed</em> target until Reached or Trapped. Greed is safe here
                        because the target is not a random guess but an actual node of the
                        other tree — a known-good state that the trees must eventually share.
                        No goal bias is needed at all: each tree's target list <em>is</em>{" "}
                        the other tree.
                    </p>
                    <p>
                        Why is meeting in the middle roughly twice as cheap? A tree covering
                        distance <InlineMath math="L"/> needs a frontier that has diffused
                        that far, and with each front growing like a disc,
                    </p>
                    <BlockMath math="\text{work}(L) \;\sim\; c \cdot L^{2} \quad\Rightarrow\quad 2 \cdot \text{work}(L/2) \;=\; \tfrac{1}{2}\, \text{work}(L)"/>
                    <Terms items={[
                        ["L", "start-to-goal distance the search must span"],
                        ["\\text{work}(L)", <>tree size needed to reach distance <InlineMath math="L"/> — grows superlinearly (area-like in 2D) because the tree fills space as it extends</>],
                        ["c", "a constant absorbing map clutter and step size"],
                    ]}/>
                    <p>
                        Each of the two trees only spans half the distance, and the
                        superlinear growth makes two half-problems cheaper than one whole
                        one. The sandbox readout shows the ratio live.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 비대칭은 의도된 것이다. EXTEND는 RRT의 조심스러운 스텝이다.
                        균일 표본 쪽으로 <InlineMath math="\eta"/> 하나, Voronoi bias
                        탐사를 유지한다. CONNECT는 순수한 탐욕이다. <em>고정된</em> 목표를
                        향해 Reached 또는 Trapped까지 EXTEND를 반복한다. 여기서 탐욕이
                        안전한 이유는 목표가 무작위 추측이 아니라 상대 트리의 실제
                        노드이기 때문이다. 두 트리가 결국 공유해야 할, 확실히 좋은
                        상태다. goal bias는 아예 필요 없다. 각 트리의 목표 목록이 곧
                        상대 트리다.
                    </p>
                    <p>
                        가운데에서 만나는 것이 왜 대략 두 배 싼가? 거리{" "}
                        <InlineMath math="L"/>을 가로지르는 트리는 그만큼 퍼진 전선이
                        필요하고, 전선 하나가 원판처럼 자라므로
                    </p>
                    <BlockMath math="\text{work}(L) \;\sim\; c \cdot L^{2} \quad\Rightarrow\quad 2 \cdot \text{work}(L/2) \;=\; \tfrac{1}{2}\, \text{work}(L)"/>
                    <Terms items={[
                        ["L", "탐색이 가로질러야 하는 시작→goal 거리"],
                        ["\\text{work}(L)", <>거리 <InlineMath math="L"/>에 닿는 데 필요한 트리 크기. 트리가 뻗으면서 공간을 채우므로 초선형(2D에서는 넓이 꼴)으로 자란다</>],
                        ["c", "맵의 어수선함과 스텝 크기를 흡수하는 상수"],
                    ]}/>
                    <p>
                        두 트리는 각자 절반 거리만 감당하고, 초선형 성장 덕에 반쪽 문제
                        둘이 온쪽 문제 하나보다 싸다. sandbox readout이 그 비율을 실시간으로
                        보여 준다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Probabilistically complete</strong>, like RRT — and usually
                        far faster to a first path (about 2× on the sandbox corridor).</li>
                    <li><strong>Not optimal</strong>: the spliced path inherits both trees'
                        jaggedness, plus a seam where they met. The RRT* family, next, is
                        the cure.</li>
                    <li><strong>No goal bias</strong>: bidirectional growth replaces it —
                        one fewer parameter to tune, one fewer failure mode (a bias pulling
                        into a wall).</li>
                    <li><strong>The bridge must be checked</strong>: CONNECT stops within{" "}
                        <InlineMath math="\text{goal\_tol}"/> of the target, so the final
                        splice segment gets an explicit collision test — skipping it lets
                        paths tunnel through thin walls.</li>
                </ul>}
                ko={<ul>
                    <li>RRT처럼 <strong>확률적 완전</strong>이고, 첫 경로까지는 보통 훨씬
                        빠르다 (sandbox 복도에서 약 2배).</li>
                    <li><strong>최적이 아니다</strong>: 접합된 경로는 두 트리의 삐죽함에
                        만난 자리의 이음매까지 물려받는다. 다음의 RRT* 계열이 그 치료제다.</li>
                    <li><strong>goal bias가 없다</strong>: 양방향 성장이 그 역할을
                        대신한다. 튜닝할 파라미터가 하나 줄고, bias가 벽을 향해 당기는
                        실패 모드도 하나 사라진다.</li>
                    <li><strong>bridge는 검사해야 한다</strong>: CONNECT는 목표의{" "}
                        <InlineMath math="\text{goal\_tol}"/> 안에서 멈추므로 마지막 접합
                        구간에 명시적 충돌 검사가 붙는다. 건너뛰면 얇은 벽을 관통하는
                        경로가 성공으로 새어나간다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Two trees, alternating roles every iteration — the sampler drives one,
                    the other gives chase:
                </p>}
                ko={<p>
                    트리 둘이 매 반복 역할을 바꾼다. 표본이 한쪽을 몰고, 다른 쪽이 쫓는다:
                </p>}
            />
            <Pseudocode code={`T_a ← {start};  T_b ← {goal}
repeat up to max_iterations:
    q_rand ← uniform sample                                   # 1
    q_new ← EXTEND(T_a, q_rand)                               # 2
    if q_new exists:
        q_meet ← CONNECT(T_b, q_new)                          # 3
        if q_meet exists and bridge q_new → q_meet is free:   # 4
            return path(start → q_new) + path(q_meet → goal)
    swap(T_a, T_b)                                            # 5`}/>
            <T
                en={<ol>
                    <li>Uniform only — no goal bias, since each tree already aims at the
                        other.</li>
                    <li>EXTEND: RRT's step — nearest node, one <InlineMath math="\eta"/>{" "}
                        toward the sample, collision-checked (Trapped if blocked).</li>
                    <li>CONNECT: EXTEND repeatedly toward the <em>fixed</em>{" "}
                        <InlineMath math="q_{\text{new}}"/> until within tolerance
                        (Reached) or blocked (Trapped) — steer clamps to the target, so
                        progress is monotone and the loop terminates.</li>
                    <li>The two meeting nodes can be up to a tolerance apart; the explicit
                        bridge check closes that gap safely before splicing.</li>
                    <li>The swap keeps both trees growing — without it one tree would do
                        all the exploring and the balance argument above collapses.</li>
                </ol>}
                ko={<ol>
                    <li>균일 표본만 쓴다. goal bias는 없다. 각 트리가 이미 상대를 겨누고
                        있기 때문이다.</li>
                    <li>EXTEND: RRT의 그 스텝이다. nearest 노드에서 표본 쪽으로{" "}
                        <InlineMath math="\eta"/> 하나, 충돌 검사 포함 (막히면 Trapped).</li>
                    <li>CONNECT: <em>고정된</em> <InlineMath math="q_{\text{new}}"/>를
                        향해 허용 오차 안(Reached)이나 막힐 때(Trapped)까지 EXTEND를
                        반복한다. steer가 목표에서 잘리므로 전진이 단조라 루프는 반드시
                        끝난다.</li>
                    <li>만난 두 노드는 허용 오차만큼 떨어져 있을 수 있다. 접합 전에
                        명시적 bridge 검사가 그 틈을 안전하게 닫는다.</li>
                    <li>swap이 두 트리를 모두 자라게 한다. 없으면 한 트리가 탐사를 다
                        하게 되어 위의 절반 논증이 무너진다.</li>
                </ol>}
            />
            <Proof title={t("Why CONNECT terminates", "CONNECT가 반드시 끝나는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> CONNECT repeats EXTEND toward a fixed
                            target <InlineMath math="q"/>. Each Advanced step moves the
                            tree's nearest node strictly closer:
                        </p>
                        <BlockMath math="\lVert q_{\text{new}} - q \rVert \;=\; \max\bigl(\lVert q_{\text{near}} - q \rVert - \eta,\; 0\bigr)"/>
                        <Terms items={[
                            ["q", <>the fixed connect target — a node of the other tree</>],
                            ["q_{\\text{near}},\\ q_{\\text{new}}", "the tree's nearest node and the steered new node"],
                            ["\\eta", "the step size; steer clamps onto the target once within one step"],
                        ]}/>
                        <p>
                            Distance to <InlineMath math="q"/> decreases by{" "}
                            <InlineMath math="\eta"/> every Advanced step, so within{" "}
                            <InlineMath math="\lceil \lVert q_0 - q \rVert / \eta \rceil + 1"/>{" "}
                            steps CONNECT either Reaches or some step collides (Trapped).
                            Either way it returns. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> CONNECT는 고정 목표{" "}
                            <InlineMath math="q"/>를 향해 EXTEND를 반복한다. Advanced
                            스텝마다 트리의 nearest 노드가 엄밀히 가까워진다:
                        </p>
                        <BlockMath math="\lVert q_{\text{new}} - q \rVert \;=\; \max\bigl(\lVert q_{\text{near}} - q \rVert - \eta,\; 0\bigr)"/>
                        <Terms items={[
                            ["q", "고정된 connect 목표. 상대 트리의 노드다"],
                            ["q_{\\text{near}},\\ q_{\\text{new}}", "트리의 nearest 노드와 steer 된 새 노드"],
                            ["\\eta", "스텝 크기. 한 스텝 안으로 들어오면 steer가 목표에 정확히 붙인다"],
                        ]}/>
                        <p>
                            <InlineMath math="q"/>까지의 거리가 Advanced 스텝마다{" "}
                            <InlineMath math="\eta"/>씩 줄어드니,{" "}
                            <InlineMath math="\lceil \lVert q_0 - q \rVert / \eta \rceil + 1"/>{" "}
                            스텝 안에 Reached가 되거나 어느 스텝이 충돌한다(Trapped).
                            어느 쪽이든 반환한다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox is an S-corridor: a single tree must snake through two
                    turns, while the two RRT-Connect fronts each handle one turn and splice
                    in the middle — the readout pits its iteration count against
                    single-tree RRT on the same seed (about 2× on average). The replay
                    below is the repository demo; on the open map the trees meet almost
                    immediately.
                </p>}
                ko={<p>
                    sandbox는 S자 복도다. 단일 트리는 두 굽이를 다 지나야 하지만,
                    RRT-Connect의 두 전선은 굽이 하나씩 맡아 가운데에서 접합된다.
                    readout이 같은 seed의 단일 트리 RRT와 반복 수를 맞세운다 (평균
                    2배쯤). 아래 replay는 저장소 demo다. 열린 맵에서는 두 트리가 거의
                    즉시 만난다.
                </p>}
            />
            <RrtConnectSandbox/>
            <TraceReplay algo="rrt_connect" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's RRT-Connect demo — two trees grow from both endpoints and splice at the first clean meeting",
                "저장소 RRT-Connect demo의 실제 trace. 두 트리가 양 끝점에서 자라다 처음으로 깨끗이 만나는 곳에서 접합된다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    EXTEND and CONNECT are small private helpers over the shared tree; the
                    splice orientation is decided by object identity against the start
                    tree, so the returned path always begins at the start. Embedded below
                    in full.
                </p>}
                ko={<p>
                    EXTEND와 CONNECT는 공유 트리 위의 작은 private 헬퍼다. 접합 방향은
                    시작 트리와의 객체 동일성으로 정해져, 반환 경로가 항상 시작점에서
                    출발한다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/rrt_connect.py",
                            code: rrtConnectPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/rrt_connect.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/rrt_connect.cpp",
                            code: rrtConnectCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/rrt_connect.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete RRT-Connect implementation, embedded from the repository sources",
                    "RRT-Connect 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    J. J. Kuffner, S. M. LaValle,{" "}
                    <a href="https://doi.org/10.1109/ROBOT.2000.844730" target="_blank"
                       rel="noopener noreferrer">
                        <em>RRT-Connect: An Efficient Approach to Single-Query Path
                            Planning</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation, 2000.
                </li>
                <li>
                    S. M. LaValle, J. J. Kuffner,{" "}
                    <a href="https://doi.org/10.1177/02783640122067453" target="_blank"
                       rel="noopener noreferrer">
                        <em>Randomized Kinodynamic Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2001.
                </li>
            </ol>
        </>
    )
}

export default RrtConnect
