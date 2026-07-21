import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import SstSandbox from "../../../../components/panels/global/sst/SstSandbox";
import sstPy from "../../../../../../python/navigation/global_planning/sampling/sst.py?raw";
import sstCpp from "../../../../../../cpp/src/global_planning/sampling/sst.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Sst = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every RRT* on the earlier pages assumed a{" "}
                    <em>steering function</em>: given two states, it could draw the exact
                    edge between them. Real vehicles cannot. A car cannot slide sideways to
                    the sampled point, and solving the two-point boundary-value problem that
                    would connect two poses under its dynamics is usually intractable. SST
                    (Stable Sparse RRT, Li, Littlefield &amp; Bekris, 2016) throws the
                    steering function away. It grows the tree by picking a node, firing a{" "}
                    <em>random control</em> forward for a random duration, and keeping
                    wherever the vehicle lands. The price of forward-only growth is a tree
                    that would explode with redundant nodes, so SST pairs it with a witness
                    rule that keeps only the locally cheapest node in each neighborhood and
                    prunes the rest. Stable, sparse, and no boundary-value solver anywhere.
                </p>}
                ko={<p>
                    앞 페이지의 RRT*은 모두 <em>steering function</em>을 전제했다. 두 상태가
                    주어지면 그 사이 정확한 간선을 그을 수 있다는 것이다. 실제 차량은 그럴 수
                    없다. 자동차는 표본 점으로 옆으로 미끄러져 갈 수 없고, 두 자세를 그 동역학
                    아래 잇는 two-point 경계값 문제는 대개 풀기 어렵다. SST(Stable Sparse RRT,
                    Li, Littlefield &amp; Bekris, 2016)는 steering function을 버린다. 노드
                    하나를 고르고 <em>랜덤 control</em>을 랜덤 시간만큼 정방향으로 쏴, 차량이
                    도착한 자리를 그대로 남기며 트리를 키운다. 정방향 성장의 대가는 중복
                    노드로 폭발하는 트리라, SST는 근방에서 지역적으로 가장 싼 노드만 남기고
                    나머지를 쳐내는 witness 규칙을 함께 쓴다. 안정적이고, 희소하며, 경계값
                    solver는 어디에도 없다.
                </p>}
            />

            <h2>{t("Growing Without a Steering Function", "steer 없이 트리를 키운다")}</h2>
            <T
                en={<>
                    <p>
                        This repository runs SST in a purely geometric 2D world. The map only
                        answers state and motion validity (the same <InlineMath math="\text{SamplingSpace}"/>{" "}
                        the RRT family uses), and the planner owns a tiny unicycle model: a
                        state is <InlineMath math="(x, y, \theta)"/>, a control is a forward
                        speed and turn rate <InlineMath math="(v, \omega)"/> held constant,
                        and an edge is the Euler-integrated arc the vehicle traces. Cost is
                        the geometric length of that arc, so it stays comparable with the
                        earlier planners. Two radii govern everything:
                    </p>
                    <BlockMath math="x_{\text{sel}} = \arg\min_{x \in V_{\text{act}} \,\cap\, B(s, \delta_{\text{BN}})} \mathrm{cost}(x) \qquad\qquad \text{keep } x_{\text{new}} \iff \mathrm{cost}(x_{\text{new}}) < \mathrm{cost}\bigl(\mathrm{rep}(w)\bigr)"/>
                    <Terms items={[
                        ["x_{\\text{sel}}", "the node chosen to propagate from this iteration (BestNear)"],
                        ["V_{\\text{act}}", "the active node set — the nodes still allowed to be selected and grown"],
                        ["s", "the (goal-biased) sample drawn this iteration"],
                        ["B(s, \\delta_{\\text{BN}})", <>the ball of radius <InlineMath math="\delta_{\text{BN}}"/> around the sample</>],
                        ["\\mathrm{cost}(x)", <>accumulated arc length from the start to <InlineMath math="x"/></>],
                        ["\\delta_{\\text{BN}}", <><strong>the new term</strong>: the BestNear radius. Selection prefers the cheapest active node near the sample, not the nearest, which biases growth along low-cost routes</>],
                        ["x_{\\text{new}}", "the state reached by propagating a random control forward from the selected node"],
                        ["w = \\mathrm{witness}(x_{\\text{new}})", <><strong>the new term</strong>: the witness governing <InlineMath math="x_{\text{new}}"/> — the nearest witness point, or a fresh one if none lies within <InlineMath math="\delta_s"/></>],
                        ["\\mathrm{rep}(w)", <>the current active representative of witness <InlineMath math="w"/> (none, if the witness is fresh)</>],
                    ]}/>
                    <p>
                        The BestNear radius <InlineMath math="\delta_{\text{BN}}"/> decides{" "}
                        <em>who grows</em>: among active nodes near the sample, the cheapest
                        one is chosen, so the tree pushes outward along inexpensive routes.
                        The witness radius <InlineMath math="\delta_s"/> decides{" "}
                        <em>who survives</em>: the space is covered by witness points no
                        closer than <InlineMath math="\delta_s"/> to each other, each witness
                        keeps a single active representative, and a new node is admitted only
                        when it beats its witness&apos;s representative on cost. The loser is
                        deactivated and, if it is a childless leaf, pruned away with any
                        now-orphaned ancestors.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 저장소는 SST를 순수 기하 2D 세계에서 돌린다. 맵은 상태·이동
                        유효성만 답하고(RRT 계열이 쓰는 그 <InlineMath math="\text{SamplingSpace}"/>와
                        같다), planner가 작은 unicycle 모델을 소유한다. 상태는{" "}
                        <InlineMath math="(x, y, \theta)"/>, control은 상수로 유지되는 전진
                        속도와 회전율 <InlineMath math="(v, \omega)"/>, 간선은 차량이 그리는
                        Euler 적분 호다. 비용은 그 호의 기하 길이라, 앞 planner들과 그대로
                        비교된다. 반경 둘이 모든 것을 지배한다:
                    </p>
                    <BlockMath math="x_{\text{sel}} = \arg\min_{x \in V_{\text{act}} \,\cap\, B(s, \delta_{\text{BN}})} \mathrm{cost}(x) \qquad\qquad \text{keep } x_{\text{new}} \iff \mathrm{cost}(x_{\text{new}}) < \mathrm{cost}\bigl(\mathrm{rep}(w)\bigr)"/>
                    <Terms items={[
                        ["x_{\\text{sel}}", "이번 반복에 전파의 출발점으로 고른 노드 (BestNear)"],
                        ["V_{\\text{act}}", "active 노드 집합. 아직 선택·성장이 허용되는 노드들"],
                        ["s", "이번 반복에 뽑은 (goal-bias) 표본"],
                        ["B(s, \\delta_{\\text{BN}})", <>표본 둘레 반경 <InlineMath math="\delta_{\text{BN}}"/>의 공</>],
                        ["\\mathrm{cost}(x)", <>시작에서 <InlineMath math="x"/>까지 누적된 호 길이</>],
                        ["\\delta_{\\text{BN}}", <><strong>새로 추가된 항</strong>: BestNear 반경. 선택은 표본 근방에서 가장 가까운 노드가 아니라 가장 싼 active 노드를 택해, 저비용 경로를 따라 성장을 몬다</>],
                        ["x_{\\text{new}}", "고른 노드에서 랜덤 control을 정방향 전파해 도착한 상태"],
                        ["w = \\mathrm{witness}(x_{\\text{new}})", <><strong>새로 추가된 항</strong>: <InlineMath math="x_{\text{new}}"/>을 지배하는 witness. 가장 가까운 witness 점이거나, <InlineMath math="\delta_s"/> 안에 하나도 없으면 새로 만든 witness</>],
                        ["\\mathrm{rep}(w)", <>witness <InlineMath math="w"/>의 현재 active 대표 (witness가 새것이면 없음)</>],
                    ]}/>
                    <p>
                        BestNear 반경 <InlineMath math="\delta_{\text{BN}}"/>은{" "}
                        <em>누가 성장하는가</em>를 정한다. 표본 근방 active 노드 중 가장 싼
                        것을 골라, 트리가 저렴한 경로를 따라 바깥으로 뻗는다. witness 반경{" "}
                        <InlineMath math="\delta_s"/>은 <em>누가 살아남는가</em>를 정한다.
                        공간은 서로 <InlineMath math="\delta_s"/>보다 가깝지 않은 witness
                        점들로 덮이고, 각 witness는 active 대표 하나만 유지하며, 새 노드는 그
                        witness의 대표를 비용에서 이겨야만 받아들여진다. 진 쪽은 비활성화되고,
                        자식 없는 leaf면 이제 고아가 된 조상들과 함께 가지치기된다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>No steering / boundary-value solver</strong>: the tree grows
                        by forward propagation of random controls only, so SST plans for
                        dynamics where connecting two states exactly is intractable.</li>
                    <li><strong>Sparse and bounded active set</strong>: because witnesses are{" "}
                        <InlineMath math="\delta_s"/>-separated and each keeps one
                        representative, the number of active nodes is bounded independently
                        of iteration count — the sandbox shows it drop as{" "}
                        <InlineMath math="\delta_s"/> grows.</li>
                    <li><strong>Anytime, asymptotically near-optimal</strong>: the incumbent
                        cost is non-increasing, and SST* (shrinking{" "}
                        <InlineMath math="\delta_{\text{BN}}"/> and <InlineMath math="\delta_s"/>{" "}
                        over iterations) recovers asymptotic optimality.</li>
                    <li><strong>Probabilistically complete</strong> for the propagated
                        control set, exactly as kinodynamic RRT.</li>
                    <li><strong>Cost</strong>: per iteration one BestNear scan over the
                        active set (<InlineMath math="O(|V_{\text{act}}|)"/>) and one nearest
                        witness scan (<InlineMath math="O(|W|)"/>), plus the collision checks
                        along the propagated arc — no near-set radius search or rewiring.</li>
                </ul>}
                ko={<ul>
                    <li><strong>steer/경계값 solver 없음</strong>: 트리는 랜덤 control의
                        정방향 전파만으로 자라므로, 두 상태를 정확히 잇는 것이 어려운 동역학도
                        SST는 계획한다.</li>
                    <li><strong>희소하고 유계인 active 집합</strong>: witness가 서로{" "}
                        <InlineMath math="\delta_s"/>만큼 떨어져 있고 각자 대표 하나만
                        유지하므로, active 노드 수는 반복 수와 무관하게 유계다. sandbox에서{" "}
                        <InlineMath math="\delta_s"/>를 키우면 그 수가 줄어드는 것이 보인다.</li>
                    <li><strong>Anytime, 점근 준최적</strong>: 현직 비용이 비증가하고,
                        SST*(<InlineMath math="\delta_{\text{BN}}"/>과{" "}
                        <InlineMath math="\delta_s"/>를 반복에 따라 축소)은 점근 최적성을
                        회복한다.</li>
                    <li>전파하는 control 집합에 대해 kinodynamic RRT와 똑같이{" "}
                        <strong>확률적 완전</strong>.</li>
                    <li><strong>비용</strong>: 반복마다 active 집합 위 BestNear 스캔 한 번
                        (<InlineMath math="O(|V_{\text{act}}|)"/>)과 nearest witness 스캔 한
                        번(<InlineMath math="O(|W|)"/>), 그리고 전파한 호를 따른 충돌 검사.
                        near 반경 탐색이나 rewire는 없다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One selection, one random propagation, one domination test per
                    iteration. No steering, no near-set, no rewiring:
                </p>}
                ko={<p>
                    반복마다 선택 하나, 랜덤 전파 하나, 지배 검정 하나. steer도, near 집합도,
                    rewire도 없다:
                </p>}
            />
            <Pseudocode code={`V_active ← {start};  witnesses ← {start};  best ← ∞
repeat max_iterations times:
    s ← goal with probability p, else uniform sample
    x_sel ← cheapest active node within δ_BN of s (else nearest)   # 1
    x_new ← propagate random control (v, ω) forward;  skip if in collision  # 2
    w ← nearest witness to x_new;  add w if none within δ_s        # 3
    if w has a representative and cost(x_new) ≥ cost(rep(w)): skip # 4
    add x_new under x_sel;  rep(w) ← x_new
    if old representative existed: deactivate and prune its leaf chain # 5
    if x_new reaches goal region and beats best:                  # 6
        best ← that cost;  publish improved path`}/>
            <T
                en={<ol>
                    <li>BestNear, not nearest: selection is by cost among active nodes in the
                        sample&apos;s ball, falling back to the nearest active node when the
                        ball is empty.</li>
                    <li>Forward propagation replaces steering entirely. A random{" "}
                        <InlineMath math="(v, \omega)"/> is held for a random duration and
                        Euler-integrated; every waypoint and the chord to it are collision
                        checked, and any hit discards the whole arc.</li>
                    <li>The witness is located first. A brand-new witness starts with no
                        representative, so the next test always admits the first node to
                        reach it.</li>
                    <li>The domination test: a node survives only if it is strictly cheaper
                        than the incumbent representative of its witness. This is what keeps
                        the active set sparse.</li>
                    <li>Pruning walks up from the deactivated node, dropping childless
                        inactive leaves so a whole dead branch collapses in one pass.</li>
                    <li>No early return: reaching the goal only updates the incumbent, and
                        the loop runs its full budget refining the answer.</li>
                </ol>}
                ko={<ol>
                    <li>nearest가 아니라 BestNear다. 선택은 표본의 공 안 active 노드 중 비용
                        기준이고, 공이 비면 가장 가까운 active 노드로 후퇴한다.</li>
                    <li>정방향 전파가 steering을 통째로 대신한다. 랜덤{" "}
                        <InlineMath math="(v, \omega)"/>를 랜덤 시간만큼 유지해 Euler
                        적분하고, 모든 waypoint와 그리로 가는 chord를 충돌 검사하며, 하나라도
                        부딪히면 호 전체를 버린다.</li>
                    <li>witness를 먼저 찾는다. 갓 만든 witness는 대표가 없으므로, 다음 검정은
                        그리로 처음 닿은 노드를 늘 받아들인다.</li>
                    <li>지배 검정: 노드는 자기 witness의 현직 대표보다 엄밀히 싸야만 살아남는다.
                        이것이 active 집합을 희소하게 유지한다.</li>
                    <li>가지치기는 비활성화된 노드에서 위로 올라가며 자식 없는 비활성 leaf를
                        떨궈, 죽은 가지 하나가 한 번에 무너진다.</li>
                    <li>조기 반환이 없다. goal 도달은 현직 해를 갱신할 뿐이고, 루프는 예산을 다
                        써 답을 다듬는다.</li>
                </ol>}
            />
            <Proof title={t("Why the active set stays bounded", "active 집합이 유계로 남는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> A witness is created only when a new
                            node lies farther than <InlineMath math="\delta_s"/> from every
                            existing witness. Hence the witness set{" "}
                            <InlineMath math="W"/> is <InlineMath math="\delta_s"/>-separated:
                        </p>
                        <BlockMath math="\forall\, w_i, w_j \in W,\ i \neq j:\quad \lVert w_i - w_j \rVert > \delta_s"/>
                        <Terms items={[
                            ["W", "the set of witness points accumulated so far"],
                            ["w_i, w_j", "any two distinct witnesses"],
                            ["\\delta_s", "the witness / sparsification radius"],
                        ]}/>
                        <p>
                            Place a ball of radius <InlineMath math="\delta_s/2"/> at each
                            witness. Separation makes these balls disjoint, and every one of
                            them lies inside the <InlineMath math="\delta_s/2"/>-dilation of
                            the reachable free region <InlineMath math="X"/>. Comparing
                            volumes bounds the count:
                        </p>
                        <BlockMath math="|V_{\text{act}}| \;\le\; |W| \;\le\; \frac{\mu\!\left(X \oplus B(0, \delta_s/2)\right)}{\mu\!\left(B(0, \delta_s/2)\right)} \;=\; N_{\text{pack}}(\delta_s)"/>
                        <Terms items={[
                            ["|V_{\\text{act}}|", "the number of active nodes"],
                            ["|W|", <>the number of witnesses; each holds at most one active representative, so <InlineMath math="|V_{\text{act}}| \le |W|"/></>],
                            ["\\mu(\\cdot)", "area (Lebesgue measure) of a region"],
                            ["X \\oplus B(0, \\delta_s/2)", <>the reachable free region <InlineMath math="X"/> dilated by a <InlineMath math="\delta_s/2"/> ball (Minkowski sum)</>],
                            ["N_{\\text{pack}}(\\delta_s)", <><strong>the new term</strong>: the packing number — a finite bound depending only on <InlineMath math="\delta_s"/> and the region, not on the iteration count</>],
                        ]}/>
                        <p>
                            The bound is independent of how many iterations run, so the tree
                            cannot accumulate active nodes without limit. Larger{" "}
                            <InlineMath math="\delta_s"/> shrinks{" "}
                            <InlineMath math="N_{\text{pack}}"/> and the sandbox&apos;s active
                            count with it. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> witness는 새 노드가 기존 모든 witness에서{" "}
                            <InlineMath math="\delta_s"/>보다 멀 때만 만들어진다. 따라서
                            witness 집합 <InlineMath math="W"/>는{" "}
                            <InlineMath math="\delta_s"/>만큼 분리되어 있다:
                        </p>
                        <BlockMath math="\forall\, w_i, w_j \in W,\ i \neq j:\quad \lVert w_i - w_j \rVert > \delta_s"/>
                        <Terms items={[
                            ["W", "지금까지 쌓인 witness 점 집합"],
                            ["w_i, w_j", "서로 다른 두 witness"],
                            ["\\delta_s", "witness / sparsification 반경"],
                        ]}/>
                        <p>
                            각 witness에 반경 <InlineMath math="\delta_s/2"/>의 공을 놓는다.
                            분리 조건이 이 공들을 서로소로 만들고, 그 각각은 도달 가능한 자유
                            영역 <InlineMath math="X"/>를 <InlineMath math="\delta_s/2"/>만큼
                            팽창시킨 영역 안에 든다. 부피를 비교하면 개수가 유계다:
                        </p>
                        <BlockMath math="|V_{\text{act}}| \;\le\; |W| \;\le\; \frac{\mu\!\left(X \oplus B(0, \delta_s/2)\right)}{\mu\!\left(B(0, \delta_s/2)\right)} \;=\; N_{\text{pack}}(\delta_s)"/>
                        <Terms items={[
                            ["|V_{\\text{act}}|", "active 노드 수"],
                            ["|W|", <>witness 수. 각자 active 대표를 최대 하나 들므로 <InlineMath math="|V_{\text{act}}| \le |W|"/></>],
                            ["\\mu(\\cdot)", "영역의 넓이 (Lebesgue 측도)"],
                            ["X \\oplus B(0, \\delta_s/2)", <>도달 가능한 자유 영역 <InlineMath math="X"/>를 <InlineMath math="\delta_s/2"/> 공으로 팽창시킨 것 (Minkowski 합)</>],
                            ["N_{\\text{pack}}(\\delta_s)", <><strong>새로 추가된 항</strong>: 패킹 수. 반복 수가 아니라 <InlineMath math="\delta_s"/>와 영역에만 의존하는 유한한 상한</>],
                        ]}/>
                        <p>
                            이 상한은 반복을 몇 번 돌리든 무관하므로, 트리는 active 노드를
                            한없이 쌓을 수 없다. <InlineMath math="\delta_s"/>가 클수록{" "}
                            <InlineMath math="N_{\text{pack}}"/>이 줄고, sandbox의 active
                            수도 따라 준다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox grows one SST tree around a single block. Raise the witness
                    radius <InlineMath math="\delta_s"/> from 0.3 to 0.5 and the active node
                    count drops sharply (roughly 840 down to 350 at seed 1) while the
                    same-budget RRT* keeps every one of its ~3800 sampled nodes: that gap is
                    the sparsity SST is named for. The replay below is the repository demo on
                    the benchmark grids, where each arc is a forward-propagated unicycle
                    control and pruned branches vanish as cheaper representatives take over.
                </p>}
                ko={<p>
                    sandbox는 블록 하나를 도는 SST 트리 하나를 키운다. witness 반경{" "}
                    <InlineMath math="\delta_s"/>를 0.3에서 0.5로 올리면 active 노드 수가
                    가파르게 준다(seed 1에서 약 840에서 350으로). 반면 같은 예산의 RRT*는
                    표본 노드 약 3800개를 하나도 버리지 않는다. 그 격차가 SST의 이름이 된
                    희소함이다. 아래 replay는 벤치마크 grid 위 저장소 demo다. 각 호는 정방향
                    전파된 unicycle control이고, 더 싼 대표가 자리를 넘겨받으면 가지치기된
                    가지가 사라진다.
                </p>}
            />
            <SstSandbox/>
            <TraceReplay vehicle carLength={0.8} algo="sst" maps={["maze01", "open01"]} label={t(
                "Real traces from the repository's SST demo — random controls propagate forward into arcs, and witness pruning keeps the active tree sparse",
                "저장소 SST demo의 실제 trace. 랜덤 control이 호로 정방향 전파되고, witness 가지치기가 active 트리를 희소하게 유지한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The planner owns its unicycle propagation and the witness bookkeeping
                    (active set, per-witness representative, leaf-chain pruning) in parallel
                    node arrays, since the shared sampling tree models none of that. Both
                    language versions are embedded in full below.
                </p>}
                ko={<p>
                    planner는 unicycle 전파와 witness 장부(active 집합, witness별 대표,
                    leaf-chain 가지치기)를 병렬 노드 배열로 직접 소유한다. 공유 sampling
                    트리는 그중 무엇도 모델링하지 않기 때문이다. 두 언어 버전 전체를 아래에
                    embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/sst.py",
                            code: sstPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/sst.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/sst.cpp",
                            code: sstCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/sst.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete SST implementation, embedded from the repository sources",
                    "SST 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    Y. Li, Z. Littlefield, K. E. Bekris,{" "}
                    <a href="https://doi.org/10.1177/0278364914558017" target="_blank"
                       rel="noopener noreferrer">
                        <em>Asymptotically Optimal Sampling-based Kinodynamic Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2016.
                </li>
                <li>
                    S. M. LaValle, J. J. Kuffner,{" "}
                    <a href="https://doi.org/10.1177/02783640122067453" target="_blank"
                       rel="noopener noreferrer">
                        <em>Randomized Kinodynamic Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2001.
                </li>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank"
                       rel="noopener noreferrer">
                        <em>Sampling-based Algorithms for Optimal Motion Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2011.
                </li>
            </ol>
        </>
    )
}

export default Sst
