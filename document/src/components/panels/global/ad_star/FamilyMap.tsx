import {useTr} from "../../../../libs/i18n";

// A* 계열 지도를 2×2 로 정리한 다이어그램. AD*가 두 축의 교차점임을 보여 준다.
// 외부 자산 없이 디자인 토큰만 쓴다.
const Quad = ({name, desc, accent}: {name: string; desc: string; accent?: boolean}) => (
    <div className={`rounded-xl border px-4 py-3 text-center ${
        accent
            ? "border-[var(--accent)] bg-[color-mix(in_srgb,var(--accent)_10%,transparent)]"
            : "border-border bg-surface"}`}>
        <div className="font-semibold">{name}</div>
        <div className="text-xs text-muted mt-0.5">{desc}</div>
    </div>
)

const FamilyMap = () => {
    const t = useTr()
    return (
        <div className="my-5 flex justify-center">
            <div className="w-full max-w-[560px]">
                <div className="grid gap-2" style={{gridTemplateColumns: "auto 1fr 1fr"}}>
                    <div/>
                    <div className="text-center text-xs font-semibold text-muted uppercase tracking-wide">
                        {t("known, fixed map", "지도를 알고, 고정")}
                    </div>
                    <div className="text-center text-xs font-semibold text-muted uppercase tracking-wide">
                        {t("unknown / changing map", "지도를 모르거나 변함")}
                    </div>
                    <div className="flex items-center justify-end text-xs font-semibold text-muted uppercase tracking-wide"
                         style={{writingMode: "vertical-rl", transform: "rotate(180deg)"}}>
                        {t("time to deliberate", "시간 여유 있음")}
                    </div>
                    <Quad name="A*" desc={t("one optimal search", "한 번의 최적 탐색")}/>
                    <Quad name="D* Lite" desc={t("incremental repair", "incremental 수리")}/>
                    <div className="flex items-center justify-end text-xs font-semibold text-muted uppercase tracking-wide"
                         style={{writingMode: "vertical-rl", transform: "rotate(180deg)"}}>
                        {t("answer needed now", "답이 지금 필요")}
                    </div>
                    <Quad name="ARA*" desc={t("anytime ε-schedule", "anytime ε-스케줄")}/>
                    <Quad name="AD*" accent
                          desc={t("anytime + incremental", "anytime + incremental")}/>
                </div>
            </div>
        </div>
    )
}

export default FamilyMap
