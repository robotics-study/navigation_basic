import {lazy, Suspense} from "react";
import {useTr} from "../../libs/i18n";

// 히어로의 라이브 엔진은 lazy import로 분리한다 — 첫 페인트를 막지 않도록 알고리즘
// 코드와 실행을 초기 번들 밖으로 뺀다.
const TwinDemo = lazy(() => import("./home/TwinDemo"));

// 홈 히어로 — 같은 문제를 A*(이산 탐색)와 RRT*(sampling)가 나란히 푸는 라이브 데모를
// 그대로 노출한다. 정적 GIF 대신 "이 사이트의 페이지가 이런 식으로 동작한다"를 첫
// 화면에서 바로 만지게 한다.
const HeroSearch = () => {
    const t = useTr()
    const vw = typeof window !== "undefined" ? window.innerWidth : 1024
    const wide = vw >= 720
    const panel = wide
        ? Math.min(300, Math.floor((Math.min(vw, 940) - 150) / 2))
        : Math.min(360, vw - 72)

    return (
        <div className="hero-3d">
            <div className="px-4 py-7">
                <Suspense fallback={
                    <div style={{minHeight: panel + 96}}
                         className="flex items-center justify-center text-sm text-muted">
                        {t("loading live demo…", "라이브 데모 로딩 중…")}
                    </div>
                }>
                    <TwinDemo panel={panel}/>
                </Suspense>
            </div>
        </div>
    )
}

export default HeroSearch
