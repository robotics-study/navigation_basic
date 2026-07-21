import {useTr} from "../../libs/i18n";
import {SandboxScene} from "./astar/AStarSandbox";

// 홈 히어로 — 라이브 A* sandbox 를 그대로 노출한다. 정적 GIF 그리드 대신
// "이 사이트의 페이지가 이런 식으로 동작한다"를 첫 화면에서 바로 만지게 한다.
const HeroSearch = () => {
    const t = useTr()
    return (
        <div className="hero-3d">
            <div className="flex justify-center py-7 px-4">
                <SandboxScene panel={Math.min(420, window.innerWidth - 80)}/>
            </div>
            <div className="hero-cap">
                <span className="dot" aria-hidden="true"/>
                {t("live · A* running in your browser", "live · 브라우저에서 실행 중인 A*")}
            </div>
        </div>
    )
}

export default HeroSearch
