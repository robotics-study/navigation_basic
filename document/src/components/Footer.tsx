import {T} from "../libs/i18n";

const LICENSE_URL = "https://github.com/robotics-study/navigation/blob/main/LICENSE"
const REPO_URL = "https://github.com/robotics-study/navigation"

const Footer = () => (
    <footer className="site-footer">
        <T
            en={<p>
                Study notes on robot navigation algorithms — independent C++ / Python implementations ·{" "}
                <a href={REPO_URL} target="_blank" rel="noopener noreferrer">robotics-study/navigation</a>
            </p>}
            ko={<p>
                로봇 navigation 알고리즘을 공부하며 만든 노트 · C++ / Python 독립 이중 구현 ·{" "}
                <a href={REPO_URL} target="_blank" rel="noopener noreferrer">robotics-study/navigation</a>
            </p>}
        />
        <T
            en={<p>
                © 2026 robotics-study ·{" "}
                <a href={LICENSE_URL} target="_blank" rel="noopener noreferrer">MIT License</a>
                {" "}· Unofficial study notes
            </p>}
            ko={<p>
                © 2026 robotics-study ·{" "}
                <a href={LICENSE_URL} target="_blank" rel="noopener noreferrer">MIT License</a>
                {" "}· 비공식 학습 노트
            </p>}
        />
    </footer>
)

export default Footer
