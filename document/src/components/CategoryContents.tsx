import 'katex/dist/katex.min.css';
import algorithms from "../pages/algorithms";
import {ICategoryIntro} from "../pages/categories";
import {CATEGORIES, SECTIONS, sectionOf} from "../pages/algorithms/roadmap";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";

// 중분류(카테고리) 소개 페이지 래퍼. pager의 이전은 소속 대분류의 소개,
// 다음은 이 카테고리의 첫 집필 알고리즘이다 (없으면 숨김).
const CategoryContents = ({intro}: {intro: ICategoryIntro}) => {
    const {go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    const category = CATEGORIES.find((c) => c.key === intro.key)!
    const section = SECTIONS.find((s) => s.key === sectionOf(intro.key))!
    const Contents = intro.contents
    const firstAlgo = algorithms.find((a) => a.category === intro.key && a.contents)

    return (
        <main className="content">
            <article className="content-inner">
                <p className="eyebrow">{pick(lang, section.title)} · {t("Overview", "개요")}</p>
                <h1>{pick(lang, category.title)}</h1>

                <Contents/>

                <nav className="pager">
                    <a onClick={() => goSection(section.key)}>
                        <div className="dir">{t("← Prev", "← 이전")}</div>
                        <div className="ttl">{pick(lang, section.title)} · Introduction</div>
                    </a>
                    {firstAlgo
                        ? <a className="next" onClick={() => go(firstAlgo.slug)}>
                            <div className="dir">{t("Next →", "다음 →")}</div>
                            <div className="ttl">{pick(lang, firstAlgo.title)}</div>
                        </a>
                        : <span/>}
                </nav>
            </article>
        </main>
    )
}

export default CategoryContents
