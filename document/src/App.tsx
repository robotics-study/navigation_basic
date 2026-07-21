import {Suspense, useCallback, useEffect, useMemo, useState} from "react";
import algorithms from "./pages/algorithms";
import {applyPageMeta, algoMeta} from "./libs/seo";
import Header from "./components/Header";
import Sidebar from "./components/Sidebar";
import Toc from "./components/Toc";
import Footer from "./components/Footer";
import Home from "./pages/home/Home";
import AlgorithmContents from "./components/AlgorithmContents";
import {BASE_PATH} from "./libs/url";
import {useNavigate, useLocation, BrowserRouter, Routes, Route} from "react-router-dom";
import {useAlgoNav} from "./libs/nav";
import cn from "./libs/cn";
import {LangProvider, useLang} from "./libs/i18n";

const PageSelector = () => {
    const navigate = useNavigate()
    const location = useLocation()
    const {lang} = useLang()
    const {current: slug} = useAlgoNav()
    const [menuOpen, setMenuOpen] = useState(false)
    const closeMenu = useCallback(() => setMenuOpen(false), [])

    // 예전 정적 사이트 링크(/en/algorithms/astar.html 등)는 새 경로로 정리해 준다.
    useEffect(() => {
        const legacy = location.pathname.match(/^\/(en|ko)\/algorithms\/([a-z0-9_]+)(?:\.html)?$/)
        if (legacy) {
            const search = legacy[1] === "ko" ? "?lang=ko" : ""
            navigate({pathname: `/algo/${legacy[2]}`, search}, {replace: true})
        }
    }, [location.pathname, navigate])

    const current = useMemo(
        () => algorithms.find((item) => item.slug === slug && item.contents),
        [slug],
    )

    // 페이지 전환·언어 전환마다 제목·메타를 현재 뷰에 맞춘다 (SPA 이므로 크롤러/프리뷰용 갱신).
    useEffect(() => {
        applyPageMeta(algoMeta(lang, current))
    }, [current, lang])

    return (
        <>
            <Header onMenu={() => setMenuOpen((o) => !o)} showMenu={!!current}/>
            {current ? (
                <>
                    <div className="layout">
                        <Sidebar open={menuOpen} onNavigate={closeMenu}/>
                        <Suspense fallback={
                            <main className="content">
                                <div className="grid place-items-center py-24 text-muted text-sm">Loading…</div>
                            </main>
                        }>
                            <AlgorithmContents {...current}/>
                        </Suspense>
                        <Toc pageKey={current.slug}/>
                    </div>
                    <div className={cn("backdrop", menuOpen && "open")} onClick={closeMenu}/>
                </>
            ) : (
                <Home/>
            )}
            <Footer/>
        </>
    )
}

const App = () => {
    return <BrowserRouter basename={BASE_PATH || "/"}
                          future={{v7_startTransition: true, v7_relativeSplatPath: true}}>
        <LangProvider>
            <Routes>
                <Route path={"/"} element={<PageSelector/>}/>
                <Route path={"/algo/:slug"} element={<PageSelector/>}/>
                <Route path={"*"} element={<PageSelector/>}/>
            </Routes>
        </LangProvider>
    </BrowserRouter>
}

export default App
