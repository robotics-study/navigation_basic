import {ComponentType} from "react";

// 영/한 두 언어 문자열 쌍. 알고리즘 제목·섹션 등 언어에 따라 바뀌는 메타데이터에 쓴다.
export interface Localized<T = string> {
    en: T,
    ko: T,
}

// 대분류(section) → 중분류(category) 2단 구조. section 은 저장소 최상위 카테고리
// (global_planning / local_planning / multi_agent)와 1:1 이고, category 는 그 안의
// 알고리즘 계열이다. search/sampling 은 global_planning 하위 디렉토리와 일치한다.
export type AlgoSection = "global" | "local" | "multi";
export type AlgoCategory = "search" | "sampling" | "local" | "multi";

export interface ISupportedExample {
    python?: boolean,
    "c++"?: boolean,
}

export interface IAlgoData {
    // URL 경로(/algo/<slug>)이자 configs/<category>/<slug>.yaml, 소스 파일명과 동일한 식별자.
    slug: string,
    title: Localized,
    category: AlgoCategory,
    supportedExample?: ISupportedExample,
    // 지연 로딩(React.lazy)된 컴포넌트일 수 있다. contents 가 없으면 아직 집필되지 않은 페이지.
    contents?: ComponentType,
    // 본문 major 섹션(h2) 제목 목록 — 사이드바/TOC/검색 인덱스가 공유한다.
    // 렌더된 헤딩 텍스트(현재 언어)와 문자열이 일치해야 앵커(slug)가 맞는다.
    sections?: Localized[],
}
