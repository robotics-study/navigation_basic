import {ComponentType, lazy} from "react";
import {AlgoSection, Localized} from "../../../types/global";

// 대분류 소개 페이지 레지스트리. 알고리즘 각론 전에 "global planning 이란 무엇인가"류의
// 개념 설명을 담는다. sections 의 en/ko 문자열은 렌더된 본문 h2 헤딩과 정확히 일치해야
// TOC/검색 앵커(slug)가 맞는다. 집필된 페이지는 멀티라인 리터럴로 쓴다 (스크립트 파싱 규약).
export interface ISectionIntro {
    key: AlgoSection;
    contents: ComponentType;
    sections: Localized[];
}

const data: ISectionIntro[] = [
    {
        key: "global",
        contents: lazy(() => import("./GlobalPlanning")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Where It Sits in the Navigation Stack", ko: "Navigation Stack 에서의 위치"},
            {en: "Two Families: Search and Sampling", ko: "두 계열: Search 와 Sampling"},
            {en: "How This Repository Abstracts It", ko: "이 저장소의 추상화"},
            {en: "Suggested Reading Order", ko: "권장 읽기 순서"},
        ],
    },
    {
        key: "local",
        contents: lazy(() => import("./LocalPlanning")),
        sections: [
            {en: "Why a Second Planner", ko: "왜 planner 가 하나 더 필요한가"},
            {en: "The Problem", ko: "문제 정의"},
            {en: "Families of Local Planners", ko: "Local planner 의 계열"},
            {en: "What Is Coming", ko: "구현 예정"},
        ],
    },
    {
        key: "multi",
        contents: lazy(() => import("./MultiAgent")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Why It Is Hard", ko: "왜 어려운가"},
            {en: "Decoupled, Coupled, and In Between", ko: "Decoupled, Coupled, 그리고 그 사이"},
            {en: "What Is Coming", ko: "구현 예정"},
        ],
    },
]

export default data
