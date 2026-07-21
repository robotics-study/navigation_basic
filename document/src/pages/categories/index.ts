import {ComponentType, lazy} from "react";
import {AlgoCategory, Localized} from "../../../types/global";

// 중분류(카테고리) 소개 페이지 레지스트리 — "Graph Search 란 무엇인가"류의 설명.
// 대분류와 이름이 같은 카테고리(local/multi)는 대분류 소개가 그 역할을 하므로 여기 없다.
// sections 의 en/ko 는 렌더된 본문 h2 와 정확히 일치해야 한다. 집필 페이지는 멀티라인
// 리터럴로 쓴다 (스크립트 파싱 규약).
export interface ICategoryIntro {
    key: AlgoCategory;
    contents: ComponentType;
    sections: Localized[];
}

const data: ICategoryIntro[] = [
    {
        key: "search",
        contents: lazy(() => import("./GraphSearch")),
        sections: [
            {en: "What Graph Search Is", ko: "Graph Search 란"},
            {en: "The Search Model", ko: "탐색 모델"},
            {en: "Guarantees", ko: "보장"},
            {en: "A Map of the Algorithms", ko: "알고리즘 지도"},
        ],
    },
    {
        key: "sampling",
        contents: lazy(() => import("./Sampling")),
        sections: [
            {en: "Why Sample", ko: "왜 sampling 인가"},
            {en: "The Building Blocks", ko: "구성 요소"},
            {en: "Trees and Roadmaps", ko: "Tree 와 Roadmap"},
            {en: "Guarantees", ko: "보장"},
            {en: "A Map of the Algorithms", ko: "알고리즘 지도"},
        ],
    },
]

export default data
