import {ReactNode, useMemo} from "react";

// 의존성 없는 최소 syntax highlighter. 정확한 파서가 아니라 읽기 보조가 목적이라
// comment / string / keyword / number / 전처리·데코레이터만 구분한다.
// 색은 전역 CSS 의 --tok-* 변수를 그대로 쓴다 (라이트/다크 자동 대응).
export type CodeLang = "python" | "cpp";

const KEYWORDS: Record<CodeLang, Set<string>> = {
    python: new Set([
        "def", "class", "return", "if", "elif", "else", "for", "while", "in", "not",
        "and", "or", "import", "from", "as", "with", "try", "except", "finally",
        "raise", "pass", "break", "continue", "lambda", "yield", "None", "True",
        "False", "is", "global", "nonlocal", "assert", "del", "async", "await", "self",
    ]),
    cpp: new Set([
        "auto", "bool", "break", "case", "catch", "char", "class", "const",
        "constexpr", "continue", "default", "delete", "do", "double", "else", "enum",
        "explicit", "extern", "final", "float", "for", "friend", "if", "inline",
        "int", "long", "namespace", "new", "noexcept", "nullptr", "operator",
        "override", "private", "protected", "public", "return", "short", "signed",
        "sizeof", "static", "struct", "switch", "template", "this", "throw", "true",
        "false", "try", "typedef", "typename", "union", "unsigned", "using",
        "virtual", "void", "volatile", "while", "size_t", "uint32_t", "int64_t",
    ]),
};

// 언어별 마스터 토큰 정규식 — 순서가 우선순위다 (comment > string > number > word).
const PATTERNS: Record<CodeLang, RegExp> = {
    python: /(#[^\n]*)|("""[\s\S]*?"""|'''[\s\S]*?''')|("(?:[^"\\\n]|\\.)*"|'(?:[^'\\\n]|\\.)*')|(@\w+)|(\b\d+(?:\.\d+)?(?:e[+-]?\d+)?\b)|([A-Za-z_]\w*)/g,
    cpp: /(\/\/[^\n]*|\/\*[\s\S]*?\*\/)|(^\s*#\s*\w+[^\n]*)|("(?:[^"\\\n]|\\.)*"|'(?:[^'\\\n]|\\.)*')|(\b\d+(?:\.\d+)?(?:e[+-]?\d+)?f?\b)|([A-Za-z_]\w*)/gm,
};

const STYLE: Record<string, string> = {
    comment: "var(--tok-comment)",
    string: "var(--tok-string)",
    keyword: "var(--tok-tag)",
    number: "var(--tok-expr)",
    meta: "var(--tok-attr)",       // 데코레이터 / 전처리 지시자
}

function classify(match: RegExpExecArray, lang: CodeLang): string | null {
    if (lang === "python") {
        const [, comment, triString, string, deco, num, word] = match
        if (comment) return "comment"
        if (triString) return "comment"   // docstring 은 주석 취급이 읽기에 자연스럽다
        if (string) return "string"
        if (deco) return "meta"
        if (num) return "number"
        if (word) return KEYWORDS.python.has(word) ? "keyword" : null
        return null
    }
    const [, comment, preproc, string, num, word] = match
    if (comment) return "comment"
    if (preproc) return "meta"
    if (string) return "string"
    if (num) return "number"
    if (word) return KEYWORDS.cpp.has(word) ? "keyword" : null
    return null
}

export function highlight(code: string, lang: CodeLang): ReactNode[] {
    const out: ReactNode[] = []
    const re = new RegExp(PATTERNS[lang].source, PATTERNS[lang].flags)
    let last = 0
    let key = 0
    for (let m = re.exec(code); m !== null; m = re.exec(code)) {
        if (m.index > last) out.push(code.slice(last, m.index))
        const cls = classify(m, lang)
        out.push(cls
            ? <span key={key++} style={{color: STYLE[cls]}}>{m[0]}</span>
            : m[0])
        last = m.index + m[0].length
    }
    if (last < code.length) out.push(code.slice(last))
    return out
}

const CodeHighlight = ({code, lang}: {code: string; lang: CodeLang}) => {
    const nodes = useMemo(() => highlight(code, lang), [code, lang])
    return <>{nodes}</>
}

export default CodeHighlight
