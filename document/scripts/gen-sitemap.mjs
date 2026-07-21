// 빌드 전에 public/sitemap.xml 을 생성한다. 페이지 목록은 pages/algorithms/index.ts 에서
// 읽는다: 집필된 페이지는 멀티라인 리터럴(`{\n slug: ...` + contents:)로 쓰는 것이 레지스트리
// 컨벤션이라, 그 블록만 잡으면 미집필 한 줄 항목은 자연히 제외된다.
// 각 URL 에 en/ko hreflang 대체 링크를 함께 적는다.
import {readFileSync, writeFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const ORIGIN = "https://robotics-study.github.io";
const BASE = "/navigation/";

const indexTs = readFileSync(join(root, "src/pages/algorithms/index.ts"), "utf-8");
const slugs = indexTs
    .split(/\n\s*\{\s*\n\s*slug:/)
    .slice(1)
    .filter((block) => block.includes("contents:"))
    .map((block) => block.match(/^\s*"([a-z0-9_]+)"/)[1]);
if (slugs.length === 0) throw new Error("no written pages found in pages/algorithms/index.ts");

const url = (lang, slug) => {
    const path = slug !== undefined ? `algo/${slug}/` : "";
    const qs = lang === "ko" ? "?lang=ko" : "";
    return `${ORIGIN}${BASE}${path}${qs}`;
};

const esc = (s) => s.replaceAll("&", "&amp;");
const today = new Date().toISOString().slice(0, 10);

const entry = (lang, slug, priority) => `  <url>
    <loc>${esc(url(lang, slug))}</loc>
    <lastmod>${today}</lastmod>
    <priority>${priority}</priority>
    <xhtml:link rel="alternate" hreflang="en" href="${esc(url("en", slug))}"/>
    <xhtml:link rel="alternate" hreflang="ko" href="${esc(url("ko", slug))}"/>
    <xhtml:link rel="alternate" hreflang="x-default" href="${esc(url("en", slug))}"/>
  </url>`;

const entries = [
    entry("en", undefined, "1.0"),
    entry("ko", undefined, "1.0"),
    ...slugs.flatMap((s) => [entry("en", s, "0.8"), entry("ko", s, "0.8")]),
];

const xml = `<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9"
        xmlns:xhtml="http://www.w3.org/1999/xhtml">
${entries.join("\n")}
</urlset>
`;

writeFileSync(join(root, "public/sitemap.xml"), xml);
console.log(`sitemap.xml: ${entries.length} URLs (pages: ${slugs.join(", ")})`);
