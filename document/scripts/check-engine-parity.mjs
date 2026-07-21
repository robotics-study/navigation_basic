// 웹 라이브 TS 엔진과 저장소 python demo 의 정밀 대조.
// public/data 의 py trace 에서 (파라미터, 시작/목표, 기대 metric)을 읽어 같은 입력으로
// TS 엔진을 돌리고 결과를 비교한다. exact=true 인 엔진은 연산·tie-break 까지 미러라
// 확장 수까지 일치해야 하고, 나머지는 tie-break 가 달라 비용(최적성)만 비교한다.
// 실행: node scripts/check-engine-parity.mjs   (사전에 esbuild 로 엔진을 번들한다)
import {execFileSync} from "node:child_process";
import {gunzipSync} from "node:zlib";
import {mkdtempSync, readFileSync, rmSync} from "node:fs";
import {tmpdir} from "node:os";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";
import {createRequire} from "node:module";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = mkdtempSync(join(tmpdir(), "parity-"));
const bundle = join(outDir, "engines.cjs");
execFileSync(join(root, "node_modules", ".bin", "esbuild"), [
    join(root, "scripts", "parity-entry.ts"),
    "--bundle", "--format=cjs", "--platform=node", `--outfile=${bundle}`,
    "--loader:.ts=ts",
], {stdio: "pipe"});
const engines = createRequire(import.meta.url)(bundle);

const loadMap = (name) => engines.parseGridMap(
    JSON.parse(readFileSync(join(root, "public", "data", "maps", `${name}.json`), "utf-8")));
const loadTrace = (algo, name) =>
    gunzipSync(readFileSync(join(root, "public", "data", "traces", algo, `${name}.py.jsonl.gz`)))
        .toString("utf-8").trim().split("\n").map((l) => JSON.parse(l));

const finalOf = (events) => events[events.length - 1];
const lastPath = (events) => {
    for (let i = events.length - 1; i >= 0; i--) {
        if (events[i].event === "path_found") return events[i].path;
    }
    return null;
};

// (algo, map) → TS 엔진 실행. params 는 py trace 의 planning_started 스냅샷.
const RUNNERS = {
    astar: (m, s, g, p) => engines.runAStar(
        {map: m, start: s, goal: g, heuristicWeight: p.heuristic_weight ?? 1, connectivity: 8}),
    bfs: (m, s, g) => engines.runBFS({map: m, start: s, goal: g, connectivity: 8}),
    dijkstra: (m, s, g) => engines.runAStar(
        {map: m, start: s, goal: g, heuristicWeight: 0, connectivity: 8}),
    dstar_lite: (m, s, g, p) => engines.runDStarLite(
        {map: m, start: s, goal: g, sensorRadius: p.sensor_radius ?? 3}).events,
    ara_star: (m, s, g, p) => engines.runARAStar(
        {map: m, start: s, goal: g, epsStart: p.eps_start ?? 2.5,
         epsFinal: p.eps_final ?? 1, epsStep: p.eps_step ?? 0.5}).events,
    ad_star: (m, s, g, p) => engines.runADStar(
        {map: m, start: s, goal: g, epsStart: p.eps_start ?? 2.5,
         epsFinal: p.eps_final ?? 1, epsStep: p.eps_step ?? 0.5,
         sensorRadius: p.sensor_radius ?? 3}),
    theta_star: (m, s, g, p) => engines.runThetaStar(
        {map: m, start: s, goal: g, heuristicWeight: p.heuristic_weight ?? 1}),
    lazy_theta_star: (m, s, g, p) => engines.runLazyThetaStar(
        {map: m, start: s, goal: g, heuristicWeight: p.heuristic_weight ?? 1}),
};

// exact: 연산 순서·tie-break 까지 py 를 미러 → expanded_nodes 도 일치해야 한다.
const CHECKS = [
    {algo: "astar", maps: ["maze01", "open01"], exact: false},
    {algo: "bfs", maps: ["maze01", "open01", "bfs_hopcost01"], exact: true},
    {algo: "dijkstra", maps: ["maze01", "open01"], exact: false},
    {algo: "dstar_lite", maps: ["dstar_trap01", "maze01"], exact: true},
    {algo: "ara_star", maps: ["wastar_greedy01", "maze01"], exact: false},
    {algo: "ad_star", maps: ["dstar_trap01", "maze01"], exact: true},
    {algo: "theta_star", maps: ["maze01", "open01"], exact: true},
    {algo: "lazy_theta_star", maps: ["maze01", "open01"], exact: true},
];

let failures = 0;
for (const {algo, maps, exact} of CHECKS) {
    for (const name of maps) {
        let events;
        try {
            events = loadTrace(algo, name);
        } catch {
            continue;   // 해당 맵의 trace 미탑재 (dijkstra 등은 astar 와 같은 맵만 검사)
        }
        const started = events[0];
        const expected = finalOf(events);
        const path = lastPath(events);
        if (!path) continue;
        const map = loadMap(name);
        const start = path[0];
        const goal = path[path.length - 1];
        const got = finalOf(RUNNERS[algo](map, start, goal, started.params ?? {}));

        const problems = [];
        if (Boolean(got.success) !== Boolean(expected.success)) {
            problems.push(`success ${got.success} != ${expected.success}`);
        }
        const costA = got.metrics?.path_cost ?? 0;
        const costB = expected.metrics?.path_cost ?? 0;
        if (Math.abs(costA - costB) > 1e-6) {
            problems.push(`path_cost ${costA} != ${costB}`);
        }
        if (exact) {
            const expA = got.metrics?.expanded_nodes ?? -1;
            const expB = expected.metrics?.expanded_nodes ?? -2;
            if (expA !== expB) problems.push(`expanded ${expA} != ${expB}`);
        }
        const tag = `${algo} × ${name}${exact ? " [exact]" : ""}`;
        if (problems.length) {
            failures++;
            console.log(`FAIL ${tag}: ${problems.join("; ")}`);
        } else {
            console.log(`ok   ${tag}  cost=${costB.toFixed(4)} expanded=${expected.metrics?.expanded_nodes}`);
        }
    }
}
rmSync(outDir, {recursive: true, force: true});
if (failures) {
    console.error(`\n${failures} parity failure(s)`);
    process.exit(1);
}
console.log("\nall engines match the repository demos");
