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
    jps: (m, s, g) => engines.runJPS({map: m, start: s, goal: g}),
    visibility_astar: (m, s, g, p) => engines.runVisibilityAStar(
        {map: m, start: s, goal: g, heuristicWeight: p.heuristic_weight ?? 1}),
    anya: (m, s, g, p) => engines.runAnya(
        {map: m, start: s, goal: g, vertexEpsilon: p.vertex_epsilon ?? 1e-9}),
    prm: (m, s, g, p) => engines.runPRM(
        {map: m, start: s, goal: g, numSamples: p.num_samples ?? 250,
         connectionRadius: p.connection_radius ?? 2, seed: p.seed ?? 1}),
    rrt: (m, s, g, p) => engines.runRRT(
        {map: m, start: s, goal: g, maxIterations: p.max_iterations ?? 5000,
         stepSize: p.step_size ?? 0.5, goalBias: p.goal_bias ?? 0.05,
         goalTolerance: p.goal_tolerance ?? 0.3, seed: p.seed ?? 1}),
    rrt_connect: (m, s, g, p) => engines.runRRTConnect(
        {map: m, start: s, goal: g, maxIterations: p.max_iterations ?? 4000,
         stepSize: p.step_size ?? 0.5, goalTolerance: p.goal_tolerance ?? 0.3,
         seed: p.seed ?? 1}),
    rrt_star: (m, s, g, p) => engines.runRRTStar(
        {map: m, start: s, goal: g, maxIterations: p.max_iterations ?? 8000,
         stepSize: p.step_size ?? 0.5, goalBias: p.goal_bias ?? 0.05,
         goalTolerance: p.goal_tolerance ?? 0.3,
         neighborRadius: p.neighbor_radius ?? 1.5,
         radiusMode: p.radius_mode ?? "fixed", rggGamma: p.rgg_gamma ?? 2,
         seed: p.seed ?? 1}),
    informed_rrt_star: (m, s, g, p) => engines.runInformedRRTStar(
        {map: m, start: s, goal: g, maxIterations: p.max_iterations ?? 8000,
         stepSize: p.step_size ?? 0.5, goalBias: p.goal_bias ?? 0.05,
         goalTolerance: p.goal_tolerance ?? 0.3,
         neighborRadius: p.neighbor_radius ?? 1.5,
         radiusMode: p.radius_mode ?? "fixed", rggGamma: p.rgg_gamma ?? 2,
         seed: p.seed ?? 1}),
    fast_rrt: (m, s, g, p) => engines.runFastRRT(
        {map: m, start: s, goal: g, maxIterations: p.max_iterations ?? 650,
         stepSize: p.step_size ?? 0.5, goalBias: p.goal_bias ?? 0.05,
         goalTolerance: p.goal_tolerance ?? 0.3,
         neighborRadius: p.neighbor_radius ?? 1.5,
         radiusMode: p.radius_mode ?? "fixed", rggGamma: p.rgg_gamma ?? 2,
         reachedRadius: p.reached_radius ?? 0.4, steeringAttempts: p.steering_attempts ?? 10,
         seed: p.seed ?? 1}),
    fmt_star: (m, s, g, p) => engines.runFMTStar(
        {map: m, start: s, goal: g, numSamples: p.num_samples ?? 1500,
         gamma: p.gamma ?? 30, seed: p.seed ?? 1}),
    bit_star: (m, s, g, p) => engines.runBITStar(
        {map: m, start: s, goal: g, batchSize: p.batch_size ?? 200,
         maxBatches: p.max_batches ?? 15, gamma: p.gamma ?? 30, seed: p.seed ?? 1}),
    abit_star: (m, s, g, p) => engines.runABITStar(
        {map: m, start: s, goal: g, batchSize: p.batch_size ?? 200,
         maxBatches: p.max_batches ?? 15, gamma: p.gamma ?? 30,
         inflationFactor: p.inflation_factor ?? 10, inflationFinal: p.inflation_final ?? 1,
         truncationFactor: p.truncation_factor ?? 2, seed: p.seed ?? 1}),
    ait_star: (m, s, g, p) => engines.runAITStar(
        {map: m, start: s, goal: g, batchSize: p.batch_size ?? 200,
         maxBatches: p.max_batches ?? 15, gamma: p.gamma ?? 30, seed: p.seed ?? 1}),
    eit_star: (m, s, g, p) => engines.runEITStar(
        {map: m, start: s, goal: g, batchSize: p.batch_size ?? 200,
         maxBatches: p.max_batches ?? 15, gamma: p.gamma ?? 30,
         stepSize: p.step_size ?? 0.5, seed: p.seed ?? 1}),
    fcit_star: (m, s, g, p) => engines.runFCITStar(
        {map: m, start: s, goal: g, batchSize: p.batch_size ?? 40,
         maxBatches: p.max_batches ?? 5, seed: p.seed ?? 1}),
    // SST 경로는 goal_tol 안 waypoint로 끝나므로 시나리오 goal을 하드코딩한다
    // (hybrid_astar와 같은 이유; 두 맵 시나리오 goal 동일).
    sst: (m, s, g, p) => engines.runSST(
        {map: m, start: s, goal: [9.25, 9.25],
         maxIterations: p.max_iterations ?? 30000, goalBias: p.goal_bias ?? 0.1,
         goalTolerance: p.goal_tolerance ?? 0.6, deltaBn: p.delta_bn ?? 1.2,
         deltaS: p.delta_s ?? 0.5, maxVelocity: p.max_velocity ?? 1.5,
         maxOmega: p.max_omega ?? 1.5, propDurationMin: p.prop_duration_min ?? 0.2,
         propDurationMax: p.prop_duration_max ?? 0.8,
         footprintRadius: p.footprint_radius ?? 0.15, sstStar: p.sst_star ?? false,
         seed: p.seed ?? 1}),
    kinodynamic_rrt_star: (m, s, g, p) => engines.runKinodynamicRRTStar(
        {map: m, start: s, goal: g, maxIterations: p.max_iterations ?? 4000,
         goalBias: p.goal_bias ?? 0.1, goalTolerance: p.goal_tolerance ?? 1.0,
         neighborRadius: p.neighbor_radius ?? 2.0, controlWeight: p.control_weight ?? 1.0,
         maxVelocity: p.max_velocity ?? 1.5,
         footprintRadius: p.footprint_radius ?? 0.15, seed: p.seed ?? 1}),
    // LQR-RRT* 경로는 goal rest 상태에 스냅되어 끝나지만 SST/hybrid와 동일하게
    // 시나리오 goal을 하드코딩한다 (두 맵 시나리오 goal 동일).
    lqr_rrt_star: (m, s, g, p) => engines.runLQRRRTStar(
        {map: m, start: s, goal: [9.25, 9.25],
         maxIterations: p.max_iterations ?? 600, stepSize: p.step_size ?? 1.5,
         goalBias: p.goal_bias ?? 0.1, goalTolerance: p.goal_tolerance ?? 1.0,
         neighborRadius: p.neighbor_radius ?? 2.0, qPos: p.q_pos ?? 1.0,
         qVel: p.q_vel ?? 1.0, rCtrl: p.r_ctrl ?? 1.0, lqrDt: p.lqr_dt ?? 0.2,
         controlLimit: p.control_limit ?? 10.0, maxVelocity: p.max_velocity ?? 1.5,
         footprintRadius: p.footprint_radius ?? 0.15, seed: p.seed ?? 1}),
    prm_star: (m, s, g, p) => engines.runPRMStar(
        {map: m, start: s, goal: g, numSamples: p.num_samples ?? 250,
         gamma: p.gamma ?? 30, seed: p.seed ?? 1}),
    // 연속 SE(2): start 는 trace 경로의 첫 pose, goal 은 시나리오 pose (theta 기본 0).
    hybrid_astar: (m, s, g, p) => engines.runHybridAStar({
        map: m, start: s, goal: [9.25, 9.25, 0],
        minTurnRadius: p.min_turn_radius, arcStep: p.arc_step,
        numSteering: p.num_steering, thetaBins: p.theta_bins,
        xyResolution: p.xy_resolution, footprintRadius: p.footprint_radius,
        allowReverse: p.allow_reverse, reversePenalty: p.reverse_penalty,
        steerPenalty: p.steer_penalty, goalPosTolerance: p.goal_pos_tolerance,
        goalHeadingTolerance: p.goal_heading_tolerance,
    }),
    // 폐루프 local planner 3종: start 는 trace 경로(robot_moved 궤적)의 첫 pose를 그대로
    // 쓴다(RobotState 라 hybrid_astar처럼 goal도 포함된 3-tuple) — goal 은 trace에 없으므로
    // 시나리오 yaml 값을 하드코딩한다(sst/lqr_rrt_star와 같은 이유).
    // 시나리오: maps/scenarios/clutter01_s1.yaml (potential_fields·vfh 공용).
    potential_fields: (m, s, g, p) => engines.runPotentialFields({
        map: m, start: s, goal: [9.25, 9.25],
        kAtt: p.k_att, kRep: p.k_rep, influenceRadius: p.influence_radius,
        kV: p.k_v, kOmega: p.k_omega, maxSpeed: p.max_speed, maxOmega: p.max_omega,
        footprintRadius: p.footprint_radius, controlDt: p.control_dt, maxSteps: p.max_steps,
        goalTolerance: p.goal_tolerance, stallWindow: p.stall_window, stallDistance: p.stall_distance,
    }),
    vfh: (m, s, g, p) => engines.runVfh({
        map: m, start: s, goal: [9.25, 9.25],
        numSectors: p.num_sectors, windowRadius: p.window_radius, threshold: p.threshold,
        smoothingWindow: p.smoothing_window, wideValleySectors: p.wide_valley_sectors,
        hM: p.h_m, kOmega: p.k_omega, maxSpeed: p.max_speed, maxOmega: p.max_omega,
        controlDt: p.control_dt, maxSteps: p.max_steps, goalTolerance: p.goal_tolerance,
        footprintRadius: p.footprint_radius, stallWindow: p.stall_window, stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/clutter01_s1.yaml (DWA 는 참조 경로 불요 — goal 만).
    dwa: (m, s, g, p) => engines.runDwa({
        map: m, startPose: s, goal: [9.25, 9.25],
        maxSpeed: p.max_speed, minSpeed: p.min_speed, maxOmega: p.max_omega,
        accel: p.accel, accelOmega: p.accel_omega,
        vSamples: p.v_samples, omegaSamples: p.omega_samples,
        simTime: p.sim_time, simSteps: p.sim_steps,
        headingWeight: p.heading_weight, clearanceWeight: p.clearance_weight,
        velocityWeight: p.velocity_weight, clearanceLimit: p.clearance_limit,
        slowRadius: p.slow_radius, controlDt: p.control_dt, maxSteps: p.max_steps,
        goalTolerance: p.goal_tolerance, footprintRadius: p.footprint_radius,
        stallWindow: p.stall_window, stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/open01_s2.yaml (goal + reference_path 전부 이 파일 값).
    pure_pursuit: (m, s, g, p) => engines.runPurePursuit({
        map: m, startPose: s, goal: [9.0, 9.0],
        referencePath: [
            [1.0, 1.0], [2.0, 1.0], [3.5, 1.2], [5.0, 1.5], [5.0, 3.5],
            [5.0, 6.0], [5.0, 8.5], [7.0, 8.8], [8.7, 9.0], [9.0, 9.0],
        ],
        lookaheadDistance: p.lookahead_distance, maxSpeed: p.max_speed, maxOmega: p.max_omega,
        slowRadius: p.slow_radius, controlDt: p.control_dt, maxSteps: p.max_steps,
        goalTolerance: p.goal_tolerance, footprintRadius: p.footprint_radius,
        stallWindow: p.stall_window, stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/open01_s3.yaml (오프셋 시작 — goal + reference_path 이 파일 값).
    stanley: (m, s, g, p) => engines.runStanley({
        map: m, startPose: s, goal: [9.0, 9.0],
        referencePath: [
            [1.0, 1.0], [2.0, 1.0], [3.5, 1.2], [5.0, 1.5], [5.0, 3.5],
            [5.0, 6.0], [5.0, 8.5], [7.0, 8.8], [8.7, 9.0], [9.0, 9.0],
        ],
        kGain: p.k_gain, kSoft: p.k_soft, wheelbase: p.wheelbase, maxSteer: p.max_steer,
        maxSpeed: p.max_speed, maxOmega: p.max_omega, slowRadius: p.slow_radius,
        controlDt: p.control_dt, maxSteps: p.max_steps, goalTolerance: p.goal_tolerance,
        footprintRadius: p.footprint_radius, stallWindow: p.stall_window,
        stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/clutter01_s2.yaml (goal + reference_path 이 파일 값).
    regulated_pure_pursuit: (m, s, g, p) => engines.runRegulatedPurePursuit({
        map: m, startPose: s, goal: [9.25, 9.25],
        referencePath: [
            [0.75, 0.75], [1.0, 4.0], [1.0, 7.5], [1.6, 8.6],
            [4.0, 8.75], [7.3, 8.75], [9.25, 9.25],
        ],
        lookaheadTime: p.lookahead_time, minLookahead: p.min_lookahead,
        maxLookahead: p.max_lookahead, regulatedMinRadius: p.regulated_min_radius,
        proximityDistance: p.proximity_distance, minRegulatedSpeed: p.min_regulated_speed,
        collisionCheckStep: p.collision_check_step, maxSpeed: p.max_speed,
        maxOmega: p.max_omega, slowRadius: p.slow_radius,
        controlDt: p.control_dt, maxSteps: p.max_steps, goalTolerance: p.goal_tolerance,
        footprintRadius: p.footprint_radius, stallWindow: p.stall_window,
        stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/clutter01_s3.yaml (Elastic Bands 전용 -- 준대각 경로가
    // 장애물 블록 5곳을 중심에서 비켜 관통해 밴드가 밀려 우회함을 시연).
    elastic_bands: (m, s, g, p) => engines.runElasticBands({
        map: m, startPose: s, goal: [9.25, 9.25],
        referencePath: [[0.75, 0.75], [3.0, 3.0], [5.4, 4.9], [7.7, 7.9], [9.25, 9.25]],
        kContraction: p.k_contraction, kRepulsion: p.k_repulsion, rhoMax: p.rho_max,
        rhoInfluence: p.rho_influence, rhoMin: p.rho_min, stepSize: p.step_size,
        deformIterations: p.deform_iterations, repairIterations: p.repair_iterations,
        repairStep: p.repair_step, overlapFactor: p.overlap_factor, maxBubbles: p.max_bubbles,
        bubbleSpacing: p.bubble_spacing, lookaheadDistance: p.lookahead_distance,
        headingGain: p.heading_gain, maxSpeed: p.v_max, maxOmega: p.omega_max,
        controlDt: p.control_dt, maxSteps: p.max_steps, goalTolerance: p.goal_tolerance,
        footprintRadius: p.footprint_radius, stallWindow: p.stall_window,
        stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/clutter01_s2.yaml (regulated_pure_pursuit과 공유 -- 장애물
    // 상단을 스치는 경로. goal_theta는 시나리오에 없어 0.0으로 기본).
    teb: (m, s, g, p) => engines.runTeb({
        map: m, startPose: s, goal: [9.25, 9.25, 0],
        referencePath: [
            [0.75, 0.75], [1.0, 4.0], [1.0, 7.5], [1.6, 8.6],
            [4.0, 8.75], [7.3, 8.75], [9.25, 9.25],
        ],
        wPath: p.w_path, wObstacle: p.w_obstacle, wVelocity: p.w_velocity,
        wAcceleration: p.w_acceleration, wTime: p.w_time, wKinematics: p.w_kinematics,
        maxSpeed: p.v_max, maxOmega: p.omega_max, aMax: p.a_max,
        minObstacleDist: p.min_obstacle_dist, dtRef: p.dt_ref, dtMin: p.dt_min,
        horizon: p.horizon, iterations: p.iterations, stepAlpha: p.step_alpha,
        maxStepXy: p.max_step_xy, maxStepTheta: p.max_step_theta, maxStepDt: p.max_step_dt,
        maxPoses: p.max_poses, reinitDistance: p.reinit_distance,
        controlDt: p.control_dt, maxSteps: p.max_steps, goalTolerance: p.goal_tolerance,
        footprintRadius: p.footprint_radius, stallWindow: p.stall_window,
        stallDistance: p.stall_distance,
    }),
    // 시나리오: maps/scenarios/open01_s4.yaml (goal-seeking reach -- goal만, reference_path
    // 없음. 벽 penalty zone 밖 내부 goal이라 MPC가 폐루프로 안정 도달한다).
    mpc: (m, s, g, p) => engines.runMpc({
        map: m, startPose: s, goal: [8.0, 8.0],
        horizon: p.horizon, iterations: p.iterations, stepAlpha: p.step_alpha, gradEps: p.grad_eps,
        maxStepV: p.max_step_v, maxStepOmega: p.max_step_omega, wGoal: p.w_goal, wObstacle: p.w_obstacle,
        wControl: p.w_control, minObstacleDist: p.min_obstacle_dist, vMax: p.v_max, omegaMax: p.omega_max,
        aMax: p.a_max, controlDt: p.control_dt, maxSteps: p.max_steps, goalTolerance: p.goal_tolerance,
        footprintRadius: p.footprint_radius, stallWindow: p.stall_window, stallDistance: p.stall_distance,
    }),
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
    {algo: "jps", maps: ["maze01", "open01"], exact: true},
    {algo: "visibility_astar", maps: ["maze01", "open01"], exact: true},
    {algo: "anya", maps: ["maze01", "open01"], exact: true},
    // sampling 계열: numpy PCG64 RNG를 미러해 표본열까지 동일 → exact.
    {algo: "prm", maps: ["maze01", "open01"], exact: true},
    {algo: "prm_star", maps: ["maze01", "open01"], exact: true},
    {algo: "rrt", maps: ["maze01", "open01"], exact: true},
    {algo: "rrt_connect", maps: ["maze01", "open01"], exact: true},
    {algo: "rrt_star", maps: ["maze01", "open01"], exact: true},
    {algo: "informed_rrt_star", maps: ["maze01", "open01"], exact: true},
    {algo: "fast_rrt", maps: ["maze01", "open01"], exact: true},
    {algo: "fmt_star", maps: ["maze01", "open01"], exact: true},
    {algo: "bit_star", maps: ["maze01", "open01"], exact: true},
    {algo: "abit_star", maps: ["maze01", "open01"], exact: true},
    {algo: "ait_star", maps: ["maze01", "open01"], exact: true},
    {algo: "eit_star", maps: ["maze01", "open01"], exact: true},
    {algo: "fcit_star", maps: ["maze01", "open01"], exact: true},
    {algo: "sst", maps: ["maze01", "open01"], exact: true},
    {algo: "kinodynamic_rrt_star", maps: ["maze01", "open01"], exact: true},
    {algo: "lqr_rrt_star", maps: ["maze01", "open01"], exact: true},
    // sin/cos 가 libm 구현마다 1 ULP 다를 수 있어 비용은 허용 오차로만 비교한다.
    {algo: "hybrid_astar", maps: ["open01", "maze01"], exact: false, costTol: 0.05},
    // 폐루프 local planner 3종: metricKeys 로 path_cost/expanded_nodes 대신 시뮬레이터
    // metrics를 비교한다. steps는 valley 선택·stall·goal 판정 등 폐루프의 모든 분기가
    // 반영된 tick 열이라 tol 0(정수 exact)이 성립한다. distance_traveled는 tick마다
    // sin/cos/atan2를 호출하는 폐루프라 libm/V8 fdlibm 1 ULP가 수백 tick 누적될 수 있어
    // hybrid_astar의 costTol과 같은 이유로 1e-3을 둔다. pf_trap01(정체 경계가 ULP
    // 민감)은 parity에서 제외 — python 테스트가 그 경계를 담당한다.
    {algo: "potential_fields", maps: ["clutter01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    {algo: "vfh", maps: ["clutter01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    {algo: "dwa", maps: ["clutter01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    {algo: "pure_pursuit", maps: ["open01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    {algo: "stanley", maps: ["open01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    {algo: "regulated_pure_pursuit", maps: ["clutter01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    {algo: "elastic_bands", maps: ["clutter01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
    // TEB만 tol을 완화한다: 매 tick 40회 damped gradient descent를 돌리고, 그 안의
    // kinematics 항이 매 반복마다 모든 내부 pose의 sin/cos를 재계산한다 -- 위 3종보다
    // 자릿수 앞자리로 몇 단위 더 많은 sin/cos 호출이 한 tick 안에 몰려 있다. 실측
    // 확인: 동일 theta 5000개를 넣고 비교하면 V8 Math.sin/cos이 Python math.sin/cos와
    // 약 2.5%의 입력에서 1ULP 어긋난다(둘 다 IEEE 754를 지키지만 서로 다른 다항식
    // 근사를 쓴다). 단일 gradient_step 반복은 이 스크립트로 직접 대조해 입력이 같으면
    // 출력도 bit-identical함을 확인했으므로 수식 자체는 정확한 미러다 -- 하지만
    // resize()의 dt_i > 1.5*dt_ref 같은 이산 분기가 tick당 40회 x 최대 40개 pose에
    // 걸쳐 그 1ULP 오차를 반복 통과시키다 보면 어느 tick에서 분기 하나가 반대로
    // 갈리고, 그 뒤로는 서로 다르지만 둘 다 유효한 궤적으로 갈라진다(둘 다 REACHED,
    // 스텝 수 근접). tol 0은 이 알고리즘 규모에서 재현 불가능하므로, 무관한 회귀(경로
    // 미도달·스텝 수 급변·NaN)는 잡아내되 이 계층적 ULP 분기는 통과시킬 여유를 둔다.
    {algo: "teb", maps: ["clutter01"],
     metricKeys: [{key: "steps", tol: 8}, {key: "distance_traveled", tol: 0.2}]},
    // MPC는 매 tick iterations회 투영 경사하강을 돌리고 그 안의 central finite-difference
    // gradient가 반복마다 2H 성분 × ±grad_eps 섭동 rollout을 재계산해 tick당 수백 회의 sin/cos를
    // 몰아 호출한다 -- 하지만 open01_s4는 벽 penalty zone 밖 내부 goal을 향한 깨끗한 reach라
    // TEB의 clutter 미로 같은 이산 분기(resize 등)가 없어, 119 tick 내내 clamp 분기가 ULP
    // 경계에서 뒤집히지 않는다. 실측 py↔TS 차이는 steps 0(정수 exact), distance_traveled 7.3e-11로,
    // pf/vfh/dwa/pure_pursuit 등 다른 결정론 폐루프 local planner와 같은 steps tol 0 / distance
    // 1e-3을 그대로 유지한다(1e-3은 tick마다의 fdlibm sin/cos 1ULP 누적을 흡수).
    {algo: "mpc", maps: ["open01"],
     metricKeys: [{key: "steps", tol: 0}, {key: "distance_traveled", tol: 1e-3}]},
];

let failures = 0;
for (const check of CHECKS) {
    const {algo, maps, exact} = check;
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
        // metricKeys 모드(local planner): path_cost/expanded_nodes 대신 시뮬레이터
        // metrics(steps, distance_traveled 등)를 지정된 허용 오차로 비교한다.
        if (check.metricKeys) {
            for (const {key, tol} of check.metricKeys) {
                const a = got.metrics?.[key];
                const b = expected.metrics?.[key];
                if (a === undefined || b === undefined || Math.abs(a - b) > tol) {
                    problems.push(`${key} ${a} != ${b}`);
                }
            }
        }
        const costB = expected.metrics?.path_cost ?? 0;
        if (!check.metricKeys) {
            const costA = got.metrics?.path_cost ?? 0;
            const costTol = check.costTol ?? 1e-6;
            if (Math.abs(costA - costB) > costTol) {
                problems.push(`path_cost ${costA} != ${costB}`);
            }
            if (exact) {
                const expA = got.metrics?.expanded_nodes ?? -1;
                const expB = expected.metrics?.expanded_nodes ?? -2;
                if (expA !== expB) problems.push(`expanded ${expA} != ${expB}`);
            }
        }
        const tag = `${algo} × ${name}${exact ? " [exact]" : ""}${check.metricKeys ? " [metrics]" : ""}`;
        if (problems.length) {
            failures++;
            console.log(`FAIL ${tag}: ${problems.join("; ")}`);
        } else if (check.metricKeys) {
            const summary = check.metricKeys
                .map(({key}) => `${key}=${expected.metrics?.[key]}`).join(" ");
            console.log(`ok   ${tag}  ${summary}`);
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
