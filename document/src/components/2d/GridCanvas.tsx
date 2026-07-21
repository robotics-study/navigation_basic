import {useMemo, useRef} from "react";
import {Circle, Group, Layer, Line, Rect, Stage} from "react-konva";
import Konva from "konva";
import {GridMap, worldToCellUnits} from "../../libs/grid";
import {Cell, GridTimeline} from "../../libs/trace/timeline";
import {useCanvasColors} from "../../libs/useTheme";

// 경로/goal 강조색 — 라이트/다크 양쪽에서 accent(indigo)와 대비되는 warm red.
export const PATH_COLOR = "#e0533d";

interface GridCanvasProps {
    map: GridMap;
    // 가장 긴 변의 픽셀 크기. 셀 크기는 여기서 유도된다.
    panel: number;
    timeline?: GridTimeline;
    // 이 step 이하의 이벤트만 그린다 (재생/스크럽).
    step?: number;
    start?: Cell;
    goal?: Cell;
    showTree?: boolean;
    // 비교용 보조 경로 (점선, muted) — any-angle vs grid 경로 대조 등에 쓴다.
    overlayPath?: Cell[];
    // 비교용 배경 셀 집합 (muted 반투명) — 다른 알고리즘의 확장 영역 대조 등에 쓴다.
    shadowCells?: Cell[];
    // 연속 상태(SE(2)) planner 용 차량 표시 — carPose는 현재(주행 중) pose,
    // goalPose는 요구 heading을 보여 주는 점선 외곽선. 있으면 원 마커를 대체한다.
    carPose?: [number, number, number];
    goalPose?: [number, number, number];
    // sandbox 상호작용 — 핸들러가 있을 때만 활성화된다.
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
    onMoveStart?: (cell: Cell) => void;
    onMoveGoal?: (cell: Cell) => void;
}

const GridCanvas = ({
                        map, panel, timeline, step = Infinity, start, goal,
                        showTree = false, overlayPath, shadowCells, carPose, goalPose,
                        onPaintCell, onMoveStart, onMoveGoal,
                    }: GridCanvasProps) => {
    const colors = useCanvasColors();
    const cell = panel / Math.max(map.width, map.height);
    const stageW = Math.round(cell * map.width);
    const stageH = Math.round(cell * map.height);

    // 셀별 최초 candidate/expanded step — frontier(발견되었지만 아직 확장 전) 표시에 쓴다.
    const firstSeen = useMemo(() => {
        const cand = new Map<number, number>()
        const exp = new Map<number, number>()
        if (timeline && !timeline.continuous) {
            for (const c of timeline.candidates) {
                const key = c.cell[0] * map.width + c.cell[1]
                if (!cand.has(key)) cand.set(key, c.step)
            }
            for (const e of timeline.expanded) {
                const key = e.cell[0] * map.width + e.cell[1]
                if (!exp.has(key)) exp.set(key, e.step)
            }
        }
        return {cand, exp}
    }, [timeline, map.width])

    const expandedVisible = useMemo(
        () => timeline ? timeline.expanded.filter((e) => e.step <= step) : [],
        [timeline, step],
    )
    const frontierVisible = useMemo(() => {
        if (!timeline) return []
        const out: Cell[] = []
        firstSeen.cand.forEach((candStep, key) => {
            const expStep = firstSeen.exp.get(key) ?? Infinity
            if (candStep <= step && step < expStep) out.push([Math.floor(key / map.width), key % map.width])
        })
        return out
    }, [timeline, firstSeen, step, map.width])
    const edgesVisible = useMemo(
        () => showTree && timeline ? timeline.edges.filter((e) => e.step <= step) : [],
        [showTree, timeline, step],
    )
    // anytime planner는 개선된 경로를 여러 번 발표한다 — 현재 step 까지의 최신 경로를 그린다.
    const visiblePath = useMemo(() => {
        if (!timeline) return null
        let latest: Cell[] | null = null
        for (const p of timeline.paths) {
            if (p.step <= step) latest = p.path
        }
        return latest
    }, [timeline, step])

    // 실행형 planner(D* Lite 등): 로봇이 주행하며 장애물을 발견한다. 이때 실제 맵은
    // 로봇이 모르는 정보이므로 벽을 흐리게(ghost) 깔고, 발견된 벽만 진하게 그린다.
    const executed = timeline !== undefined && timeline.robot.length > 0
    const revealedVisible = useMemo(
        () => executed ? timeline!.revealed.filter((r) => r.step <= step) : [],
        [executed, timeline, step],
    )
    const robotTrail = useMemo(
        () => executed ? timeline!.robot.filter((r) => r.step <= step).map((r) => r.cell) : [],
        [executed, timeline, step],
    )

    // 벽 페인팅: pointer down 시 첫 셀의 반전값을 붓 값으로 삼아 드래그 내내 유지한다.
    const paintValue = useRef<boolean | null>(null)
    const cellAt = (stage: Konva.Stage | null): Cell | null => {
        const pos = stage?.getPointerPosition()
        if (!pos) return null
        const col = Math.floor(pos.x / cell)
        const row = Math.floor(pos.y / cell)
        if (row < 0 || row >= map.height || col < 0 || col >= map.width) return null
        return [row, col]
    }
    const paint = (c: Cell) => {
        if (!onPaintCell || paintValue.current === null) return
        if (start && c[0] === start[0] && c[1] === start[1]) return
        if (goal && c[0] === goal[0] && c[1] === goal[1]) return
        onPaintCell(c[0], c[1], paintValue.current)
    }

    // 연속 상태 planner(Hybrid A* 등): cell 필드가 [x, y] world 좌표다.
    const continuous = timeline?.continuous ?? false
    const center = (c: Cell): [number, number] => {
        if (continuous) {
            const [u, v] = worldToCellUnits(map, c[0], c[1])
            return [u * cell, v * cell]
        }
        return [(c[1] + 0.5) * cell, (c[0] + 0.5) * cell]
    }

    // 차량: heading이 있는 pose를 몸체 사각형 + 앞유리 선으로 그린다. world θ와
    // canvas 회전은 y축 반전 때문에 부호가 반대다.
    const car = (pose: [number, number, number], opts: {fill?: string; stroke: string; dash?: number[]}) => {
        const [x, y] = center([pose[0], pose[1]])
        const len = cell * 1.35
        const wid = cell * 0.78
        const sw = Math.max(1.2, cell * 0.08)
        return (
            <Group x={x} y={y} rotation={-pose[2] * 180 / Math.PI} listening={false}>
                <Rect x={-len / 2} y={-wid / 2} width={len} height={wid}
                      cornerRadius={wid * 0.28} fill={opts.fill}
                      stroke={opts.stroke} strokeWidth={sw} dash={opts.dash}/>
                <Line points={[len * 0.14, -wid * 0.3, len * 0.14, wid * 0.3]}
                      stroke={opts.stroke} strokeWidth={sw} dash={opts.dash} lineCap="round"/>
            </Group>
        )
    }

    const endpoint = (c: Cell, fill: string, onMove?: (cell: Cell) => void) => {
        const [x, y] = center(c)
        return <Circle
            x={x} y={y} radius={cell * 0.36} fill={fill}
            stroke={colors.bg} strokeWidth={Math.max(1, cell * 0.06)}
            draggable={!!onMove}
            onDragEnd={(e) => {
                const col = Math.floor(e.target.x() / cell)
                const row = Math.floor(e.target.y() / cell)
                // 스냅은 부모 상태 갱신이 담당한다 — 원래 중심으로 되돌려 이중 이동을 막는다.
                e.target.position({x, y})
                onMove?.([
                    Math.max(0, Math.min(map.height - 1, row)),
                    Math.max(0, Math.min(map.width - 1, col)),
                ])
            }}
        />
    }

    return (
        <Stage width={stageW} height={stageH}
               className="bg-surface border border-border rounded-lg overflow-hidden w-fit"
               onPointerDown={(e) => {
                   if (!onPaintCell) return
                   const c = cellAt(e.target.getStage())
                   if (!c) return
                   // 시작/골 위에서 드래그를 시작하면 페인팅이 아니라 endpoint 이동이다.
                   if (start && c[0] === start[0] && c[1] === start[1]) return
                   if (goal && c[0] === goal[0] && c[1] === goal[1]) return
                   paintValue.current = !map.occupied[c[0] * map.width + c[1]]
                   paint(c)
               }}
               onPointerMove={(e) => {
                   if (paintValue.current === null) return
                   const c = cellAt(e.target.getStage())
                   if (c) paint(c)
               }}
               onPointerUp={() => { paintValue.current = null }}
               onPointerLeave={() => { paintValue.current = null }}>
            <Layer>
                {/* 비교용 배경 셀 (다른 알고리즘의 확장 영역 등) */}
                {shadowCells?.map((c, i) => (
                    <Rect key={`sh${i}`} x={c[1] * cell} y={c[0] * cell}
                          width={cell} height={cell} fill={colors.muted} opacity={0.22}/>
                ))}
                {/* 탐색 완료(CLOSED) — 연속 모드에서는 pose 점, grid 모드에서는 셀 */}
                {expandedVisible.map((e, i) => continuous
                    ? <Circle key={`e${i}`} x={center(e.cell)[0]} y={center(e.cell)[1]}
                              radius={Math.max(1.5, cell * 0.1)} fill={colors.accent}
                              opacity={0.5}/>
                    : <Rect key={`e${i}`} x={e.cell[1] * cell} y={e.cell[0] * cell}
                            width={cell} height={cell} fill={colors.accent} opacity={0.24}/>)}
                {/* frontier(OPEN) 셀 — 발견됐지만 아직 확장 전 */}
                {frontierVisible.map((c, i) => (
                    <Rect key={`f${i}`} x={c[1] * cell} y={c[0] * cell}
                          width={cell} height={cell} fill={colors.accent} opacity={0.10}/>
                ))}
                {/* 벽 — 실행형 재생에서는 미발견 벽을 ghost로만 표시 */}
                {map.occupied.map((occ, i) => occ && (
                    <Rect key={`w${i}`} x={(i % map.width) * cell} y={Math.floor(i / map.width) * cell}
                          width={cell} height={cell} fill={colors.text}
                          opacity={executed ? 0.15 : 0.78}/>
                ))}
                {/* 로봇이 발견한 벽 */}
                {revealedVisible.map((r, i) => (
                    <Rect key={`rv${i}`} x={r.cell[1] * cell} y={r.cell[0] * cell}
                          width={cell} height={cell} fill={colors.text} opacity={0.78}/>
                ))}
                {/* grid 선 — 셀이 충분히 클 때만 (작으면 노이즈) */}
                {cell >= 9 && Array.from({length: map.width + 1}, (_, k) => (
                    <Line key={`gv${k}`} points={[k * cell, 0, k * cell, stageH]}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {cell >= 9 && Array.from({length: map.height + 1}, (_, k) => (
                    <Line key={`gh${k}`} points={[0, k * cell, stageW, k * cell]}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {/* 탐색 트리 (sampling 계열용) */}
                {edgesVisible.map((e, i) => (
                    <Line key={`t${i}`} points={[...center(e.from), ...center(e.to)]}
                          stroke={colors.accent} strokeWidth={1} opacity={0.5}/>
                ))}
                {/* 비교용 보조 경로 (점선) */}
                {overlayPath && overlayPath.length > 1 && (
                    <Line points={overlayPath.flatMap((c) => center(c))}
                          stroke={colors.muted} strokeWidth={Math.max(1.6, cell * 0.16)}
                          dash={[cell * 0.5, cell * 0.4]} lineCap="round" lineJoin="round"
                          opacity={0.85}/>
                )}
                {/* 발표된 최신 경로 */}
                {visiblePath && (
                    <Line points={visiblePath.flatMap((c) => center(c))}
                          stroke={PATH_COLOR} strokeWidth={Math.max(2.5, cell * 0.28)}
                          lineCap="round" lineJoin="round"/>
                )}
                {/* 로봇 주행 궤적 + 현재 위치 */}
                {robotTrail.length > 1 && (
                    <Line points={robotTrail.flatMap((c) => center(c))}
                          stroke={colors.accent2} strokeWidth={Math.max(2, cell * 0.2)}
                          lineCap="round" lineJoin="round" opacity={0.9}/>
                )}
                {robotTrail.length > 0 && (
                    <Circle x={center(robotTrail[robotTrail.length - 1])[0]}
                            y={center(robotTrail[robotTrail.length - 1])[1]}
                            radius={cell * 0.32} fill={colors.accent2}
                            stroke={colors.bg} strokeWidth={Math.max(1, cell * 0.06)}/>
                )}
                {/* 시작/목표 — 차량 pose가 주어지면 원 마커 대신 차로 그린다 */}
                {start && !carPose && endpoint(start, colors.accent, onMoveStart)}
                {goal && !goalPose && endpoint(goal, PATH_COLOR, onMoveGoal)}
                {goalPose && car(goalPose, {stroke: PATH_COLOR, dash: [cell * 0.3, cell * 0.24]})}
                {carPose && car(carPose, {fill: colors.accent2, stroke: colors.bg})}
            </Layer>
        </Stage>
    )
}

export default GridCanvas
