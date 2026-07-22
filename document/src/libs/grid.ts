// 웹 패널이 쓰는 occupancy grid 모델. 저장소 컨벤션과 동일하게 셀 인덱스는 (row, col)
// 이며 row 0이 이미지 상단이다 (world 좌표 변환은 여기서 다루지 않는다 — 패널은 셀만 그린다).
export interface GridMap {
    name: string;
    width: number;    // cols
    height: number;   // rows
    // row-major 점유 여부. index = row * width + col.
    occupied: boolean[];
    // world 좌표 변환 (연속 상태 planner 용). 기본: resolution 1, origin (0, 0).
    resolution: number;
    originX: number;
    originY: number;
}

export const cellIndex = (map: GridMap, row: number, col: number) => row * map.width + col;

export const inBounds = (map: GridMap, row: number, col: number) =>
    row >= 0 && row < map.height && col >= 0 && col < map.width;

export const isOccupied = (map: GridMap, row: number, col: number) =>
    !inBounds(map, row, col) || map.occupied[cellIndex(map, row, col)];

// tools/web_export가 만드는 맵 JSON: rows는 '#'(occupied)/'.'(free) 문자열 배열.
export interface GridMapJson {
    name: string;
    width: number;
    height: number;
    resolution?: number;
    origin?: [number, number];
    rows: string[];
}

export function parseGridMap(json: GridMapJson): GridMap {
    const occupied: boolean[] = new Array(json.width * json.height).fill(false)
    json.rows.forEach((row, r) => {
        for (let c = 0; c < json.width; c++) occupied[r * json.width + c] = row[c] === "#"
    })
    return {
        name: json.name, width: json.width, height: json.height, occupied,
        resolution: json.resolution ?? 1,
        originX: json.origin?.[0] ?? 0,
        originY: json.origin?.[1] ?? 0,
    }
}

// 사용자가 벽을 그리는 sandbox 용 빈 맵.
export function emptyGrid(name: string, width: number, height: number): GridMap {
    return {
        name, width, height, occupied: new Array(width * height).fill(false),
        resolution: 1, originX: 0, originY: 0,
    }
}

// PGM 밝기 행(공백 구분 토큰)에서 GridMap 을 만든다 — 255(흰색)만 자유, 나머지는 점유.
// local 데모 sandbox 들이 저장소 pgm 맵을 인라인 프리셋으로 쓸 때 공유한다.
export function gridFromPgmRows(name: string, rows: string[], resolution: number): GridMap {
    const width = rows[0].trim().split(/\s+/).length
    const height = rows.length
    const occupied: boolean[] = []
    for (const row of rows) {
        for (const tok of row.trim().split(/\s+/)) occupied.push(tok !== "255")
    }
    return {name, width, height, occupied, resolution, originX: 0, originY: 0}
}

// world (x, y) → grid 단위 좌표 (연속 상태 planner 렌더용). u는 col 방향(0..width),
// v는 row 방향(0..height, 위가 0). world y는 아래가 원점이라 뒤집는다.
export const worldToCellUnits = (map: GridMap, x: number, y: number): [number, number] => [
    (x - map.originX) / map.resolution,
    map.height - (y - map.originY) / map.resolution,
]

// grid cell (row, col) → 그 셀 중심의 world (x, y). 위 worldToCellUnits의 역방향
// (연속 상태 planner의 endpoint 드래그용) — world y는 위로 증가라 row를 뒤집는다.
export const cellCenterWorld = (map: GridMap, c: [number, number]): [number, number] => [
    map.originX + (c[1] + 0.5) * map.resolution,
    map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution,
]

// world (x, y) → 정수 셀 인덱스 (row, col) — 저장소 OccupancyGrid2D.world_to_cell 미러.
// worldToCellUnits(연속 렌더용, 반올림 없음)와 달리 충돌/거리 질의가 요구하는 정확한
// floor 셀 인덱스를 낸다. occupancy 질의(충돌 검사, EDT, 근방 열거)가 전부 공유한다.
export const worldToCellIndex = (map: GridMap, x: number, y: number): [number, number] => [
    map.height - 1 - Math.floor((y - map.originY) / map.resolution),
    Math.floor((x - map.originX) / map.resolution),
]
