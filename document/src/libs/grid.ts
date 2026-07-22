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
