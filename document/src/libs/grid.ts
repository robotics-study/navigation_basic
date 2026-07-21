// 웹 패널이 쓰는 occupancy grid 모델. 저장소 컨벤션과 동일하게 셀 인덱스는 (row, col)
// 이며 row 0 이 이미지 상단이다 (world 좌표 변환은 여기서 다루지 않는다 — 패널은 셀만 그린다).
export interface GridMap {
    name: string;
    width: number;    // cols
    height: number;   // rows
    // row-major 점유 여부. index = row * width + col.
    occupied: boolean[];
}

export const cellIndex = (map: GridMap, row: number, col: number) => row * map.width + col;

export const inBounds = (map: GridMap, row: number, col: number) =>
    row >= 0 && row < map.height && col >= 0 && col < map.width;

export const isOccupied = (map: GridMap, row: number, col: number) =>
    !inBounds(map, row, col) || map.occupied[cellIndex(map, row, col)];

// tools/web_export 가 만드는 맵 JSON: rows 는 '#'(occupied)/'.'(free) 문자열 배열.
export interface GridMapJson {
    name: string;
    width: number;
    height: number;
    rows: string[];
}

export function parseGridMap(json: GridMapJson): GridMap {
    const occupied: boolean[] = new Array(json.width * json.height).fill(false)
    json.rows.forEach((row, r) => {
        for (let c = 0; c < json.width; c++) occupied[r * json.width + c] = row[c] === "#"
    })
    return {name: json.name, width: json.width, height: json.height, occupied}
}

// 사용자가 벽을 그리는 sandbox 용 빈 맵.
export function emptyGrid(name: string, width: number, height: number): GridMap {
    return {name, width, height, occupied: new Array(width * height).fill(false)}
}
