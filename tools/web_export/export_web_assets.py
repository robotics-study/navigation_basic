#!/usr/bin/env python3
"""Export web-site data assets: grid maps as JSON + demo traces as gzip JSONL.

The docs SPA (document/) replays real demo traces. Benchmark-budget traces can be
huge (GB for sampling planners), so this tool generates traces with the demo's
default (small) budgets and gzips them for static serving.

Usage:
    python tools/web_export/export_web_assets.py --algos astar --maps maze01,open01
"""

from __future__ import annotations

import argparse
import gzip
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
DATA_DIR = REPO / "document" / "public" / "data"
# 웹 재생 상한 — 이보다 큰 trace 는 커밋하지 않는다 (재생 불가 크기 방지 가드).
MAX_EVENTS = 200_000


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    """Minimal PGM reader (P2/P5)."""
    data = path.read_bytes()
    if data[:2] == b"P2":
        tokens = []
        for line in data.decode("ascii").splitlines():
            line = line.split("#", 1)[0]
            tokens.extend(line.split())
        width, height, _maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
        pixels = [int(t) for t in tokens[4:4 + width * height]]
        return width, height, pixels
    if data[:2] == b"P5":
        # 헤더 3개 정수(width height maxval)를 공백 단위로 읽고 나머지가 래스터다.
        idx = 2
        values: list[int] = []
        while len(values) < 3:
            while idx < len(data) and data[idx:idx + 1].isspace():
                idx += 1
            if data[idx:idx + 1] == b"#":
                while data[idx:idx + 1] != b"\n":
                    idx += 1
                continue
            start = idx
            while idx < len(data) and not data[idx:idx + 1].isspace():
                idx += 1
            values.append(int(data[start:idx]))
        idx += 1  # 헤더 종료 공백 1바이트
        width, height, _maxval = values
        return width, height, list(data[idx:idx + width * height])
    raise ValueError(f"unsupported PGM format: {path}")


def export_map(name: str) -> None:
    map_yaml = REPO / "maps" / "grid" / f"{name}.yaml"
    meta = yaml.safe_load(map_yaml.read_text())
    image = map_yaml.parent / meta["image"]
    width, height, pixels = read_pgm(image)
    # ROS 스타일: 0(black)=occupied, 255(white)=free. 중간값은 임계 128 로 나눈다.
    rows = [
        "".join("#" if pixels[r * width + c] < 128 else "." for c in range(width))
        for r in range(height)
    ]
    out = DATA_DIR / "maps" / f"{name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    out.write_text(
        '{\n'
        f'  "name": "{name}",\n'
        f'  "width": {width},\n'
        f'  "height": {height},\n'
        f'  "resolution": {meta["resolution"]},\n'
        f'  "origin": [{origin[0]}, {origin[1]}],\n'
        '  "rows": [\n    '
        + ",\n    ".join(f'"{row}"' for row in rows)
        + "\n  ]\n}\n"
    )
    print(f"map: {out.relative_to(REPO)} ({width}x{height})")


def run_demo(algo: str, map_name: str, impl: str, trace_path: Path) -> bool:
    common = [
        "--map", str(REPO / "maps" / "grid" / f"{map_name}.yaml"),
        "--scenario", str(REPO / "maps" / "scenarios" / f"{map_name}_s1.yaml"),
        "--params", str(REPO / "configs" / "global_planning" / f"{algo}.yaml"),
        "--trace", str(trace_path),
    ]
    if impl == "py":
        cmd = [sys.executable, str(REPO / "python" / "demos" / f"demo_{algo}.py"), *common]
        env = {"PYTHONPATH": str(REPO / "python")}
    else:
        binary = REPO / "cpp" / "build" / "demos" / f"demo_{algo}"
        if not binary.exists():
            print(f"skip: {binary.relative_to(REPO)} not built", file=sys.stderr)
            return False
        cmd = [str(binary), *common]
        env = {}
    subprocess.run(cmd, check=True, cwd=REPO, env={**os.environ, **env},
                   stdout=subprocess.DEVNULL)
    return True


def export_traces(algo: str, map_name: str) -> None:
    # C++/Python 데모는 동일 이벤트 열을 방출하므로 웹 자산은 py 한 벌만 만든다.
    for impl in ("py",):
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.jsonl"
            if not run_demo(algo, map_name, impl, trace):
                continue
            events = sum(1 for _ in trace.open())
            if events > MAX_EVENTS:
                raise SystemExit(
                    f"{algo}/{map_name}/{impl}: {events} events > {MAX_EVENTS} — "
                    "reduce the demo budget before exporting for the web"
                )
            out = DATA_DIR / "traces" / algo / f"{map_name}.{impl}.jsonl.gz"
            out.parent.mkdir(parents=True, exist_ok=True)
            # mtime=0 으로 고정해 같은 입력이면 같은 바이트가 나오게 한다 (git diff 안정).
            with trace.open("rb") as src, gzip.GzipFile(out, "wb", mtime=0) as dst:
                shutil.copyfileobj(src, dst)
            print(f"trace: {out.relative_to(REPO)} ({events} events)")


def main() -> None:
    parser = argparse.ArgumentParser(description="export web data assets for document/")
    parser.add_argument("--algos", default="", help="comma-separated algorithm slugs (empty: maps only)")
    parser.add_argument("--maps", required=True, help="comma-separated grid map names")
    args = parser.parse_args()
    algos = [a for a in args.algos.split(",") if a]
    for map_name in args.maps.split(","):
        export_map(map_name)
        for algo in algos:
            export_traces(algo, map_name)


if __name__ == "__main__":
    main()
