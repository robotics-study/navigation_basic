#!/usr/bin/env python3
"""벤치마크 그룹 비교 차트 생성 → docs/assets/bench/*.png.

측정치는 tools/bench/run_matrix.py 결과(seed=1, 기본 파라미터). 투명 배경 + 중간톤 텍스트로
라이트/다크 페이지 양쪽에서 읽히게 렌더한다. 콘텐츠 수정 시 `python docs/make_bench_charts.py` 재실행.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent / "assets" / "bench"
OUT.mkdir(parents=True, exist_ok=True)

INK = "#8b95a5"      # 라이트/다크 공통 가독 중간톤
MAZE = "#6366f1"     # indigo
OPEN = "#06b6d4"     # cyan
C2 = "#a78bfa"       # violet (2nd series in runtime chart)

plt.rcParams.update({
    "font.size": 11, "axes.edgecolor": INK, "axes.labelcolor": INK,
    "xtick.color": INK, "ytick.color": INK, "text.color": INK,
    "axes.titlecolor": INK, "figure.facecolor": "none", "axes.facecolor": "none",
    "savefig.facecolor": "none", "axes.grid": True, "grid.color": INK,
    "grid.alpha": 0.18, "grid.linewidth": 0.7,
})


def grouped(fname, title, ylabel, cats, series, colors, note=None, logy=False):
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=150)
    n = len(series)
    w = 0.8 / n
    x = list(range(len(cats)))
    for i, (label, vals) in enumerate(series):
        offs = [xi + (i - (n - 1) / 2) * w for xi in x]
        bars = ax.bar(offs, vals, width=w, label=label, color=colors[i], zorder=3)
        for b, v in zip(bars, vals):
            ax.annotate(f"{v:g}", (b.get_x() + b.get_width() / 2, v),
                        ha="center", va="bottom", fontsize=8.5, color=INK,
                        xytext=(0, 2), textcoords="offset points")
    ax.set_xticks(x)
    ax.set_xticklabels(cats)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
    if logy:
        ax.set_yscale("log")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, fontsize=9, ncol=len(series))
    ax.margins(y=0.18)
    if note:
        ax.text(0.0, -0.22, note, transform=ax.transAxes, fontsize=8.5, color=INK, alpha=0.85)
    fig.tight_layout()
    fig.savefig(OUT / fname, transparent=True, bbox_inches="tight")
    plt.close(fig)
    print("wrote", (OUT / fname).relative_to(OUT.parent.parent))


# 1) Discrete search — nodes expanded (같은 최적 경로, 관건은 확장량)
grouped(
    "discrete_expanded.png",
    "Discrete search — nodes expanded  (lower is better)",
    "nodes expanded",
    ["BFS", "Dijkstra", "A*"],
    [("maze01", [221, 211, 108]), ("open01", [267, 267, 71])],
    [MAZE, OPEN],
    note="Path cost is identical (all optimal); A*'s heuristic expands far fewer nodes.",
)

# 2) Sampling-based — path cost (같은 8000-iter 예산, 관건은 경로 품질)
grouped(
    "sampling_cost.png",
    "Sampling-based — path cost  (lower is better)",
    "path cost",
    ["RRT", "RRT*", "Fast-RRT"],
    [("maze01", [18.41, 13.46, 13.47]), ("open01", [14.37, 12.05, 12.05])],
    [MAZE, OPEN],
    note="RRT returns the first feasible path; RRT*/Fast-RRT converge near-optimal.",
)

# 3) Sampling-based — runtime, C++ vs Python (품질 동일, 언어 격차)
grouped(
    "sampling_runtime.png",
    "Sampling-based — runtime, 8000 iters  (log scale, trace on)",
    "seconds (log)",
    ["RRT* maze01", "RRT* open01", "Fast-RRT maze01", "Fast-RRT open01"],
    [("Python", [9.15, 8.35, 18.16, 18.02]), ("C++", [1.09, 0.98, 1.92, 1.87])],
    [C2, MAZE],
    note="Same algorithm and budget; C++ runs ~8-10x faster than Python.",
    logy=True,
)

print("done")
