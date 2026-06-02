

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot ICCAD early-floorplanning result by reading BOTH files:
  1) input testcase CSV: block type / location / connection matrix
  2) output/result CFG: outline / block coordinates / channels / paths

Block colors follow the contest PDF convention:
  EDGE  = green
  MACRO = pink
  SOFT  = blue
  CHANNEL = light green

Usage examples:
  python plot_floorplan_read_csv_cfg.py "E_testcase_20260414.xlsx - case00.cfg" \
      "E_testcase_20260414.xlsx - case00.csv" -o case00_plot.png

  python plot_floorplan_read_csv_cfg.py "E_testcase_20260414.xlsx - case00.cfg" \
      --csv "E_testcase_20260414.xlsx - case00.csv" --label-edges --draw-connections

Notes for paths with spaces:
  Always wrap the cfg/csv path by double quotes on Windows/Linux.
"""

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D


# ----------------------------- data models -----------------------------

@dataclass
class RectObj:
    kind: str          # "block" or "channel"
    name: str
    x: float
    y: float
    w: float
    h: float

    def edge_center(self, edge: int) -> Tuple[float, float]:
        if edge == 1:   # left
            return (self.x, self.y + self.h / 2.0)
        if edge == 2:   # top
            return (self.x + self.w / 2.0, self.y + self.h)
        if edge == 3:   # right
            return (self.x + self.w, self.y + self.h / 2.0)
        if edge == 4:   # bottom
            return (self.x + self.w / 2.0, self.y)
        raise ValueError(f"Invalid edge number {edge} for {self.name}; must be 1, 2, 3, or 4.")

    def center(self) -> Tuple[float, float]:
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)


@dataclass
class PathPattern:
    net_count: int
    hops: List[Tuple[str, int]]   # [(rect_name, edge), ...]


@dataclass
class BlockInfo:
    name: str
    area: Optional[float] = None
    width: Optional[float] = None
    height: Optional[float] = None
    aspect: str = ""
    btype: str = "UNKNOWN"       # EDGE / MACRO / SOFT / UNKNOWN
    location: str = ""


# ----------------------------- cfg parser -----------------------------

def strip_comment(line: str) -> str:
    return line.split("#", 1)[0].strip()


def parse_cfg(cfg_path: Path):
    outline: Optional[Tuple[float, float]] = None
    blocks: List[RectObj] = []
    channels: List[RectObj] = []
    paths: List[PathPattern] = []

    section = None
    with cfg_path.open("r", encoding="utf-8-sig", errors="replace") as f:
        for raw in f:
            line = strip_comment(raw)
            if not line:
                continue

            parts = line.split()
            head = parts[0].upper()

            if head == "OUTLINE":
                if len(parts) < 3:
                    raise ValueError(f"Bad Outline line: {line}")
                outline = (float(parts[1]), float(parts[2]))
                continue

            if head == "BLOCK" and len(parts) == 1:
                section = "BLOCK"
                continue
            if head == "CHANNEL" and len(parts) == 1:
                section = "CHANNEL"
                continue
            if head == "PATH" and len(parts) == 1:
                section = "PATH"
                continue
            if head == "END":
                section = None
                continue

            if section == "BLOCK":
                if head != "BLOCK" or len(parts) < 6:
                    continue
                name = parts[1]
                x, y, w, h = map(float, parts[2:6])
                blocks.append(RectObj("block", name, x, y, w, h))
                continue

            if section == "CHANNEL":
                if head == "NUMCHANNELS":
                    continue
                if head != "CHANNEL" or len(parts) < 6:
                    continue
                name = parts[1]
                x, y, w, h = map(float, parts[2:6])
                channels.append(RectObj("channel", name, x, y, w, h))
                continue

            if section == "PATH":
                if head == "NUMROUTINGPATTERN":
                    continue
                if head != "PATH" or len(parts) < 4:
                    continue
                net_count = int(float(parts[1]))
                rest = parts[2:]
                if len(rest) % 2 != 0:
                    raise ValueError(f"Bad PATH line, rectangle/edge pairs are incomplete: {line}")
                hops = []
                for i in range(0, len(rest), 2):
                    hops.append((rest[i], int(rest[i + 1])))
                paths.append(PathPattern(net_count, hops))
                continue

    if outline is None:
        max_x = max((r.x + r.w for r in blocks + channels), default=0)
        max_y = max((r.y + r.h for r in blocks + channels), default=0)
        outline = (max_x, max_y)

    return outline, blocks, channels, paths


# ----------------------------- csv parser -----------------------------

def clean_cell(s) -> str:
    if s is None:
        return ""
    return str(s).replace("\ufeff", "").strip()


def parse_float_or_none(s: str) -> Optional[float]:
    s = clean_cell(s)
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def normalize_block_type(s: str) -> str:
    s = clean_cell(s).upper().replace("HARD", "").replace(" ", "")
    if "EDGE" in s:
        return "EDGE"
    if "MACRO" in s:
        return "MACRO"
    if "SOFT" in s:
        return "SOFT"
    return "UNKNOWN"


def read_csv_rows(csv_path: Path) -> List[List[str]]:
    with csv_path.open("r", encoding="utf-8-sig", errors="replace", newline="") as f:
        return [[clean_cell(c) for c in row] for row in csv.reader(f)]


def parse_testcase_csv(csv_path: Path) -> Tuple[Dict[str, BlockInfo], Dict[Tuple[str, str], int]]:
    """Return block metadata and connection matrix pairs from testcase CSV."""
    rows = read_csv_rows(csv_path)

    # Block table: in these testcase csv files, rows whose first cell is BLKxx are block rows.
    block_infos: Dict[str, BlockInfo] = {}
    for row in rows:
        if not row:
            continue
        name = clean_cell(row[0])
        if not re.fullmatch(r"BLK\d+", name, flags=re.IGNORECASE):
            continue

        # Ignore BLK rows inside the connection matrix by checking whether col 5 looks like a type.
        btype = normalize_block_type(row[5] if len(row) > 5 else "")
        if btype == "UNKNOWN":
            continue

        block_infos[name] = BlockInfo(
            name=name,
            area=parse_float_or_none(row[1] if len(row) > 1 else ""),
            width=parse_float_or_none(row[2] if len(row) > 2 else ""),
            height=parse_float_or_none(row[3] if len(row) > 3 else ""),
            aspect=clean_cell(row[4] if len(row) > 4 else ""),
            btype=btype,
            location=clean_cell(row[6] if len(row) > 6 else ""),
        )

    # Connection matrix: find header row containing block names after "ON CHIP..." marker.
    conn: Dict[Tuple[str, str], int] = {}
    for i, row in enumerate(rows):
        joined = " ".join(row).upper()
        if "ON CHIP INTERFACE CONNECTION" not in joined:
            continue

        # Next row should be: , BLK01, BLK02, ...
        header_row = None
        for j in range(i + 1, min(i + 6, len(rows))):
            candidates = [c for c in rows[j] if re.fullmatch(r"BLK\d+", c, flags=re.IGNORECASE)]
            if len(candidates) >= 2:
                header_row = rows[j]
                break
        if header_row is None:
            break

        col_to_blk = {}
        for col, cell in enumerate(header_row):
            if re.fullmatch(r"BLK\d+", cell, flags=re.IGNORECASE):
                col_to_blk[col] = cell

        # Matrix data rows follow until first non-BLK row.
        for k in range(j + 1, len(rows)):
            r = rows[k]
            if not r or not re.fullmatch(r"BLK\d+", clean_cell(r[0]), flags=re.IGNORECASE):
                break
            src = clean_cell(r[0])
            for col, dst in col_to_blk.items():
                if col >= len(r):
                    continue
                val = parse_float_or_none(r[col])
                if val and val > 0:
                    conn[(src, dst)] = int(round(val))
        break

    return block_infos, conn


# ----------------------------- drawing helpers -----------------------------

TYPE_STYLE = {
    # color choices match PDF meaning: edge green, macro pink, soft blue.
    "EDGE":    dict(face="#2ca02c", edge="#16631a", text="#07520d", label="EDGE block"),
    "MACRO":   dict(face="#f06ab4", edge="#a00062", text="#7a004b", label="MACRO block"),
    "SOFT":    dict(face="#9fd3ee", edge="#1f77b4", text="#1f4f88", label="SOFT block"),
    "UNKNOWN": dict(face="#dddddd", edge="#666666", text="#333333", label="UNKNOWN block"),
}

CONN_COLORS = ["#1f77b4", "#ff7f0e", "#9467bd", "#d62728", "#8c564b", "#17becf"]

# Distinct, high-contrast colors for individual path lines (cycled if more paths than colors).
PATH_COLORS = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
    "#42d4f4", "#f032e6", "#bfef45", "#fabed4", "#469990",
    "#dcbeff", "#9A6324", "#fffac8", "#800000", "#aaffc3",
    "#808000", "#ffd8b1", "#000075", "#a9a9a9", "#000000",
]


def edge_label_positions(r: RectObj):
    return {
        1: (r.x + max(r.w * 0.03, 2), r.y + r.h / 2, 90),
        2: (r.x + r.w / 2, r.y + r.h - max(r.h * 0.04, 2), 0),
        3: (r.x + r.w - max(r.w * 0.03, 2), r.y + r.h / 2, 90),
        4: (r.x + r.w / 2, r.y + max(r.h * 0.04, 2), 0),
    }


def draw_edge_labels(ax, r: RectObj, fontsize: int):
    for edge, (tx, ty, rot) in edge_label_positions(r).items():
        ax.text(tx, ty, f"E{edge}", ha="center", va="center", rotation=rot,
                fontsize=max(5, fontsize - 2), alpha=0.9, clip_on=True)


def line_points_for_path(path: PathPattern, rects: Dict[str, RectObj]) -> List[Tuple[float, float]]:
    pts = []
    for name, edge in path.hops:
        if name not in rects:
            raise KeyError(f"PATH references undefined rectangle/channel '{name}'")
        pts.append(rects[name].edge_center(edge))
    return pts


def maybe_text(ax, x, y, text, fontsize=8, **kwargs):
    if not text:
        return
    ax.text(x, y, text, fontsize=fontsize, **kwargs)


# ----------------------------- plotter -----------------------------

def plot_floorplan(
    cfg_path: Path,
    csv_path: Optional[Path],
    output_path: Path,
    show: bool,
    label_edges: bool,
    draw_paths: bool,
    draw_connections: bool,
    dpi: int,
    title: Optional[str],
    top_n_paths: int = 3,
) -> None:
    outline, blocks, channels, paths = parse_cfg(cfg_path)
    ow, oh = outline

    block_infos: Dict[str, BlockInfo] = {}
    connections: Dict[Tuple[str, str], int] = {}
    if csv_path:
        block_infos, connections = parse_testcase_csv(csv_path)
        print(f"Read CSV block types: {len(block_infos)} blocks")
        print(f"Read CSV connections: {len(connections)} nonzero directed pairs")

    fig_w = 12
    fig_h = max(7, fig_w * oh / ow) if ow > 0 else 8
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # Chip outline.
    ax.add_patch(Rectangle((0, 0), ow, oh, fill=False, linestyle="--", linewidth=1.8, edgecolor="black"))
    ax.text(0, oh, f"Outline {ow:g} x {oh:g}", ha="left", va="bottom", fontsize=10)

    # Channels behind blocks.
    for ch in channels:
        ax.add_patch(Rectangle(
            (ch.x, ch.y), ch.w, ch.h,
            facecolor="#c8efba", edgecolor="#4c8b35",
            linewidth=1.0, alpha=0.50,
        ))
        if ch.w >= 15 and ch.h >= 15:
            ax.text(ch.x + ch.w / 2, ch.y + ch.h / 2, ch.name,
                    ha="center", va="center", fontsize=7, color="#4c8b35", clip_on=True)

    # Blocks colored by type from CSV.
    for b in blocks:
        info = block_infos.get(b.name, BlockInfo(name=b.name))
        style = TYPE_STYLE.get(info.btype, TYPE_STYLE["UNKNOWN"])
        ax.add_patch(Rectangle(
            (b.x, b.y), b.w, b.h,
            facecolor=style["face"], edgecolor=style["edge"],
            linewidth=1.8, alpha=0.80,
        ))

        label_lines = [b.name]
        if info.btype != "UNKNOWN":
            label_lines.append(info.btype)
        if info.location:
            label_lines.append(info.location)
        ax.text(
            b.x + b.w / 2, b.y + b.h / 2,
            "\n".join(label_lines),
            ha="center", va="center",
            fontsize=8.5, fontweight="bold", color=style["text"], clip_on=True,
        )
        if label_edges:
            draw_edge_labels(ax, b, fontsize=8)

    # Optional flylines from CSV connection matrix. This is not legal routing; only visual aid.
    if draw_connections and connections:
        rects = {b.name: b for b in blocks}
        for idx, ((src, dst), nets) in enumerate(connections.items()):
            if src not in rects or dst not in rects:
                continue
            x1, y1 = rects[src].center()
            x2, y2 = rects[dst].center()
            color = CONN_COLORS[idx % len(CONN_COLORS)]
            ax.annotate(
                "", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="->", lw=3.0, mutation_scale=16, color=color, alpha=0.85),
            )
            ax.text((x1 + x2) / 2, (y1 + y2) / 2, str(nets), fontsize=7,
                    color=color, ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.70))

    # Optional real PATH patterns from CFG.
    if draw_paths and paths:
        rects = {r.name: r for r in blocks + channels}
        total_paths = len(paths)
        path_handles = []

        # Dynamic thresholds based on path count.
        LEGEND_INLINE_LIMIT = 30   # show per-path legend entries only when <= this many paths
        LABEL_TOP_RATIO     = 0.20  # label only paths whose net_count >= top-20% threshold
        counts = [p.net_count for p in paths]
        label_threshold = sorted(counts, reverse=True)[max(0, int(len(counts) * LABEL_TOP_RATIO) - 1)]

        # Weight-based alpha: heavier paths are more opaque.
        min_c, max_c = min(counts), max(counts)
        count_range = max(max_c - min_c, 1)

        def path_alpha(net_count):
            frac = (net_count - min_c) / count_range   # 0.0 (lightest) .. 1.0 (heaviest)
            return 0.25 + 0.75 * frac                  # range 0.25 .. 1.00

        for idx, p in enumerate(paths, start=1):
            color = PATH_COLORS[(idx - 1) % len(PATH_COLORS)]
            alpha = path_alpha(p.net_count)
            lw = 1.2 + 1.5 * (p.net_count - min_c) / count_range   # 1.2 .. 2.7
            ms = 3 + 3 * (p.net_count - min_c) / count_range        # 3 .. 6
            try:
                pts = line_points_for_path(p, rects)
            except (KeyError, ValueError) as e:
                print(f"[Warning] Skip bad path #{idx}: {e}")
                continue
            if len(pts) < 2:
                continue
            xs = [v[0] for v in pts]
            ys = [v[1] for v in pts]
            ax.plot(xs, ys, color=color, alpha=alpha, marker="o",
                    linewidth=lw, markersize=ms)
            # Only label heavy paths to avoid clutter.
            if p.net_count >= label_threshold:
                mid = len(pts) // 2
                ax.text(xs[mid], ys[mid], f"({p.net_count})", fontsize=6,
                        ha="left", va="bottom", color=color, fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.12", fc="white", ec=color,
                                  alpha=0.85, linewidth=0.7))
            if total_paths <= LEGEND_INLINE_LIMIT:
                path_handles.append(
                    Line2D([0], [0], color=color, alpha=alpha, linewidth=lw,
                           marker="o", markersize=ms,
                           label=f"PATH {idx}  [{p.net_count} nets]")
                )

    ax.set_aspect("equal", adjustable="box")
    margin_x = max(ow * 0.03, 10)
    margin_y = max(oh * 0.03, 10)
    ax.set_xlim(-margin_x, ow + margin_x)
    ax.set_ylim(-margin_y, oh + margin_y)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linewidth=0.4, alpha=0.35)
    ax.set_title(title if title else cfg_path.name)

    # Legend.
    handles = [
        Rectangle((0, 0), 1, 1, facecolor=TYPE_STYLE["EDGE"]["face"], edgecolor=TYPE_STYLE["EDGE"]["edge"], alpha=0.80, label="EDGE block"),
        Rectangle((0, 0), 1, 1, facecolor=TYPE_STYLE["MACRO"]["face"], edgecolor=TYPE_STYLE["MACRO"]["edge"], alpha=0.80, label="MACRO block"),
        Rectangle((0, 0), 1, 1, facecolor=TYPE_STYLE["SOFT"]["face"], edgecolor=TYPE_STYLE["SOFT"]["edge"], alpha=0.80, label="SOFT block"),
        Rectangle((0, 0), 1, 1, facecolor="#c8efba", edgecolor="#4c8b35", alpha=0.50, label="CHANNEL"),
        Line2D([0], [0], color="black", linestyle="--", label="OUTLINE"),
    ]
    if draw_connections:
        handles.append(Line2D([0], [0], color=CONN_COLORS[0], marker=">", linewidth=3.0, label="CSV connection flyline"))
    if draw_paths and paths:
        handles.append(Line2D([0], [0], color="none", label=f"── PATHs (total: {total_paths}) ──"))
        if total_paths <= LEGEND_INLINE_LIMIT:
            handles.extend(path_handles)
        else:
            # Summary mode: show net_count range instead of per-path entries.
            min_nets, max_nets = min(p.net_count for p in paths), max(p.net_count for p in paths)
            handles.append(Line2D([0], [0], color="gray", linewidth=1.5,
                                  label=f"nets range: {min_nets} – {max_nets}"))
            handles.append(Line2D([0], [0], color="gray", linewidth=2.7, alpha=1.0,
                                  label=f"darker/thicker = heavier"))
            handles.append(Line2D([0], [0], color="gray", linewidth=1.2, alpha=0.25,
                                  label=f"lighter/thinner = lighter"))
            handles.append(Line2D([0], [0], color="none",
                                  label=f"(labels shown: top {int(LABEL_TOP_RATIO*100)}%)"))
    ax.legend(handles=handles, loc="lower right", fontsize=8, framealpha=0.88)

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi)
    print(f"Saved: {output_path}")

    if show:
        plt.show()
    plt.close(fig)

    # ---- Second figure: top-N heaviest paths only ----
    if draw_paths and paths:
        top_n = top_n_paths
        sorted_paths = sorted(enumerate(paths, start=1), key=lambda t: t[1].net_count, reverse=True)
        top_paths = sorted_paths[:top_n]
        top_indices = {idx for idx, _ in top_paths}

        fig2, ax2 = plt.subplots(figsize=(fig_w, fig_h))
        ax2.add_patch(Rectangle((0, 0), ow, oh, fill=False, linestyle="--", linewidth=1.8, edgecolor="black"))
        ax2.text(0, oh, f"Outline {ow:g} x {oh:g}", ha="left", va="bottom", fontsize=10)

        for ch in channels:
            ax2.add_patch(Rectangle(
                (ch.x, ch.y), ch.w, ch.h,
                facecolor="#c8efba", edgecolor="#4c8b35", linewidth=1.0, alpha=0.30,
            ))

        for b in blocks:
            info = block_infos.get(b.name, BlockInfo(name=b.name))
            style = TYPE_STYLE.get(info.btype, TYPE_STYLE["UNKNOWN"])
            ax2.add_patch(Rectangle(
                (b.x, b.y), b.w, b.h,
                facecolor=style["face"], edgecolor=style["edge"],
                linewidth=1.8, alpha=0.55,
            ))
            ax2.text(
                b.x + b.w / 2, b.y + b.h / 2, b.name,
                ha="center", va="center",
                fontsize=8.5, fontweight="bold", color=style["text"], clip_on=True,
            )

        rects2 = {r.name: r for r in blocks + channels}
        top_handles = []
        for rank, (idx, p) in enumerate(top_paths, start=1):
            color = PATH_COLORS[(idx - 1) % len(PATH_COLORS)]
            try:
                pts = line_points_for_path(p, rects2)
            except (KeyError, ValueError) as e:
                print(f"[Warning] Skip top path #{idx}: {e}")
                continue
            if len(pts) < 2:
                continue
            xs = [v[0] for v in pts]
            ys = [v[1] for v in pts]
            ax2.plot(xs, ys, color=color, marker="o", linewidth=3.0, markersize=7, zorder=5)
            mid = len(pts) // 2
            ax2.text(xs[mid], ys[mid], f"#{rank} ({p.net_count})", fontsize=8,
                     ha="left", va="bottom", color=color, fontweight="bold",
                     bbox=dict(boxstyle="round,pad=0.18", fc="white", ec=color, alpha=0.90, linewidth=1.0))
            top_handles.append(
                Line2D([0], [0], color=color, linewidth=3.0, marker="o", markersize=7,
                       label=f"PATH {idx}  [{p.net_count} nets]  (#{rank})")
            )

        ax2.set_aspect("equal", adjustable="box")
        ax2.set_xlim(-margin_x, ow + margin_x)
        ax2.set_ylim(-margin_y, oh + margin_y)
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        ax2.grid(True, linewidth=0.4, alpha=0.35)
        base_title = title if title else cfg_path.name
        ax2.set_title(f"{base_title}  —  Top {top_n} Heaviest PATHs")

        top_legend_handles = [
            Rectangle((0, 0), 1, 1, facecolor=TYPE_STYLE["EDGE"]["face"], edgecolor=TYPE_STYLE["EDGE"]["edge"], alpha=0.55, label="EDGE block"),
            Rectangle((0, 0), 1, 1, facecolor=TYPE_STYLE["MACRO"]["face"], edgecolor=TYPE_STYLE["MACRO"]["edge"], alpha=0.55, label="MACRO block"),
            Rectangle((0, 0), 1, 1, facecolor=TYPE_STYLE["SOFT"]["face"], edgecolor=TYPE_STYLE["SOFT"]["edge"], alpha=0.55, label="SOFT block"),
            Line2D([0], [0], color="none", label=f"── Top {top_n} PATHs (of {total_paths}) ──"),
        ] + top_handles
        ax2.legend(handles=top_legend_handles, loc="lower right", fontsize=8, framealpha=0.88)

        top_output = output_path.with_name(output_path.stem + f"_top{top_n}paths" + output_path.suffix)
        fig2.tight_layout()
        fig2.savefig(top_output, dpi=dpi)
        print(f"Saved: {top_output}")

        if show:
            plt.show()
        plt.close(fig2)


# ----------------------------- main -----------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Plot ICCAD floorplan .cfg and color block types by reading the testcase .csv."
    )
    parser.add_argument("cfg", type=Path, help="input/result .cfg file")
    parser.add_argument("csv", type=Path, nargs="?", default=None,
                        help="input testcase .csv file, optional positional")
    parser.add_argument("--csv", dest="csv_opt", type=Path, default=None,
                        help="input testcase .csv file, optional named argument")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="output image path, default: <cfg_stem>_plot.png")
    parser.add_argument("--show", action="store_true", help="show an interactive matplotlib window")
    parser.add_argument("--label-edges", action="store_true", help="draw E1/E2/E3/E4 labels on each block")
    parser.add_argument("--draw-paths", action="store_true", help="draw PATH routing patterns if cfg contains PATH section")
    parser.add_argument("--draw-connections", action="store_true",
                        help="draw center-to-center flylines from CSV connection matrix; visual aid only")
    parser.add_argument("--dpi", type=int, default=180, help="output image DPI")
    parser.add_argument("--title", default=None, help="custom plot title")
    parser.add_argument("--top-n", type=int, default=3,
                        help="number of heaviest paths to show in the second figure (default: 3)")
    args = parser.parse_args()

    if not args.cfg.exists():
        raise FileNotFoundError(args.cfg)

    csv_path = args.csv_opt if args.csv_opt is not None else args.csv
    if csv_path is not None and not csv_path.exists():
        raise FileNotFoundError(csv_path)

    output = args.output or args.cfg.with_name(args.cfg.stem + "_plot.png")

    plot_floorplan(
        cfg_path=args.cfg,
        csv_path=csv_path,
        output_path=output,
        show=args.show,
        label_edges=args.label_edges,
        draw_paths=args.draw_paths,
        draw_connections=args.draw_connections,
        dpi=args.dpi,
        title=args.title,
        top_n_paths=args.top_n,
    )


if __name__ == "__main__":
    main()