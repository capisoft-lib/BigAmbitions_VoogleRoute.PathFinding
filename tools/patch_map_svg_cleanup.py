#!/usr/bin/env python3
"""
Retire les surcouches couleur du pont sur la carte SVG.
Conserve virages verts + uturn orange. Met en valeur la diagonale Road 1703.
"""
from __future__ import annotations

import csv
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
CSV = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
SVG_PATHS = [
    ROOT / "big_ambitions_full_map_enhanced.svg",
    Path(__file__).resolve().parents[1] / "docs" / "big_ambitions_enhanced_route_graph.svg",
]

HIGHLIGHT_ROAD = "1703"
HIGHLIGHT_COLOR = "#e056fd"
WIDTH, HEIGHT, MARGIN = 1800, 1500, 70


def load_road_segments(csv_path: Path, road: str) -> str:
    points: dict[int, tuple[float, float, str]] = {}
    neighbors: dict[int, list[int]] = defaultdict(list)

    with csv_path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["edgeType"] != "base":
                continue
            a, b = int(row["fromIndex"]), int(row["toIndex"])
            for i, pre in ((a, "from"), (b, "to")):
                points[i] = (float(row[pre + "X"]), float(row[pre + "Z"]), row[pre + "Road"])
            if row["fromRoad"] == road or row["toRoad"] == road:
                neighbors[a].append(b)

    xs = [p[0] for p in points.values()]
    zs = [p[1] for p in points.values()]
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    scale = min((WIDTH - MARGIN * 2) / (max_x - min_x), (HEIGHT - MARGIN * 2) / (max_z - min_z))
    off_x = (WIDTH - (max_x - min_x) * scale) / 2
    off_y = (HEIGHT - (max_z - min_z) * scale) / 2

    def t(x: float, z: float) -> tuple[float, float]:
        return off_x + (x - min_x) * scale, off_y + (max_z - z) * scale

    segs: list[str] = []
    for a, ns in neighbors.items():
        if a not in points:
            continue
        ax, az, ar = points[a]
        for b in ns:
            if b not in points:
                continue
            bx, bz, br = points[b]
            if ar != road and br != road:
                continue
            sx, sy = t(ax, az)
            ex, ey = t(bx, bz)
            segs.append(f"M{sx:.1f},{sy:.1f} L{ex:.1f},{ey:.1f}")
    return " ".join(segs)


def patch_svg(svg_path: Path, highlight_d: str) -> None:
    text = svg_path.read_text(encoding="utf-8")

    # Retirer surcouches pont
    text = re.sub(r'\n  <g id="bridge-center-preview">.*?</g>', "", text, flags=re.DOTALL)
    text = re.sub(r'\n  <g id="bridge-merge-overlay">.*?</g>', "", text, flags=re.DOTALL)
    text = re.sub(r'\n  <g id="road-1703-highlight">.*?</g>', "", text, flags=re.DOTALL)

    # Styles bridge inutiles
    text = re.sub(r"\n      \.bridge-merge[^\n]*\n", "\n", text)

    # Légendes fusion pont (y=200 et y=344) + note overlay virages
    text = re.sub(
        r"\n\n\n\n    <rect class=\"legend\" x=\"40\" y=\"200\".*?"
        r"(?=\n  <g>\n    <rect class=\"legend\" x=\"40\" y=\"40\")",
        "\n",
        text,
        flags=re.DOTALL,
    )

    highlight_block = f"""
  <g id="road-1703-highlight">
    <!-- Diagonale R1703 (traversée R176/R195) — goulot A* restant -->
    <path id="road-highlight-1703" data-road="1703"
      data-label="Traversée diagonale Road 1703 (R176/R195) — ~9.7k nœuds A*"
      stroke="{HIGHLIGHT_COLOR}" fill="none" stroke-width="6" stroke-linecap="round"
      opacity="0.95" pointer-events="stroke" d="{highlight_d}"/>
  </g>"""

    insert_before = '\n  <g id="synthetic-turns">'
    if insert_before not in text:
        raise RuntimeError(f"Point d'insertion introuvable dans {svg_path}")
    text = text.replace(insert_before, highlight_block + insert_before, 1)

    # Légende 1703 dans le bloc principal (y=40)
    note_1703 = (
        f'    <path stroke="{HIGHLIGHT_COLOR}" stroke-width="6" fill="none" d="M62 178 H125"/>'
        f'<text class="small" x="140" y="182">magenta = diagonale Road 1703 (traversée R176/R195, goulot A*)</text>\n'
    )
    if "diagonale Road 1703" not in text:
        text = text.replace(
            '    <path class="left-turn" d="M320 150 Q345 126 370 150"/>',
            note_1703 + '    <path class="left-turn" d="M320 150 Q345 126 370 150"/>',
            1,
        )
        text = text.replace(
            '<rect class="legend" x="40" y="40" width="760" height="145"',
            '<rect class="legend" x="40" y="40" width="760" height="168"',
            1,
        )

    svg_path.write_text(text, encoding="utf-8")
    print(f"Patched {svg_path}")


def main() -> int:
    if not CSV.is_file():
        print(f"Missing CSV: {CSV}")
        return 1
    highlight_d = load_road_segments(CSV, HIGHLIGHT_ROAD)
    if not highlight_d:
        print(f"No segments for road {HIGHLIGHT_ROAD}")
        return 1
    print(f"Road {HIGHLIGHT_ROAD}: {highlight_d.count(' M') + 1} segments")
    for svg in SVG_PATHS:
        if svg.is_file():
            patch_svg(svg, highlight_d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
