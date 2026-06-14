#!/usr/bin/env python3
"""Highlight bridge deck zones on big_ambitions_full_map_enhanced.svg."""
from __future__ import annotations

import csv
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
CSV = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
SVG = ROOT / "big_ambitions_full_map_enhanced.svg"
SVG_DOCS = Path(__file__).resolve().parents[1] / "docs" / "big_ambitions_enhanced_route_graph.svg"

# 4 voies logiques 2x2 : Gley = 8 Road mono-voie (2 sens x 2 voies)
CENTER_DECK = {"181", "182", "185", "186", "190", "191", "192", "193"}
WEST_CONNECTORS = {"180", "187", "188", "189"}  # zone grise sans couleur

WIDTH, HEIGHT, MARGIN = 1800, 1500, 70


def load_segments(csv_path: Path, roads: set[str]) -> tuple[str, tuple[float, float, float, float] | None]:
    points: dict[int, tuple[float, float, str]] = {}
    neighbors: dict[int, list[int]] = defaultdict(list)

    with csv_path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["edgeType"] != "base":
                continue
            a, b = int(row["fromIndex"]), int(row["toIndex"])
            for i, pre in ((a, "from"), (b, "to")):
                points[i] = (float(row[pre + "X"]), float(row[pre + "Z"]), row[pre + "Road"])
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
    cxs: list[float] = []
    czs: list[float] = []
    for a, ns in neighbors.items():
        ax, az, ar = points[a]
        if ar not in roads:
            continue
        cxs.append(ax)
        czs.append(az)
        for b in ns:
            bx, bz, br = points[b]
            if br not in roads:
                continue
            sx, sy = t(ax, az)
            ex, ey = t(bx, bz)
            segs.append(f"M{sx:.1f},{sy:.1f} L{ex:.1f},{ey:.1f}")

    if not segs:
        return "", None
    return " ".join(segs), (min(cxs), max(cxs), min(czs), max(czs))


def patch_svg(
    svg_path: Path,
    center_d: str,
    connector_d: str,
    center_bbox: tuple[float, float, float, float],
) -> None:
    text = svg_path.read_text(encoding="utf-8")
    text = re.sub(r'\n  <g id="bridge-center-preview">.*?</g>', "", text, flags=re.DOTALL)

    block = f"""
  <g id="bridge-center-preview">
    <!-- Jaune: tablier central R181-R193 -> fusion Road 1708 (4 couloirs 2x2) -->
    <path id="bridge-center-deck" data-label="Tablier central R181-R193 -> Road 1708"
      stroke="#ffd32a" fill="none" stroke-width="6" stroke-linecap="round" opacity="0.95"
      pointer-events="stroke" d="{center_d}"/>
    <!-- Orange: bretelles ouest R180/R187/R188/R189 -> a fusionner apres 1708 -->
    <path id="bridge-west-connectors" data-label="Bretelles ouest R180-R189 -> Road 1708?"
      stroke="#ff9f43" fill="none" stroke-width="5" stroke-linecap="round" opacity="0.9"
      stroke-dasharray="10 6" pointer-events="stroke" d="{connector_d}"/>
  </g>"""

    insert_before = '\n  <g id="bridge-merge-overlay">'
    if insert_before not in text:
        insert_before = '\n  <g id="synthetic-turns">'
    text = text.replace(insert_before, block + insert_before, 1)

    legend = f"""
    <rect class="legend" x="40" y="344" width="900" height="76" rx="10"/>
    <path stroke="#ffd32a" stroke-width="6" fill="none" d="M62 372 H95"/>
    <text class="small" x="110" y="368">Jaune: tablier central R181-R193 (4 voies 2x2) -&gt; fusion Road 1708</text>
    <path stroke="#ff9f43" stroke-width="5" fill="none" stroke-dasharray="10 6" d="M62 392 H95"/>
    <text class="small" x="110" y="396">Orange pointille: bretelles ouest R180-R189 (~300m) -&gt; fusionner ensuite?</text>
    <text class="small" x="110" y="414">Couloirs: L0 R182/R181 | L1 R185/R186 | L2 R190/R192 | L3 R191/R193 (ville-ouest / ouest-ville)</text>"""

    text = re.sub(
        r'\n    <rect class="legend" x="40" y="344".*?</text>\n',
        "",
        text,
        count=1,
        flags=re.DOTALL,
    )
    if "bretelles ouest R180-R189" not in text:
        text = text.replace(
            '    <text class="small" x="60" y="278">Virages verts',
            legend + '\n    <text class="small" x="60" y="278">Virages verts',
            1,
        )

    svg_path.write_text(text, encoding="utf-8")
    print(f"Patched {svg_path}")


def main() -> int:
    if not CSV.is_file():
        print(f"Missing CSV: {CSV}")
        return 1
    center_d, center_bbox = load_segments(CSV, CENTER_DECK)
    connector_d, _ = load_segments(CSV, WEST_CONNECTORS)
    if not center_bbox:
        print("No center deck segments found")
        return 1
    print(f"Center deck: {center_d.count(' M') + 1} segments")
    print(f"Connectors: {connector_d.count(' M') + 1} segments")
    print(f"BBox center: X[{center_bbox[0]:.0f},{center_bbox[1]:.0f}] Z[{center_bbox[2]:.0f},{center_bbox[3]:.0f}]")
    patch_svg(SVG, center_d, connector_d, center_bbox)
    if SVG_DOCS.is_file():
        patch_svg(SVG_DOCS, center_d, connector_d, center_bbox)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
