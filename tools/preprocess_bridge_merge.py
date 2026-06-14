#!/usr/bin/env python3
"""
Fusionne les parties du pont (plusieurs Road Gley) en entités logiques unifiées.

Sauvegarde automatique du CSV actuel dans data/backups/ avant toute écriture.
Restaurer : copier le .bak sur big_ambitions_enhanced_routes.csv

Usage:
  python preprocess_bridge_merge.py              # backup + merge CSV + overlay SVG
  python preprocess_bridge_merge.py --backup-only
  python preprocess_bridge_merge.py --restore backups/big_ambitions_enhanced_routes_pre_bridge_merge_YYYY-MM-DD.csv
"""
from __future__ import annotations

import argparse
import csv
import re
import shutil
import sys
from collections import defaultdict
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]  # repo BigAmbitions/
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
CSV_PATH = DATA_DIR / "big_ambitions_enhanced_routes.csv"
BACKUP_DIR = DATA_DIR / "backups"
SVG_PATH = ROOT / "big_ambitions_full_map_enhanced.svg"
SVG_DOCS_PATH = Path(__file__).resolve().parents[1] / "docs" / "big_ambitions_enhanced_route_graph.svg"

WIDTH = 1800
HEIGHT = 1500
MARGIN = 70

# Virtual merged road ids (>= 1700, hors plage Gley standard)
BRIDGE_PARTS: dict[str, dict] = {
    "bridge_deck": {
        "merged_road": "1700",
        "label": "Tablier R170 (2 voies)",
        "color": "#e056fd",
        "roads": {"170": lambda lane: lane},
    },
    "bridge_ramp_north": {
        "merged_road": "1701",
        "label": "Rampe nord R173",
        "color": "#f0932b",
        "roads": {"173": lambda lane: 0},
    },
    "bridge_ramp_south": {
        "merged_road": "1702",
        "label": "Rampe sud R175",
        "color": "#ff7979",
        "roads": {"175": lambda lane: 0},
    },
    "bridge_cross_176_195": {
        "merged_road": "1703",
        "label": "Traversée R176/R195",
        "color": "#686de0",
        "roads": {"176": lambda lane: 0, "195": lambda lane: 1},
    },
    "bridge_cross_177_194": {
        "merged_road": "1704",
        "label": "Traversée R177/R194",
        "color": "#4834d4",
        "roads": {"177": lambda lane: 0, "194": lambda lane: 1},
    },
    "bridge_cross_178_184": {
        "merged_road": "1705",
        "label": "Traversée R178/R184",
        "color": "#22a6b3",
        "roads": {"178": lambda lane: 0, "184": lambda lane: 1},
    },
    "bridge_cross_179_183": {
        "merged_road": "1706",
        "label": "Traversée R179/R183",
        "color": "#6ab04c",
        "roads": {"179": lambda lane: 0, "183": lambda lane: 1},
    },
    "bridge_junction_north": {
        "merged_road": "1707",
        "label": "Jonction nord R168",
        "color": "#7ed6df",
        "roads": {"168": lambda lane: lane},
    },
}

ROAD_TO_PART: dict[str, tuple[str, dict]] = {}
MERGED_ROAD_TO_PART: dict[str, str] = {}
for part_id, spec in BRIDGE_PARTS.items():
    MERGED_ROAD_TO_PART[spec["merged_road"]] = part_id
    for road in spec["roads"]:
        ROAD_TO_PART[road] = (part_id, spec)

# Zone industrielle : ne pas dessiner l'overlay tablier / jonction (side effects visuels)
INDUSTRIAL_OVERLAY_EXCLUDE = {
    "xmin": -1775.0,
    "xmax": -1645.0,
    "zmin": -1325.0,
    "zmax": -1135.0,
}
# Tablier R170 et jonction R168 : fusion CSV mais pas d'overlay (zone industrielle / side effects)
OVERLAY_SKIP_PARTS = {"bridge_junction_north", "bridge_deck"}
OVERLAY_INDUSTRIAL_FILTER_PARTS: set[str] = set()


def in_industrial_overlay_exclude(x: float, z: float) -> bool:
    b = INDUSTRIAL_OVERLAY_EXCLUDE
    return b["xmin"] <= x <= b["xmax"] and b["zmin"] <= z <= b["zmax"]


def should_draw_overlay_segment(part_id: str, ax: float, az: float, bx: float, bz: float) -> bool:
    if part_id in OVERLAY_SKIP_PARTS:
        return False
    if part_id in OVERLAY_INDUSTRIAL_FILTER_PARTS:
        if in_industrial_overlay_exclude(ax, az) or in_industrial_overlay_exclude(bx, bz):
            return False
    return True


def backup_csv(csv_path: Path) -> Path:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = date.today().isoformat()
    backup = BACKUP_DIR / f"big_ambitions_enhanced_routes_pre_bridge_merge_{stamp}.csv"
    if backup.exists():
        # même jour : suffixe horaire pour ne jamais écraser
        from datetime import datetime

        backup = BACKUP_DIR / (
            f"big_ambitions_enhanced_routes_pre_bridge_merge_{stamp}_"
            f"{datetime.now().strftime('%H%M%S')}.csv"
        )
    shutil.copy2(csv_path, backup)
    return backup


def remap_road_lane(road: str, lane: str) -> tuple[str, str, str | None]:
    if road not in ROAD_TO_PART:
        return road, lane, None
    part_id, spec = ROAD_TO_PART[road]
    lane_i = int(lane) if lane not in ("", None) else 0
    new_lane = spec["roads"][road](lane_i)
    return spec["merged_road"], str(new_lane), part_id


def remap_name(name: str, old_road: str, new_road: str, new_lane: str) -> str:
    if not name:
        return name
    return re.sub(
        rf"Road_{re.escape(old_road)}-Lane_\d+",
        f"Road_{new_road}-Lane_{new_lane}",
        name,
        count=1,
    )


def merge_row(row: dict[str, str]) -> dict[str, str]:
    out = dict(row)
    part_hint: str | None = None

    for prefix in ("from", "to"):
        road = row[f"{prefix}Road"]
        lane = row[f"{prefix}Lane"]
        new_road, new_lane, part_id = remap_road_lane(road, lane)
        if part_id:
            part_hint = part_hint or part_id
        out[f"{prefix}Road"] = new_road
        out[f"{prefix}Lane"] = new_lane
        out[f"{prefix}Name"] = remap_name(row[f"{prefix}Name"], road, new_road, new_lane)

    out["bridgePart"] = part_hint or ""
    return out


def load_rows(csv_path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    if "bridgePart" not in fieldnames:
        fieldnames.append("bridgePart")
    return fieldnames, rows


def write_merged_csv(csv_path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> int:
    merged = [merge_row(r) for r in rows]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(merged)
    return sum(1 for r in merged if r.get("bridgePart"))


def parse_base_graph(csv_path: Path):
    points: dict[int, dict] = {}
    neighbors: dict[int, list[int]] = defaultdict(list)
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("edgeType") != "base":
                continue
            a = int(row["fromIndex"])
            b = int(row["toIndex"])
            points[a] = {
                "x": float(row["fromX"]),
                "z": float(row["fromZ"]),
                "road": row["fromRoad"],
                "part": MERGED_ROAD_TO_PART.get(row["fromRoad"], ""),
            }
            points[b] = {
                "x": float(row["toX"]),
                "z": float(row["toZ"]),
                "road": row["toRoad"],
                "part": MERGED_ROAD_TO_PART.get(row["toRoad"], ""),
            }
            neighbors[a].append(b)
    return points, neighbors


def transform_factory(points: dict[int, dict]):
    xs = [p["x"] for p in points.values()]
    zs = [p["z"] for p in points.values()]
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    scale = min(
        (WIDTH - MARGIN * 2) / (max_x - min_x),
        (HEIGHT - MARGIN * 2) / (max_z - min_z),
    )
    off_x = (WIDTH - (max_x - min_x) * scale) / 2
    off_y = (HEIGHT - (max_z - min_z) * scale) / 2

    def t(x: float, z: float) -> tuple[float, float]:
        return off_x + (x - min_x) * scale, off_y + (max_z - z) * scale

    return t


def build_overlay_svg(csv_path: Path) -> str:
    points, neighbors = parse_base_graph(csv_path)
    transform = transform_factory(points)
    by_part: dict[str, list[str]] = defaultdict(list)

    for idx, ns in neighbors.items():
        if idx not in points:
            continue
        a = points[idx]
        for n in ns:
            if n not in points:
                continue
            b = points[n]
            part = a.get("part") or b.get("part")
            if not part:
                continue
            if not should_draw_overlay_segment(part, a["x"], a["z"], b["x"], b["z"]):
                continue
            ax, ay = transform(a["x"], a["z"])
            bx, by = transform(b["x"], b["z"])
            by_part[part].append(f"M{ax:.1f},{ay:.1f} L{bx:.1f},{by:.1f}")

    lines = [
        '  <g id="bridge-merge-overlay">',
        "    <!-- Fusion pont : vérification visuelle (ne pas regénérer tout le SVG) -->",
    ]
    for part_id, spec in BRIDGE_PARTS.items():
        if part_id in OVERLAY_SKIP_PARTS:
            continue
        segs = by_part.get(part_id)
        if not segs:
            continue
        color = spec["color"]
        d = " ".join(segs)
        lines.append(
            f'    <path class="bridge-merge" id="bridge-part-{part_id}" '
            f'data-part="{part_id}" data-label="{spec["label"]}" '
            f'stroke="{color}" fill="none" stroke-width="5" stroke-linecap="round" '
            f'opacity="0.92" pointer-events="stroke" d="{d}"/>'
        )
    lines.append("  </g>")
    return "\n".join(lines)


def patch_svg(svg_path: Path, overlay_block: str) -> None:
    text = svg_path.read_text(encoding="utf-8")

    # styles overlay
    style_extra = """
      .bridge-merge { stroke-linecap: round; stroke-linejoin: round; }
      .bridge-merge:hover { stroke-width: 7; opacity: 1; }"""
    if ".bridge-merge" not in text:
        text = text.replace("    </style>", f"{style_extra}\n    </style>", 1)

    # remplacer overlay existant
    import re as re_mod

    text = re_mod.sub(
        r'\n  <g id="bridge-merge-overlay">.*?</g>',
        "",
        text,
        flags=re_mod.DOTALL,
    )

    # légende fusion
    legend_extra = """
    <rect class="legend" x="40" y="200" width="760" height="132" rx="10"/>
    <text class="label" x="60" y="234">Fusion pont (préprocess CSV)</text>
    <text class="small" x="60" y="258">Traits épais = parties fusionnées | tablier/jonction masqués en zone industrielle</text>
    <text class="small" x="60" y="278">Virages verts (turn-4252-2268, turn-3921-439) au-dessus de l'overlay</text>"""
    legend_swatches = []
    y = 302
    for part_id, spec in BRIDGE_PARTS.items():
        if part_id in OVERLAY_SKIP_PARTS:
            continue
        legend_swatches.append(
            f'    <path stroke="{spec["color"]}" stroke-width="5" fill="none" d="M62 {y} H95"/>'
            f'<text class="small" x="110" y="{y + 4}">{spec["label"]} → Road {spec["merged_road"]}</text>'
        )
        y += 18
    legend_block = legend_extra + "\n" + "\n".join(legend_swatches)

    if "Fusion pont (préprocess CSV)" not in text:
        text = text.replace("  <g>\n    <rect class=\"legend\" x=\"40\" y=\"40\"", legend_block + "\n  <g>\n    <rect class=\"legend\" x=\"40\" y=\"40\"", 1)
    else:
        text = re_mod.sub(
            r'\n    <rect class="legend" x="40" y="200".*?(?=\n  <g>\n    <rect class="legend" x="40" y="40")',
            "",
            text,
            flags=re_mod.DOTALL,
        )
        text = text.replace(
            '  <g>\n    <rect class="legend" x="40" y="40"',
            legend_block + "\n  <g>\n    <rect class=\"legend\" x=\"40\" y=\"40\"",
            1,
        )

    insert_before = '\n  <g id="synthetic-turns">'
    if insert_before not in text:
        raise RuntimeError('Point d\'insertion SVG introuvable (<g id="synthetic-turns">).')
    text = text.replace(insert_before, "\n" + overlay_block + insert_before, 1)
    svg_path.write_text(text, encoding="utf-8")


def restore_csv(backup_path: Path, csv_path: Path) -> None:
    if not backup_path.is_file():
        raise FileNotFoundError(backup_path)
    shutil.copy2(backup_path, csv_path)
    print(f"Restored {csv_path} <= {backup_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion des parties du pont dans le CSV routes.")
    parser.add_argument("--backup-only", action="store_true", help="Créer uniquement la sauvegarde.")
    parser.add_argument("--restore", metavar="BACKUP_CSV", help="Restaurer un backup sur le CSV actif.")
    parser.add_argument("--no-svg", action="store_true", help="Ne pas patcher le SVG.")
    parser.add_argument(
        "--overlay-only",
        action="store_true",
        help="Régénère uniquement l'overlay SVG (CSV inchangé).",
    )
    args = parser.parse_args()

    if args.restore:
        restore_csv(Path(args.restore), CSV_PATH)
        return 0

    if not CSV_PATH.is_file():
        print(f"CSV introuvable: {CSV_PATH}", file=sys.stderr)
        return 1

    if args.overlay_only:
        overlay = build_overlay_svg(CSV_PATH)
        patch_svg(SVG_PATH, overlay)
        print(f"SVG overlay (only): {SVG_PATH}")
        if SVG_DOCS_PATH.parent.is_dir():
            shutil.copy2(SVG_PATH, SVG_DOCS_PATH)
        return 0

    with CSV_PATH.open(encoding="utf-8") as probe:
        head = probe.read(4096)
    if "Road_1700-" in head or ",1700," in head:
        print(
            "Le CSV semble déjà fusionné (Road 1700+). "
            "Restaurez un backup avant de relancer.",
            file=sys.stderr,
        )
        return 2

    backup = backup_csv(CSV_PATH)
    print(f"Backup: {backup}")

    if args.backup_only:
        return 0

    fieldnames, rows = load_rows(CSV_PATH)
    touched = write_merged_csv(CSV_PATH, fieldnames, rows)
    print(f"CSV fusionné: {CSV_PATH} ({touched} arêtes bridgePart)")

    if not args.no_svg:
        overlay = build_overlay_svg(CSV_PATH)
        patch_svg(SVG_PATH, overlay)
        print(f"SVG overlay: {SVG_PATH}")
        if SVG_DOCS_PATH.parent.is_dir():
            shutil.copy2(SVG_PATH, SVG_DOCS_PATH)
            print(f"SVG copié: {SVG_DOCS_PATH}")

    print("\nPour annuler avant validation:")
    print(f"  python {Path(__file__).name} --restore \"{backup}\"")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
