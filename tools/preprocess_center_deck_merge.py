#!/usr/bin/env python3
"""
Fusion tablier autoroute: R181-R193 + bretelles R180/R187/R188/R189 -> Road 1708 (4 lanes).

- Supprime les arêtes base internes au couloir
- Ajoute 4 segments droits bidirectionnels (2x2 unidirectionnel, 1 segment/lane)
- Ignore l'altitude (Y) pour les contrôles de virage synthétique

Usage:
  python preprocess_center_deck_merge.py              # backup + merge
  python preprocess_center_deck_merge.py --restore BACKUP.csv
"""
from __future__ import annotations

import argparse
import csv
import re
import shutil
from datetime import date, datetime
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
CSV_PATH = DATA_DIR / "big_ambitions_enhanced_routes.csv"
BACKUP_DIR = DATA_DIR / "backups"

MERGED_ROAD = "1708"
MERGE_SOURCE_ROADS = {
    "181", "182", "185", "186", "190", "191", "192", "193",
    "180", "187", "188", "189",
}

ROAD_TO_LANE: dict[str, str] = {
    "182": "0", "181": "0", "180": "0",
    "185": "1", "186": "1", "187": "1",
    "190": "2", "192": "2", "188": "2",
    "191": "3", "193": "3", "189": "3",
}

# Portails externes par couloir (extrémités hors graphe dense)
LANE_PORTALS: dict[int, tuple[int, int]] = {
    0: (6711, 2098),    # ville (1706) <-> ouest (1706)
    1: (12085, 10002),  # ville (1705) <-> ouest (1705)
    2: (9742, 946),     # ouest (1704->188) <-> ville (192->1704)
    3: (2152, 6028),    # ouest (1703->189) <-> ville (193->1703)
}


def backup_csv(csv_path: Path) -> Path:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = date.today().isoformat()
    backup = BACKUP_DIR / f"big_ambitions_enhanced_routes_pre_center_deck_{stamp}.csv"
    if backup.exists():
        backup = BACKUP_DIR / (
            f"big_ambitions_enhanced_routes_pre_center_deck_{stamp}_"
            f"{datetime.now().strftime('%H%M%S')}.csv"
        )
    shutil.copy2(csv_path, backup)
    return backup


def remap_name(name: str, old_road: str, new_road: str, new_lane: str) -> str:
    if not name:
        return name
    return re.sub(
        rf"Road_{re.escape(old_road)}-Lane_\d+",
        f"Road_{new_road}-Lane_{new_lane}",
        name,
        count=1,
    )


def remap_row(row: dict[str, str]) -> dict[str, str]:
    out = dict(row)
    touched = False
    for prefix in ("from", "to"):
        road = row[f"{prefix}Road"]
        if road not in MERGE_SOURCE_ROADS:
            continue
        lane = ROAD_TO_LANE[road]
        out[f"{prefix}Road"] = MERGED_ROAD
        out[f"{prefix}Lane"] = lane
        out[f"{prefix}Name"] = remap_name(row[f"{prefix}Name"], road, MERGED_ROAD, lane)
        touched = True
    if touched:
        part = row.get("bridgePart", "")
        out["bridgePart"] = part or "bridge_center_deck"
    return out


def load_waypoints(rows: list[dict[str, str]]) -> dict[int, dict[str, float | str]]:
    pts: dict[int, dict[str, float | str]] = {}
    for row in rows:
        for prefix in ("from", "to"):
            idx = int(row[f"{prefix}Index"])
            pts[idx] = {
                "x": float(row[f"{prefix}X"]),
                "y": float(row[f"{prefix}Y"]),
                "z": float(row[f"{prefix}Z"]),
                "name": row[f"{prefix}Name"],
            }
    return pts


def make_straight_edge(
    edge_id: int,
    lane: int,
    a: int,
    b: int,
    pts: dict[int, dict[str, float | str]],
    source_suffix: str,
) -> dict[str, str]:
    pa, pb = pts[a], pts[b]
    cx = float(pa["x"])
    cz = (float(pa["z"]) + float(pb["z"])) / 2.0
    cy = (float(pa["y"]) + float(pb["y"])) / 2.0
    return {
        "edgeId": str(edge_id),
        "edgeType": "synthetic_turn",
        "maneuver": "straight",
        "fromIndex": str(a),
        "fromName": f"Road_{MERGED_ROAD}-Lane_{lane}-Portal_A",
        "fromRoad": MERGED_ROAD,
        "fromLane": str(lane),
        "fromX": f"{float(pa['x']):.3f}",
        "fromY": f"{float(pa['y']):.3f}",
        "fromZ": f"{float(pa['z']):.3f}",
        "toIndex": str(b),
        "toName": f"Road_{MERGED_ROAD}-Lane_{lane}-Portal_B",
        "toRoad": MERGED_ROAD,
        "toLane": str(lane),
        "toX": f"{float(pb['x']):.3f}",
        "toY": f"{float(pb['y']):.3f}",
        "toZ": f"{float(pb['z']):.3f}",
        "controlX": f"{cx:.3f}",
        "controlY": f"{cy:.3f}",
        "controlZ": f"{cz:.3f}",
        "angleDegrees": "0.00",
        "fromLaneIsLeftmostTurnLane": "1",
        "source": f"manual_bridge_center_L{lane}_{source_suffix}",
        "bridgePart": "bridge_center_deck",
    }


def merge_csv(csv_path: Path) -> tuple[int, int, int]:
    with csv_path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if "bridgePart" not in fieldnames:
        fieldnames.append("bridgePart")

    remapped_rows: list[dict[str, str]] = []
    removed_internal = 0
    for row in rows:
        fr, tr = row["fromRoad"], row["toRoad"]
        if row["edgeType"] == "base" and fr in MERGE_SOURCE_ROADS and tr in MERGE_SOURCE_ROADS:
            removed_internal += 1
            continue
        remapped_rows.append(remap_row(row))

    pts = load_waypoints(remapped_rows)
    next_id = max(int(r["edgeId"]) for r in remapped_rows) + 1
    added = 0
    for lane, (a, b) in LANE_PORTALS.items():
        if a not in pts or b not in pts:
            raise RuntimeError(f"Portail manquant L{lane}: {a} ou {b}")
        remapped_rows.append(make_straight_edge(next_id, lane, a, b, pts, "fwd"))
        next_id += 1
        remapped_rows.append(make_straight_edge(next_id, lane, b, a, pts, "rev"))
        next_id += 1
        added += 2

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(remapped_rows)

    touched = sum(
        1
        for row in remapped_rows
        if row.get("bridgePart") == "bridge_center_deck"
    )
    return removed_internal, added, touched


def restore_csv(backup: Path, csv_path: Path) -> None:
    shutil.copy2(backup, csv_path)
    print(f"Restored {csv_path} <= {backup}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion tablier central -> Road 1708")
    parser.add_argument("--restore", metavar="BACKUP_CSV")
    args = parser.parse_args()

    if args.restore:
        restore_csv(Path(args.restore), CSV_PATH)
        return 0

    if not CSV_PATH.is_file():
        print(f"CSV not found: {CSV_PATH}")
        return 1

    # Déjà fusionné ?
    with CSV_PATH.open(encoding="utf-8") as f:
        sample = f.read(8000)
    if f"Road_{MERGED_ROAD}-" in sample or ",1708," in sample:
        print("CSV semble déjà contenir Road 1708. Restaurez un backup avant de relancer.")
        return 2

    backup = backup_csv(CSV_PATH)
    print(f"Backup: {backup}")

    removed, added, touched = merge_csv(CSV_PATH)
    print(f"CSV fusionné: {CSV_PATH}")
    print(f"  arêtes internes supprimées: {removed}")
    print(f"  segments droits ajoutés: {added}")
    print(f"  lignes bridge_center_deck: {touched}")
    print(f"\nRestaurer: python {Path(__file__).name} --restore \"{backup}\"")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
