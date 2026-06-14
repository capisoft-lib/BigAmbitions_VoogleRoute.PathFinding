#!/usr/bin/env python3
"""
Collapse Road 1703 (R176/R195) : 2 voies unidirectionnelles -> 2 segments droits.

Lane 0 (court, côté industriel/1708) : 12992 <-> 10744
Lane 1 (diagonale ville)              : 4441 <-> 7679

Usage:
  python preprocess_1703_collapse.py
  python preprocess_1703_collapse.py --restore BACKUP.csv
"""
from __future__ import annotations

import argparse
import csv
import shutil
from datetime import date, datetime
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
CSV_PATH = DATA_DIR / "big_ambitions_enhanced_routes.csv"
BACKUP_DIR = DATA_DIR / "backups"

TARGET_ROAD = "1703"
BRIDGE_PART = "bridge_cross_176_195"
SOURCE_TAG = "manual_bridge_1703_collapse"

# lane -> (portal_a, portal_b)
LANE_PORTALS: dict[int, tuple[int, int]] = {
    0: (12992, 10744),  # R171 -> 1708 (industriel)
    1: (4441, 7679),    # 1708 -> R124 (diagonale ville)
}


def backup_csv(csv_path: Path) -> Path:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = date.today().isoformat()
    backup = BACKUP_DIR / f"big_ambitions_enhanced_routes_pre_1703_collapse_{stamp}.csv"
    if backup.exists():
        backup = BACKUP_DIR / (
            f"big_ambitions_enhanced_routes_pre_1703_collapse_{stamp}_"
            f"{datetime.now().strftime('%H%M%S')}.csv"
        )
    shutil.copy2(csv_path, backup)
    return backup


def load_waypoints(rows: list[dict[str, str]]) -> dict[int, dict[str, float]]:
    pts: dict[int, dict[str, float]] = {}
    for row in rows:
        for prefix in ("from", "to"):
            idx = int(row[f"{prefix}Index"])
            pts[idx] = {
                "x": float(row[f"{prefix}X"]),
                "y": float(row[f"{prefix}Y"]),
                "z": float(row[f"{prefix}Z"]),
            }
    return pts


def make_straight_edge(
    edge_id: int,
    lane: int,
    a: int,
    b: int,
    pts: dict[int, dict[str, float]],
    suffix: str,
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
        "fromName": f"Road_{TARGET_ROAD}-Lane_{lane}-Portal_A",
        "fromRoad": TARGET_ROAD,
        "fromLane": str(lane),
        "fromX": f"{pa['x']:.3f}",
        "fromY": f"{pa['y']:.3f}",
        "fromZ": f"{pa['z']:.3f}",
        "toIndex": str(b),
        "toName": f"Road_{TARGET_ROAD}-Lane_{lane}-Portal_B",
        "toRoad": TARGET_ROAD,
        "toLane": str(lane),
        "toX": f"{pb['x']:.3f}",
        "toY": f"{pb['y']:.3f}",
        "toZ": f"{pb['z']:.3f}",
        "controlX": f"{cx:.3f}",
        "controlY": f"{cy:.3f}",
        "controlZ": f"{cz:.3f}",
        "angleDegrees": "0.00",
        "fromLaneIsLeftmostTurnLane": "1",
        "source": f"{SOURCE_TAG}_L{lane}_{suffix}",
        "bridgePart": BRIDGE_PART,
    }


def collapse_csv(csv_path: Path) -> tuple[int, int]:
    with csv_path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if "bridgePart" not in fieldnames:
        fieldnames.append("bridgePart")

    if any(row.get("source", "").startswith(SOURCE_TAG) for row in rows):
        raise RuntimeError("1703 semble déjà collapsed (source manual_bridge_1703_collapse).")

    kept: list[dict[str, str]] = []
    removed = 0
    for row in rows:
        if (
            row["edgeType"] == "base"
            and row["fromRoad"] == TARGET_ROAD
            and row["toRoad"] == TARGET_ROAD
        ):
            removed += 1
            continue
        kept.append(row)

    pts = load_waypoints(kept)
    next_id = max(int(r["edgeId"]) for r in kept) + 1
    added = 0
    for lane, (a, b) in LANE_PORTALS.items():
        if a not in pts or b not in pts:
            raise RuntimeError(f"Portail L{lane} manquant: {a}, {b}")
        kept.append(make_straight_edge(next_id, lane, a, b, pts, "fwd"))
        next_id += 1
        kept.append(make_straight_edge(next_id, lane, b, a, pts, "rev"))
        next_id += 1
        added += 2

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(kept)

    return removed, added


def main() -> int:
    parser = argparse.ArgumentParser(description="Collapse Road 1703 en 2 segments.")
    parser.add_argument("--restore", metavar="BACKUP_CSV")
    args = parser.parse_args()

    if args.restore:
        shutil.copy2(Path(args.restore), CSV_PATH)
        print(f"Restored {CSV_PATH}")
        return 0

    if not CSV_PATH.is_file():
        print(f"CSV not found: {CSV_PATH}")
        return 1

    backup = backup_csv(CSV_PATH)
    print(f"Backup: {backup}")

    removed, added = collapse_csv(CSV_PATH)
    print(f"Collapse 1703: removed={removed} internal base edges, added={added} synthetic straights")
    print(f"\nRestore: python {Path(__file__).name} --restore \"{backup}\"")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
