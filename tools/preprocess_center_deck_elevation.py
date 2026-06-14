#!/usr/bin/env python3
"""
Subdivise les segments droits Road 1708 (fusion tablier) en reprenant le profil
d'élévation du backup : pics locaux sur le chemin d'origine, max 5 segments/couloir.

Les extrémités (portails) conservent leur altitude actuelle ; les points intermédiaires
reprennent X/Y/Z du backup.

Usage:
  python preprocess_center_deck_elevation.py
  python preprocess_center_deck_elevation.py --backup ../data/backups/...csv
  python preprocess_center_deck_elevation.py --restore BACKUP.csv
"""
from __future__ import annotations

import argparse
import csv
import re
import shutil
from collections import defaultdict, deque
from datetime import date, datetime
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
CSV_PATH = DATA_DIR / "big_ambitions_enhanced_routes.csv"
BACKUP_DIR = DATA_DIR / "backups"

MERGED_ROAD = "1708"
SOURCE_PREFIX = "manual_bridge_center"
MAX_SEGMENTS = 5

LANE_ROAD: dict[str, int] = {
    "182": 0, "181": 0, "180": 0,
    "185": 1, "186": 1, "187": 1,
    "190": 2, "192": 2, "188": 2,
    "191": 3, "193": 3, "189": 3,
}

LANE_PORTALS: dict[int, tuple[int, int]] = {
    0: (6711, 2098),
    1: (12085, 10002),
    2: (9742, 946),
    3: (2152, 6028),
}


def backup_csv(csv_path: Path, tag: str) -> Path:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = date.today().isoformat()
    backup = BACKUP_DIR / f"big_ambitions_enhanced_routes_{tag}_{stamp}.csv"
    if backup.exists():
        backup = BACKUP_DIR / (
            f"big_ambitions_enhanced_routes_{tag}_{stamp}_"
            f"{datetime.now().strftime('%H%M%S')}.csv"
        )
    shutil.copy2(csv_path, backup)
    return backup


def default_pre_merge_backup() -> Path:
    candidates = sorted(
        BACKUP_DIR.glob("big_ambitions_enhanced_routes_pre_center_deck_*.csv"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError("Aucun backup pre_center_deck trouvé dans data/backups/")
    return candidates[0]


def lane_roads(lane: int) -> set[str]:
    return {r for r, lv in LANE_ROAD.items() if lv == lane}


def load_graph(path: Path) -> tuple[dict[int, dict], dict[int, list[int]]]:
    pts: dict[int, dict] = {}
    fwd: dict[int, list[int]] = defaultdict(list)
    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["edgeType"] not in ("base", "synthetic_turn"):
                continue
            a, b = int(row["fromIndex"]), int(row["toIndex"])
            for pre, idx in (("from", a), ("to", b)):
                pts[idx] = {
                    "x": float(row[f"{pre}X"]),
                    "y": float(row[f"{pre}Y"]),
                    "z": float(row[f"{pre}Z"]),
                    "name": row[f"{pre}Name"],
                    "road": row[f"{pre}Road"],
                    "lane": row[f"{pre}Lane"],
                }
            fwd[a].append(b)
    return pts, dict(fwd)


def trace_path(
    pts: dict[int, dict],
    fwd: dict[int, list[int]],
    start: int,
    end: int,
    allowed_roads: set[str],
) -> list[int] | None:
    q: deque[tuple[int, list[int]]] = deque([(start, [start])])
    seen = {start}
    while q:
        cur, path = q.popleft()
        if cur == end:
            return path
        for nxt in fwd.get(cur, []):
            if nxt in seen:
                continue
            if nxt != end and pts[nxt]["road"] not in allowed_roads:
                continue
            seen.add(nxt)
            q.append((nxt, path + [nxt]))
    return None


def find_local_peaks(path: list[int], pts: dict[int, dict]) -> list[int]:
    peaks: list[int] = []
    for j in range(1, len(path) - 1):
        wp = path[j]
        y0, y1, y2 = pts[path[j - 1]]["y"], pts[wp]["y"], pts[path[j + 1]]["y"]
        if y1 >= y0 and y1 >= y2 and (y1 > y0 or y1 > y2):
            peaks.append(wp)
    return peaks


def pick_cut_chain(
    backup_pts: dict[int, dict],
    backup_fwd: dict[int, list[int]],
    portal_a: int,
    portal_b: int,
    lane: int,
    current_pts: dict[int, dict],
) -> list[int]:
    roads = lane_roads(lane)
    path = trace_path(backup_pts, backup_fwd, portal_a, portal_b, roads)
    if not path:
        raise RuntimeError(f"Chemin backup introuvable L{lane}: {portal_a} -> {portal_b}")

    peaks = find_local_peaks(path, backup_pts)
    chain = [portal_a] + peaks + [portal_b]

    # Limiter à MAX_SEGMENTS (donc MAX_SEGMENTS - 1 points intermédiaires)
    max_internals = MAX_SEGMENTS - 1
    if len(chain) - 2 > max_internals:
        internals = chain[1:-1]
        step = len(internals) / max_internals
        picked = [internals[int(i * step)] for i in range(max_internals)]
        chain = [portal_a] + picked + [portal_b]

    # Extrémités : altitude du CSV actuel (portails)
    out = [portal_a]
    for wp in chain[1:-1]:
        out.append(wp)
    out.append(portal_b)
    return out


def remap_peak_name(name: str, lane: int) -> str:
    if not name:
        return f"Road_{MERGED_ROAD}-Lane_{lane}-Elev"
    m = re.search(r"Waypoint_(\d+)$", name)
    suffix = m.group(1) if m else "peak"
    return f"Road_{MERGED_ROAD}-Lane_{lane}-Waypoint_{suffix}"


def waypoint_data(
    idx: int,
    lane: int,
    backup_pts: dict[int, dict],
    current_pts: dict[int, dict],
    is_portal: bool,
) -> dict:
    if is_portal:
        p = current_pts[idx]
        name = p.get("name") or f"Road_{MERGED_ROAD}-Lane_{lane}-Portal"
    else:
        p = backup_pts[idx]
        name = remap_peak_name(p.get("name", ""), lane)
    return {
        "x": p["x"],
        "y": p["y"],
        "z": p["z"],
        "name": name,
    }


def make_straight_edge(
    edge_id: int,
    lane: int,
    a: int,
    b: int,
    wa: dict,
    wb: dict,
    source_suffix: str,
) -> dict[str, str]:
    cx = wa["x"]
    cz = (wa["z"] + wb["z"]) / 2.0
    cy = (wa["y"] + wb["y"]) / 2.0
    return {
        "edgeId": str(edge_id),
        "edgeType": "synthetic_turn",
        "maneuver": "straight",
        "fromIndex": str(a),
        "fromName": wa["name"],
        "fromRoad": MERGED_ROAD,
        "fromLane": str(lane),
        "fromX": f"{wa['x']:.3f}",
        "fromY": f"{wa['y']:.3f}",
        "fromZ": f"{wa['z']:.3f}",
        "toIndex": str(b),
        "toName": wb["name"],
        "toRoad": MERGED_ROAD,
        "toLane": str(lane),
        "toX": f"{wb['x']:.3f}",
        "toY": f"{wb['y']:.3f}",
        "toZ": f"{wb['z']:.3f}",
        "controlX": f"{cx:.3f}",
        "controlY": f"{cy:.3f}",
        "controlZ": f"{cz:.3f}",
        "angleDegrees": "0.00",
        "fromLaneIsLeftmostTurnLane": "1",
        "source": f"{SOURCE_PREFIX}_L{lane}_{source_suffix}",
        "bridgePart": "bridge_center_deck",
    }


def apply_elevation(
    csv_path: Path,
    pre_merge_backup: Path,
) -> tuple[int, int, dict[int, list[int]]]:
    backup_pts, backup_fwd = load_graph(pre_merge_backup)

    with csv_path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if "bridgePart" not in fieldnames:
        fieldnames.append("bridgePart")

    current_pts, _ = load_graph(csv_path)
    kept = [r for r in rows if not r.get("source", "").startswith(SOURCE_PREFIX)]
    removed = len(rows) - len(kept)

    chains: dict[int, list[int]] = {}
    next_id = max(int(r["edgeId"]) for r in kept) + 1
    added = 0

    for lane, (portal_a, portal_b) in LANE_PORTALS.items():
        chain_ab = pick_cut_chain(
            backup_pts, backup_fwd, portal_a, portal_b, lane, current_pts
        )
        chain_ba = list(reversed(chain_ab))
        chains[lane] = chain_ab

        for chain, suffix in ((chain_ab, "fwd"), (chain_ba, "rev")):
            for i in range(len(chain) - 1):
                a, b = chain[i], chain[i + 1]
                wa = waypoint_data(a, lane, backup_pts, current_pts, i == 0)
                wb = waypoint_data(b, lane, backup_pts, current_pts, i + 1 == len(chain) - 1)
                kept.append(make_straight_edge(next_id, lane, a, b, wa, wb, f"{suffix}_{i}"))
                next_id += 1
                added += 1

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(kept)

    sync_csv_to_mod_data(csv_path)

    return removed, added, chains


def sync_csv_to_mod_data(csv_path: Path) -> Path:
    """Copie le CSV source PathFinding/data vers VoogleRoute/Data (déployé en jeu)."""
    mod_data = csv_path.resolve().parents[2] / "Data" / csv_path.name
    mod_data.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(csv_path, mod_data)
    return mod_data


def print_summary(chains: dict[int, list[int]], backup_pts: dict[int, dict], current_pts: dict[int, dict]) -> None:
    print("\nProfil appliqué (extrémités = CSV actuel, milieu = backup):")
    for lane, chain in sorted(chains.items()):
        print(f"\n  Lane {lane}: {' -> '.join(str(w) for w in chain)} ({len(chain)-1} segments)")
        for i in range(len(chain) - 1):
            a, b = chain[i], chain[i + 1]
            ya = current_pts[a]["y"] if i == 0 else backup_pts[a]["y"]
            yb = current_pts[b]["y"] if i + 1 == len(chain) - 1 else backup_pts[b]["y"]
            dist = ((backup_pts[b]["x"] - backup_pts[a]["x"]) ** 2 + (backup_pts[b]["z"] - backup_pts[a]["z"]) ** 2) ** 0.5
            slope = abs(yb - ya) / dist * 100 if dist else 0.0
            print(f"    {a}->{b}: Y {ya:.2f}->{yb:.2f}  pente~{slope:.2f}%")


def main() -> int:
    parser = argparse.ArgumentParser(description="Subdivise Road 1708 avec élévation backup.")
    parser.add_argument("--backup", metavar="PRE_MERGE_CSV", help="Backup avant fusion center_deck.")
    parser.add_argument("--restore", metavar="BACKUP_CSV")
    args = parser.parse_args()

    if args.restore:
        shutil.copy2(Path(args.restore), CSV_PATH)
        print(f"Restored {CSV_PATH} <= {args.restore}")
        return 0

    if not CSV_PATH.is_file():
        print(f"CSV not found: {CSV_PATH}")
        return 1

    pre_merge = Path(args.backup) if args.backup else default_pre_merge_backup()
    if not pre_merge.is_file():
        print(f"Backup introuvable: {pre_merge}")
        return 1

    has_center_segments = False
    with CSV_PATH.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("source", "").startswith(SOURCE_PREFIX):
                has_center_segments = True
                break
    if not has_center_segments:
        print("Aucun segment manual_bridge_center dans le CSV. Fusion center_deck déjà subdivisée?")
        return 2

    local_backup = backup_csv(CSV_PATH, "pre_center_deck_elevation")
    print(f"Backup local: {local_backup}")
    print(f"Profil élévation depuis: {pre_merge}")

    backup_pts, _ = load_graph(pre_merge)
    current_pts, _ = load_graph(CSV_PATH)
    removed, added, chains = apply_elevation(CSV_PATH, pre_merge)
    print(f"CSV mis à jour: {CSV_PATH}")
    print(f"  segments droits supprimés: {removed}")
    print(f"  segments subdivisés ajoutés: {added}")
    print(f"  Data/ synchronisé: {sync_csv_to_mod_data(CSV_PATH)}")
    print_summary(chains, backup_pts, current_pts)
    print(f"\nRestaurer: python {Path(__file__).name} --restore \"{local_backup}\"")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
