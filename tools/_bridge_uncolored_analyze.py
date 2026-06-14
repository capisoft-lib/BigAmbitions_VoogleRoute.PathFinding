#!/usr/bin/env python3
"""Identify bridge roads not yet in fusion overlay (uncolored on map)."""
from __future__ import annotations

import csv
import statistics
from collections import defaultdict
from pathlib import Path

CSV = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"

# Already merged virtual parts (from preprocess_bridge_merge.py)
MERGED_SOURCE_ROADS = {
    "170", "173", "175", "168",  # deck, ramps, junction
    "176", "195", "177", "194", "178", "184", "179", "183",  # traversées
}
MERGED_VIRTUAL = {"1700", "1701", "1702", "1703", "1704", "1705", "1706", "1707"}
CENTER_DECK = {"181", "182", "185", "186", "190", "191", "192", "193"}  # yellow preview

# 2x2 unidirectionnel : 2 voies ville→ouest, 2 voies ouest→ville
LANES_2X2 = {
    0: {"dir": "ville->ouest", "roads": ("182", "181"), "city": 6711, "west": 2458},
    1: {"dir": "ville->ouest", "roads": ("185", "186"), "city": 12085, "west": 6530},
    2: {"dir": "ouest->ville", "roads": ("190", "192"), "west": 4151, "city": 946},
    3: {"dir": "ouest->ville", "roads": ("191", "193"), "west": 6775, "city": 6028},
}

BRIDGE_CONNECTORS = {"180", "187", "188", "189"}  # candidats zone sans couleur


def load_road_stats() -> dict[str, dict]:
    stats: dict[str, dict] = defaultdict(lambda: {"xs": [], "zs": [], "ys": [], "count": 0})
    with CSV.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["edgeType"] not in ("base", "synthetic_turn"):
                continue
            for pre in ("from", "to"):
                r = row[f"{pre}Road"]
                stats[r]["xs"].append(float(row[f"{pre}X"]))
                stats[r]["zs"].append(float(row[f"{pre}Z"]))
                stats[r]["ys"].append(float(row[f"{pre}Y"]))
                stats[r]["count"] += 1
    return stats


def bbox(road: str, stats: dict) -> tuple[float, float, float, float]:
    s = stats[road]
    return min(s["xs"]), max(s["xs"]), min(s["zs"]), max(s["zs"])


def main() -> None:
    stats = load_road_stats()

    print("=== 4 voies tablier central (2x2 unidirectionnel) ===\n")
    for lane, spec in LANES_2X2.items():
        r_fwd, r_rev = spec["roads"]
        print(f"Lane {lane} [{spec['dir']}]")
        print(f"  R{r_fwd} + R{r_rev}  city wp {spec['city']}  west wp {spec['west']}")
        for r in spec["roads"]:
            if r in stats:
                x0, x1, z0, z1 = bbox(r, stats)
                med_y = statistics.median(stats[r]["ys"])
                print(f"    R{r}: {stats[r]['count']} refs  X[{x0:.0f},{x1:.0f}] Z[{z0:.0f},{z1:.0f}] Y~{med_y:.1f}")

    print("\n=== Zone SANS couleur (pas dans overlay fusion) ===\n")

  # Roads in bridge bbox but not merged
    bridge_x = (-1700, -250)
    bridge_z = (-1350, -100)

    candidates = []
    for r, s in stats.items():
        if not r.isdigit():
            continue
        ri = int(r)
        if r in MERGED_VIRTUAL or r in CENTER_DECK:
            continue
        x0, x1, z0, z1 = bbox(r, stats)
        cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
        if not (bridge_x[0] <= cx <= bridge_x[1] and bridge_z[0] <= cz <= bridge_z[1]):
            continue
        if r in MERGED_SOURCE_ROADS:
            tag = "traversée (moitié non fusionnée?)"
        elif r in BRIDGE_CONNECTORS:
            tag = "CONNECTEUR -> tablier jaune"
        else:
            tag = "autre pont"
        candidates.append((ri, r, stats[r]["count"] // 2, x0, x1, z0, z1, tag))

    for _, r, nodes, x0, x1, z0, z1, tag in sorted(candidates):
        print(f"R{r:>3s}: ~{nodes:4d} wps  X[{x0:6.0f},{x1:6.0f}] Z[{z0:6.0f},{z1:6.0f}]  {tag}")

    print("\n=== Connexions connecteurs → tablier jaune ===\n")
    with CSV.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["edgeType"] != "base":
                continue
            fr, tr = row["fromRoad"], row["toRoad"]
            if fr in BRIDGE_CONNECTORS and tr in CENTER_DECK:
                print(f"  R{fr} -> R{tr}  {row['fromIndex']}->{row['toIndex']}")
            if tr in BRIDGE_CONNECTORS and fr in CENTER_DECK:
                print(f"  R{fr} -> R{tr}  {row['fromIndex']}->{row['toIndex']}")

    print("\n=== Connexions traversées fusionnées → zone grise / jaune ===\n")
    links = [
        ("1706", "182", "ville -> jaune L0"),
        ("1705", "185", "ville -> jaune L1"),
        ("188", "190", "ouest -> jaune L2"),
        ("189", "191", "ouest -> jaune L3"),
        ("181", "180", "jaune L0 -> connecteur"),
        ("186", "187", "jaune L1 -> connecteur"),
        ("192", "1704", "jaune L2 -> traversée"),
        ("193", "1703", "jaune L3 -> traversée"),
    ]
    with CSV.open(encoding="utf-8") as f:
        edges = list(csv.DictReader(f))
    for a, b, label in links:
        hits = [
            e
            for e in edges
            if e["edgeType"] == "base"
            and ((e["fromRoad"] == a and e["toRoad"] == b) or (e["fromRoad"] == b and e["toRoad"] == a))
        ]
        print(f"  {label}: {len(hits)} arete(s)  ({a}<->{b})")


if __name__ == "__main__":
    main()
