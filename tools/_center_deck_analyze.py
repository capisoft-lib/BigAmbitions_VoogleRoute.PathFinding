#!/usr/bin/env python3
import csv
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
pts: dict[int, dict] = {}
with p.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        for pre in ("from", "to"):
            i = int(row[pre + "Index"])
            pts[i] = {
                "x": float(row[pre + "X"]),
                "z": float(row[pre + "Z"]),
                "y": float(row[pre + "Y"]),
                "r": row[pre + "Road"],
                "n": row[pre + "Name"],
            }

boundaries = [
    ("L0 in 182", 6711),
    ("L0 out 181", 2458),
    ("L0 out 180", 5531),
    ("L1 in 185", 12085),
    ("L1 out 186", 6530),
    ("L1 out 187", 9236),
    ("L2 in 190", 4151),
    ("L2 out 192", 946),
    ("L2 out 1704", 473),
    ("L3 in 191", 6775),
    ("L3 out 193", 6028),
    ("L3 out 1703", 4441),
]
for label, wp in boundaries:
    q = pts.get(wp)
    if not q:
        print(label, "MISSING", wp)
        continue
    print(f"{label:14s} wp{wp:5d} R{q['r']:>4s} ({q['x']:8.1f},{q['z']:8.1f}) y={q['y']:5.1f}")
