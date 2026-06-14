#!/usr/bin/env python3
import csv
from collections import defaultdict
from pathlib import Path

focus = ["180", "187", "188", "189", "176", "177", "178", "179", "183", "184", "194", "195"]
p = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
stats = defaultdict(lambda: {"n": 0, "xs": [], "zs": []})
with p.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        if row["edgeType"] != "base":
            continue
        for pre in ("from", "to"):
            r = row[pre + "Road"]
            if r in focus:
                stats[r]["n"] += 1
                stats[r]["xs"].append(float(row[pre + "X"]))
                stats[r]["zs"].append(float(row[pre + "Z"]))

for r in focus:
    s = stats[r]
    if not s["n"]:
        print("R" + r, "absent")
        continue
    dx = max(s["xs"]) - min(s["xs"])
    dz = max(s["zs"]) - min(s["zs"])
    span = (dx * dx + dz * dz) ** 0.5
    print(
        f"R{r}: wps~{s['n'] // 2:3d}  X[{min(s['xs']):6.0f},{max(s['xs']):6.0f}]  "
        f"Z[{min(s['zs']):6.0f},{max(s['zs']):6.0f}]  span={span:.0f}m"
    )
