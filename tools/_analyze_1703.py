#!/usr/bin/env python3
import csv
from collections import defaultdict, deque
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
pts = {}
fwd = defaultdict(list)
rev = defaultdict(list)

with p.open(encoding="utf-8") as f:
    rows = list(csv.DictReader(f))
    for row in rows:
        if row["edgeType"] not in ("base", "synthetic_turn"):
            continue
        a, b = int(row["fromIndex"]), int(row["toIndex"])
        fwd[a].append(b)
        rev[b].append(a)
        for i, pre in ((a, "from"), (b, "to")):
            pts[i] = dict(
                x=float(row[pre + "X"]), z=float(row[pre + "Z"]), y=float(row[pre + "Y"]),
                r=row[pre + "Road"], l=row[pre + "Lane"], n=row[pre + "Name"],
            )

print("=== Boundary edges Road 1703 ===")
for row in rows:
    if row["edgeType"] != "base":
        continue
    fr, tr = row["fromRoad"], row["toRoad"]
    a, b = int(row["fromIndex"]), int(row["toIndex"])
    if fr == "1703" and tr != "1703":
        print(f"OUT L{row['fromLane']} R{fr}->R{tr} {a}->{b} ({pts[a]['x']:.0f},{pts[a]['z']:.0f})")
    if tr == "1703" and fr != "1703":
        print(f"IN  L{row['toLane']} R{fr}->R{tr} {a}->{b} ({pts[b]['x']:.0f},{pts[b]['z']:.0f})")

# trace lane 0 and 1 from each IN
for lane in ("0", "1"):
    ins = []
    outs = []
    for row in rows:
        if row["edgeType"] != "base":
            continue
        if row["toRoad"] == "1703" and row["toLane"] == lane and row["fromRoad"] != "1703":
            ins.append(int(row["toIndex"]))
        if row["fromRoad"] == "1703" and row["fromLane"] == lane and row["toRoad"] != "1703":
            outs.append(int(row["fromIndex"]))
    print(f"\nLane {lane}: IN wps {ins}, OUT wps {outs}")

# count internal base edges on 1703
internal = sum(
    1 for row in rows
    if row["edgeType"] == "base" and row["fromRoad"] == "1703" and row["toRoad"] == "1703"
)
print(f"\nInternal base edges on 1703: {internal}")
