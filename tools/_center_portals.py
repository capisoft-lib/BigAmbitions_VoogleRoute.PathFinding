#!/usr/bin/env python3
import csv
from collections import defaultdict
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
MERGE = {"181", "182", "185", "186", "190", "191", "192", "193", "180", "187", "188", "189"}
LANE_ROAD = {
    "182": 0, "181": 0, "180": 0,
    "185": 1, "186": 1, "187": 1,
    "190": 2, "192": 2, "188": 2,
    "191": 3, "193": 3, "189": 3,
}
pts = {}
with p.open(encoding="utf-8") as f:
    rows = list(csv.DictReader(f))
    for row in rows:
        for pre in ("from", "to"):
            i = int(row[pre + "Index"])
            pts[i] = dict(
                x=float(row[pre + "X"]), z=float(row[pre + "Z"]), y=float(row[pre + "Y"]),
                r=row[pre + "Road"], n=row[pre + "Name"],
            )

print("=== Boundary edges (merge <-> rest) ===")
for row in rows:
    if row["edgeType"] != "base":
        continue
    fr, tr = row["fromRoad"], row["toRoad"]
    a, b = int(row["fromIndex"]), int(row["toIndex"])
    if fr in MERGE and tr not in MERGE:
        lane = LANE_ROAD.get(fr, "?")
        print(f"L{lane} OUT  R{fr}->R{tr}  {a}->{b}  ({pts[a]['x']:.0f},{pts[a]['z']:.0f})")
    if tr in MERGE and fr not in MERGE:
        lane = LANE_ROAD.get(tr, "?")
        print(f"L{lane} IN   R{fr}->R{tr}  {a}->{b}  ({pts[b]['x']:.0f},{pts[b]['z']:.0f})")
