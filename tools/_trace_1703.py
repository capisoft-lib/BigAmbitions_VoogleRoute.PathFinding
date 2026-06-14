#!/usr/bin/env python3
import csv
from collections import defaultdict
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
fwd = defaultdict(list)
road = {}
lane_wp = {}

with p.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        if row["edgeType"] not in ("base", "synthetic_turn"):
            continue
        a, b = int(row["fromIndex"]), int(row["toIndex"])
        fwd[a].append(b)
        road[a] = row["fromRoad"]
        road[b] = row["toRoad"]
        if row["fromRoad"] == "1703":
            lane_wp[a] = row["fromLane"]

def trace(start, max_n=300):
    path = [start]
    cur = start
    seen = {start}
    for _ in range(max_n):
        nxts = [n for n in fwd[cur] if n not in seen]
        if not nxts:
            break
        # prefer staying on 1703
        stay = [n for n in nxts if road.get(n) == "1703"]
        nxt = stay[0] if stay else nxts[0]
        path.append(nxt)
        seen.add(nxt)
        cur = nxt
        if road.get(cur) != "1703":
            break
    return path

for label, start in [("L0 from IN 12992", 12992), ("L1 from IN 4441", 4441)]:
    path = trace(start)
    print(label, "len", len(path), "end wp", path[-1], "road", road.get(path[-1]))
    print("  lanes", [lane_wp.get(w, "?") for w in path[:5]], "...")
