#!/usr/bin/env python3
"""Full chain per lane: traverse -> connector -> center -> connector -> traverse."""
import csv
from collections import defaultdict, deque
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
fwd = defaultdict(list)
road = {}

with p.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        if row["edgeType"] not in ("base", "synthetic_turn"):
            continue
        a, b = int(row["fromIndex"]), int(row["toIndex"])
        fwd[a].append(b)
        road[a] = row["fromRoad"]
        road[b] = row["toRoad"]

CHAINS = [
    ("L0 ville->ind", 6847, {1706, 182, 181, 180, 1703, 193}),
    ("L1 ville->ind", 4713, {1705, 185, 186, 187, 1704, 192}),
    ("L2 ind->ville", 4441, {1703, 193, 191, 189}),
    ("L3 ind->ville", 473, {1704, 192, 190, 188}),
]


def bfs_roads(start: int, allowed: set[str], limit=300) -> list[str]:
    q = deque([(start, [road.get(start, "?")])])
    seen = {start}
    best = []
    while q:
        u, path = q.popleft()
        if len(path) > len(best):
            best = path
        if len(path) >= limit:
            break
        for v in fwd[u]:
            if v in seen:
                continue
            rv = road.get(v, "?")
            if rv not in allowed and rv != road.get(u, ""):
                continue
            seen.add(v)
            q.append((v, path + [rv]))
    return best


for label, start, allowed in CHAINS:
    path = bfs_roads(start, allowed)
    print(label, "->", " -> ".join(path[:20]))

print("\n=== Chaine complete L0 depuis 6847 (sans filtre) ===")
q = deque([(6847, [6847])])
seen = {6847}
roads_seq = []
while q and len(seen) < 400:
    u, path = q.popleft()
    if len(path) > len(roads_seq):
        roads_seq = [road.get(w, "?") for w in path]
    for v in fwd[u]:
        if v not in seen:
            seen.add(v)
            q.append((v, path + [v]))
print("roads:", " -> ".join(roads_seq[:25]), "... total wps", len(roads_seq))
