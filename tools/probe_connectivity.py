#!/usr/bin/env python3
import csv
import sys
from collections import deque
from pathlib import Path

CSV = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "data/backups/big_ambitions_enhanced_routes_pre_bridge_merge_2026-06-11.csv"
forward: dict[int, list[int]] = {}
positions: dict[int, tuple[float, float]] = {}

with CSV.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        if row["edgeType"] not in ("base", "synthetic_turn"):
            continue
        a, b = int(row["fromIndex"]), int(row["toIndex"])
        forward.setdefault(a, []).append(b)
        positions[a] = (float(row["fromX"]), float(row["fromZ"]))
        positions[b] = (float(row["toX"]), float(row["toZ"]))

# BFS without lane-flow (optimistic connectivity)
def bfs_reach(start: int, max_steps=50000) -> set[int]:
    seen = {start}
    q = deque([start])
    steps = 0
    while q and steps < max_steps:
        steps += 1
        c = q.popleft()
        for n in forward.get(c, []):
            if n not in seen:
                seen.add(n)
                q.append(n)
    return seen

tests = [
    ("SW", 7733),
    ("NE", 1133),
    ("downtown", 516),
    ("industrial", 13382),
    ("SE", 3891),
    ("NW", 4929),
]

print(f"CSV: {CSV.name}")
for i, (na, a) in enumerate(tests):
    for nb, b in tests[i + 1 :]:
        from_a = bfs_reach(a)
        ok = b in from_a
        from_b = bfs_reach(b) if not ok else from_a
        print(f"  {na}({a}) -> {nb}({b}): {'CONNECTED' if ok else 'DISCONNECTED'}  | reach(a)={len(from_a)} reach(b)={len(from_b)}")
