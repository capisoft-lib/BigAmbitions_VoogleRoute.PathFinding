#!/usr/bin/env python3
"""Find map extremes and report A* viability via subprocess to DiagRunner."""
import csv
import math
import subprocess
import sys
from pathlib import Path

CSV = Path(__file__).resolve().parents[1] / "data" / "big_ambitions_enhanced_routes.csv"
if len(sys.argv) > 1:
    CSV = Path(sys.argv[1])

positions: dict[int, tuple[float, float]] = {}
with CSV.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        for idx_col, x_col, z_col in (("fromIndex", "fromX", "fromZ"), ("toIndex", "toX", "toZ")):
            i = int(row[idx_col])
            positions[i] = (float(row[x_col]), float(row[z_col]))

xs = [p[0] for p in positions.values()]
zs = [p[1] for p in positions.values()]
bounds = (min(xs), max(xs), min(zs), max(zs))
print(f"CSV: {CSV.name}")
print(f"Bounds X [{bounds[0]:.1f}, {bounds[1]:.1f}] Z [{bounds[2]:.1f}, {bounds[3]:.1f}]")
print(f"Waypoints: {len(positions)}")

corners = [
    ("SW", bounds[0], bounds[2]),
    ("SE", bounds[1], bounds[2]),
    ("NW", bounds[0], bounds[3]),
    ("NE", bounds[1], bounds[3]),
    ("downtown", 131.11, 123.24),
    ("industrial", -1740.94, -1163.29),
    ("bridge_city", -593.9, -393.9),
]


def nearest(x: float, z: float) -> tuple[int, float]:
    best_i, best_d = -1, 1e18
    for i, (wx, wz) in positions.items():
        d = math.hypot(wx - x, wz - z)
        if d < best_d:
            best_i, best_d = i, d
    return best_i, best_d


corner_wps = {}
for name, x, z in corners:
    wp, d = nearest(x, z)
    wx, wz = positions[wp]
    corner_wps[name] = (wp, wx, wz, d)
    print(f"  {name:12s} wp={wp:5d} at ({wx:7.1f},{wz:7.1f}) snap={d:.1f}m")

pairs = []
names = [c[0] for c in corners]
for i, a in enumerate(names):
    for b in names[i + 1 :]:
        wa = corner_wps[a]
        wb = corner_wps[b]
        dist = math.hypot(wa[1] - wb[1], wa[2] - wb[2])
        pairs.append((dist, a, b, wa[0], wb[0]))

pairs.sort(reverse=True)
print("\nTop distance pairs (waypoint-to-waypoint):")
for dist, a, b, sa, sb in pairs[:12]:
    print(f"  {dist:7.0f}m  {a:12s} ({sa}) -> {b:12s} ({sb})")
