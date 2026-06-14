import csv
from collections import defaultdict

path = r"c:\Users\AI\AI\Dev\Cursor\BigAmbitions\bigambitions\Assets\Mods\VoogleRoute\PathFinding\data\big_ambitions_enhanced_routes.csv"
roads = defaultdict(lambda: {"lanes": set(), "xmin": 1e9, "xmax": -1e9, "zmin": 1e9, "zmax": -1e9, "nodes": set()})

with open(path, newline="", encoding="utf-8") as f:
    for row in csv.DictReader(f):
        for road_col, lane_col, idx_col, x_col, z_col in (
            ("fromRoad", "fromLane", "fromIndex", "fromX", "fromZ"),
            ("toRoad", "toLane", "toIndex", "toX", "toZ"),
        ):
            rid = row.get(road_col)
            lane = row.get(lane_col)
            wp = row.get(idx_col)
            x = float(row.get(x_col) or 0)
            z = float(row.get(z_col) or 0)
            if not rid:
                continue
            r = int(rid)
            roads[r]["lanes"].add(int(lane) if lane else -1)
            roads[r]["xmin"] = min(roads[r]["xmin"], x)
            roads[r]["xmax"] = max(roads[r]["xmax"], x)
            roads[r]["zmin"] = min(roads[r]["zmin"], z)
            roads[r]["zmax"] = max(roads[r]["zmax"], z)
            if wp:
                roads[r]["nodes"].add(int(wp))

bridge_ids = {168, 170, 173, 175, 176, 177, 178, 179, 183, 184, 194, 195, 144, 196, 148}
print("=== Bridge-related roads (explicit set) ===")
for r in sorted(bridge_ids):
    if r not in roads:
        print(f"Road {r}: NOT IN CSV")
        continue
    d = roads[r]
    lanes = sorted(d["lanes"])
    print(
        f"Road {r:3d}: lanes={lanes} nodes={len(d['nodes']):4d} "
        f"x=[{d['xmin']:7.1f},{d['xmax']:7.1f}] z=[{d['zmin']:7.1f},{d['zmax']:7.1f}]"
    )
