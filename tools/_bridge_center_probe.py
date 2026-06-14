import csv
from collections import defaultdict
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "data/backups/big_ambitions_enhanced_routes_pre_bridge_merge_2026-06-11.csv"
roads = defaultdict(lambda: {"lanes": set(), "zmin": 1e9, "zmax": -1e9, "xmin": 1e9, "xmax": -1e9})
with p.open(encoding="utf-8") as f:
    for row in csv.DictReader(f):
        for rc, lc, xc, zc in (
            ("fromRoad", "fromLane", "fromX", "fromZ"),
            ("toRoad", "toLane", "toX", "toZ"),
        ):
            r = row[rc]
            roads[r]["lanes"].add(row[lc])
            x, z = float(row[xc]), float(row[zc])
            roads[r]["xmin"] = min(roads[r]["xmin"], x)
            roads[r]["xmax"] = max(roads[r]["xmax"], x)
            roads[r]["zmin"] = min(roads[r]["zmin"], z)
            roads[r]["zmax"] = max(roads[r]["zmax"], z)

approach = [168, 170, 173, 175, 176, 177, 178, 179, 183, 184, 194, 195]
center = [181, 182, 185, 186, 190, 191, 192, 193]
for label, ids in (("Approches / bretelles (fusion actuelle)", approach), ("Tablier central autoroute (NON fusionné)", center)):
    print("==", label, "==")
    for r in ids:
        d = roads[str(r)]
        print(
            f"  R{r}: lanes={sorted(d['lanes'])} "
            f"x=[{d['xmin']:.0f},{d['xmax']:.0f}] z=[{d['zmin']:.0f},{d['zmax']:.0f}]"
        )
