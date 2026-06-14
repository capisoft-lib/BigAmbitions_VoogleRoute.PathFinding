"""Probe A* path stats through central bridge span (R181-R193)."""
import csv
from collections import defaultdict, deque
from pathlib import Path

# Use pre-merge backup for original road ids
CSV = Path(__file__).resolve().parents[1] / "data/backups/big_ambitions_enhanced_routes_pre_bridge_merge_2026-06-11.csv"

CENTER_ROADS = {"181", "182", "185", "186", "190", "191", "192", "193"}
APPROACH_ROADS = {"176", "177", "178", "179", "183", "184", "194", "195"}
DECK_INDUSTRIAL = {"170", "173", "175", "168"}

MAX_ASTAR = 32768


def load_graph(path: Path):
    positions: dict[int, tuple[float, float]] = {}
    road_by_wp: dict[int, str] = {}
    forward: dict[int, list[int]] = defaultdict(list)
    synthetic: set[tuple[int, int]] = set()

    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            et = row["edgeType"]
            if et not in ("base", "synthetic_turn"):
                continue
            a, b = int(row["fromIndex"]), int(row["toIndex"])
            fx, fz = float(row["fromX"]), float(row["fromZ"])
            tx, tz = float(row["toX"]), float(row["toZ"])
            positions[a] = (fx, fz)
            positions[b] = (tx, tz)
            road_by_wp[a] = row["fromRoad"]
            road_by_wp[b] = row["toRoad"]
            forward[a].append(b)
            if et == "synthetic_turn":
                synthetic.add((a, b))

    return positions, road_by_wp, dict(forward), synthetic


def flat_dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def astar(
    forward: dict[int, list[int]],
    positions: dict[int, tuple[float, float]],
    start: int,
    goal: int,
    max_nodes: int = MAX_ASTAR,
):
    open_list = [start]
    open_set = {start}
    came_from: dict[int, int] = {}
    g = {start: 0.0}
    f = {start: flat_dist(positions[start], positions[goal])}
    closed: set[int] = set()
    explored = 0
    center_wps_on_path: set[int] = set()
    center_edges_on_path = 0

    while open_list:
        explored += 1
        if explored > max_nodes:
            return None, explored, 0, 0, 0

        current = min(open_list, key=lambda n: f.get(n, 1e18))
        open_list.remove(current)
        open_set.discard(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                ru = road_by_wp.get(u, "")
                rv = road_by_wp.get(v, "")
                if ru in CENTER_ROADS or rv in CENTER_ROADS:
                    center_edges_on_path += 1
                    if ru in CENTER_ROADS:
                        center_wps_on_path.add(u)
                    if rv in CENTER_ROADS:
                        center_wps_on_path.add(v)
            return path, explored, len(path), len(center_wps_on_path), center_edges_on_path

        closed.add(current)
        inc = came_from.get(current, -1)
        g_cur = g[current]

        for nxt in forward.get(current, []):
            if nxt in closed:
                continue
            # simplified: allow all forward edges (no lane-flow filter)
            step = flat_dist(positions[current], positions[nxt])
            tent = g_cur + step
            if tent >= g.get(nxt, 1e18):
                continue
            came_from[nxt] = current
            g[nxt] = tent
            f[nxt] = tent + flat_dist(positions[nxt], positions[goal])
            if nxt not in open_set:
                open_set.add(nxt)
                open_list.append(nxt)

    return None, explored, 0, 0, 0


def count_subgraph(road_by_wp, forward, roads: set[str]):
    wps = {wp for wp, r in road_by_wp.items() if r in roads}
    edges = 0
    for u, ns in forward.items():
        if road_by_wp.get(u) not in roads:
            continue
        for v in ns:
            if road_by_wp.get(v) in roads:
                edges += 1
    return len(wps), edges


positions, road_by_wp, forward, synthetic = load_graph(CSV)

center_wps, center_edges = count_subgraph(road_by_wp, forward, CENTER_ROADS)
approach_wps, approach_edges = count_subgraph(road_by_wp, forward, APPROACH_ROADS)

print("=== Graphe tablier central (R181-R193) ===")
print(f"Waypoints (noeuds): {center_wps}")
print(f"Arêtes base forward (dans le sous-graphe): {center_edges}")

# count synthetic touching center
syn_center = sum(
    1
    for a, b in synthetic
    if road_by_wp.get(a) in CENTER_ROADS or road_by_wp.get(b) in CENTER_ROADS
)
print(f"Arêtes synthetic_turn touchant le centre: {syn_center}")

print("\n=== Comparaison bretelles (R176-R195) ===")
print(f"Waypoints: {approach_wps}, arêtes base: {approach_edges}")

# sample routes that should cross bridge center: downtown -> industrial via highway
# downtown ~(131, 123), industrial ~(-1740, -1163)
# find nearest wps
def nearest_wps(x, z, n=3):
    dists = []
    for wp, (wx, wz) in positions.items():
        d = flat_dist((x, z), (wx, wz))
        dists.append((d, wp))
    dists.sort()
    return [wp for _, wp in dists[:n]]


downtown = (131.11, 123.24)
industrial = (-1740.94, -1163.29)
r183_out = (-593.9, -393.9)  # bridge city-side approach

starts = nearest_wps(*downtown)
ends = nearest_wps(*industrial)

print("\n=== A* (sans filtre lane-flow, max 32768) ===")
for label, origin in (("downtown", downtown), ("r183_out", r183_out)):
    s_cands = nearest_wps(*origin)
    e_cands = nearest_wps(*industrial)
    best = None
    for s in s_cands[:2]:
        for e in e_cands[:2]:
            path, explored, plen, c_wps, c_edges = astar(forward, positions, s, e)
            if path and (best is None or explored < best[1]):
                best = (path, explored, plen, c_wps, c_edges, s, e)
    if best:
        path, explored, plen, c_wps, c_edges, s, e = best
        uses_center = c_wps > 0
        print(
            f"{label} -> industrial: OK pathLen={plen} explored={explored} "
            f"center_wps_on_path={c_wps} center_edges_on_path={c_edges} "
            f"uses_center={uses_center} ({s}->{e})"
        )
    else:
        print(f"{label} -> industrial: FAIL")

# BFS through center only from any center wp
any_center = next(wp for wp, r in road_by_wp.items() if r in CENTER_ROADS)
q = deque([any_center])
seen = {any_center}
while q and len(seen) < 50000:
    c = q.popleft()
    for n in forward.get(c, []):
        if road_by_wp.get(n) in CENTER_ROADS and n not in seen:
            seen.add(n)
            q.append(n)
print(f"\n=== Composante forward centre (depuis un wp): {len(seen)} waypoints atteignables")
