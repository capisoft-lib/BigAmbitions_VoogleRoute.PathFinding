import csv
import html
import math
import re
import sys
from collections import defaultdict
from pathlib import Path


WIDTH = 1800
HEIGHT = 1500
MARGIN = 70
INTERSECTION_RADIUS = 34.0
MULTILANE_INTERSECTION_RADIUS = 62.0
MIN_TURN_DEGREES = 28.0
MAX_TURN_DEGREES = 142.0
MIN_UTURN_DEGREES = 145.0
MAX_UTURN_DEGREES = 181.0
EXCLUDED_SYNTHETIC_ROAD_PAIRS = {
    ("183", "116"),
    ("184", "116"),
    ("130", "116"),
    ("132", "119"),
}
EXCLUDED_SYNTHETIC_WAYPOINT_PAIRS = {
    (3262, 4356),
    (3262, 1266),
    (5028, 12413),
    (5028, 11868),
    (4077, 7729),
    (4077, 1085),
    (4077, 7828),
    (10028, 13106),
    (4635, 6491),
    (4503, 3093),
    (4503, 529),
    (4503, 12492),
    (3378, 6491),
    (2269, 4791),
    (8621, 8131),
    (8621, 8475),
    (11002, 514),
    (10028, 2277),
    (4503, 7629),
    (2733, 1121),
    (13298, 1121),
    (12798, 6196),
    (9047, 1137),
    (12817, 5058),
    (2146, 5058),
    (6140, 6374),
    (12056, 837),
    (13115, 9401),
    (13115, 10350),
    (2453, 12649),
    (12056, 465),
    (1588, 8257),
    (9246, 5837),
    (9246, 1450),
    (5447, 1468),
    (4635, 11315),
    (4635, 12122),
    (4635, 1085),
    (4635, 7828),
    (10028, 13327),
    (12996, 1720),
    (12996, 3427),
    (11044, 13327),
    (12599, 562),
    (12599, 8257),
    (9246, 8790),
    (9421, 7477),
    (9421, 11645),
}
EXCLUDED_INTERNAL_UTURN_ROADS = {"116", "119"}
CORRIDOR_UTURN_ROAD_PAIRS = (
    ("10", "11"),
    ("13", "12"),
    ("14", "15"),
    ("34", "33"),
    ("47", "48"),
    ("49", "50"),
    ("51", "52"),
    ("60", "61"),
    ("62", "63"),
    ("64", "65"),
    ("92", "91"),
    ("93", "94"),
    ("95", "96"),
    ("110", "109"),
    ("111", "112"),
    ("116", "115"),
)
CORRIDOR_UTURN_ROADS = {road for pair in CORRIDOR_UTURN_ROAD_PAIRS for road in pair}
PARALLEL_UTURN_MAX_DIST = 55.0


def parse_waypoints(csv_path):
    points = {}
    neighbors = {}
    incoming = defaultdict(list)
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames and "edgeType" in reader.fieldnames:
            return parse_enhanced_route_edges(reader)

        for row in reader:
            idx = int(row["listIndex"])
            name = row["name"]
            ns = [int(part) for part in row.get("neighbors", "").split(";") if part.strip()]
            road_match = re.search(r"Road_(\d+)", name)
            lane_match = re.search(r"Lane_(\d+)", name)
            road = road_match.group(1) if road_match else "unknown"
            lane = int(lane_match.group(1)) if lane_match else -1
            point = {
                "idx": idx,
                "name": name,
                "x": float(row["posX"]),
                "y": float(row["posY"]),
                "z": float(row["posZ"]),
                "disabled": row["disabled"] not in ("0", "False", "false", ""),
                "road": road,
                "lane": lane,
            }
            points[idx] = point
            neighbors[idx] = ns

    for idx, ns in neighbors.items():
        for n in ns:
            incoming[n].append(idx)

    return points, neighbors, incoming


def parse_enhanced_route_edges(reader):
    points = {}
    neighbors = defaultdict(list)
    incoming = defaultdict(list)
    for row in reader:
        if row.get("edgeType") != "base":
            continue
        a = point_from_edge_row(row, "from")
        b = point_from_edge_row(row, "to")
        points[a["idx"]] = a
        points[b["idx"]] = b
        neighbors[a["idx"]].append(b["idx"])
        incoming[b["idx"]].append(a["idx"])
    return points, dict(neighbors), incoming


def point_from_edge_row(row, prefix):
    return {
        "idx": int(row[f"{prefix}Index"]),
        "name": row[f"{prefix}Name"],
        "x": float(row[f"{prefix}X"]),
        "y": float(row[f"{prefix}Y"]),
        "z": float(row[f"{prefix}Z"]),
        "disabled": False,
        "road": row[f"{prefix}Road"],
        "lane": int(row[f"{prefix}Lane"]),
    }


def is_connector(point):
    name = point["name"]
    return "Connector" in name or "CConnect" in name


def v_sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def v_len(v):
    return math.hypot(v[0], v[1])


def v_norm(v):
    length = v_len(v)
    if length < 0.001:
        return 0.0, 0.0
    return v[0] / length, v[1] / length


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def signed_angle(a, b):
    a = v_norm(a)
    b = v_norm(b)
    if v_len(a) < 0.001 or v_len(b) < 0.001:
        return 0.0
    cross = a[0] * b[1] - a[1] * b[0]
    d = max(-1.0, min(1.0, dot(a, b)))
    return math.degrees(math.atan2(cross, d))


def same_lane(a, b):
    return a["road"] == b["road"] and a["lane"] == b["lane"]


def build_lane_info(points, neighbors):
    lanes = defaultdict(list)
    lane_edges = defaultdict(list)
    for p in points.values():
        if p["disabled"] or is_connector(p) or p["lane"] < 0:
            continue
        lanes[(p["road"], p["lane"])].append(p)

    for idx, ns in neighbors.items():
        if idx not in points:
            continue
        a = points[idx]
        if a["disabled"] or is_connector(a):
            continue
        for n in ns:
            if n not in points:
                continue
            b = points[n]
            if b["disabled"] or is_connector(b):
                continue
            if same_lane(a, b):
                lane_edges[(a["road"], a["lane"])].append((a, b))

    info = {}
    for key, pts in lanes.items():
        sx = sum(p["x"] for p in pts) / len(pts)
        sz = sum(p["z"] for p in pts) / len(pts)
        dx = dz = 0.0
        for a, b in lane_edges.get(key, []):
            dx += b["x"] - a["x"]
            dz += b["z"] - a["z"]
        direction = v_norm((dx, dz))
        if v_len(direction) < 0.001 and len(pts) >= 2:
            pts_sorted = sorted(pts, key=lambda p: p["name"])
            direction = v_norm((pts_sorted[-1]["x"] - pts_sorted[0]["x"], pts_sorted[-1]["z"] - pts_sorted[0]["z"]))
        info[key] = {
            "road": key[0],
            "lane": key[1],
            "center": (sx, sz),
            "direction": direction,
            "count": len(pts),
        }
    return info


def mark_leftmost_lanes(lane_info):
    by_road = defaultdict(list)
    for info in lane_info.values():
        by_road[info["road"]].append(info)

    allowed = set()
    for road, lanes in by_road.items():
        if not lanes:
            continue
        used = [lane for lane in lanes if v_len(lane["direction"]) >= 0.001]
        if not used:
            for lane in lanes:
                allowed.add((road, lane["lane"]))
            continue

        ref = used[0]["direction"]
        clusters = [[], []]
        for lane in lanes:
            direction = lane["direction"]
            if v_len(direction) < 0.001:
                clusters[0].append(lane)
            elif dot(direction, ref) >= 0:
                clusters[0].append(lane)
            else:
                clusters[1].append(lane)

        for cluster in clusters:
            if not cluster:
                continue
            if len(cluster) == 1:
                allowed.add((road, cluster[0]["lane"]))
                continue
            avg_dir = v_norm((
                sum(l["direction"][0] for l in cluster),
                sum(l["direction"][1] for l in cluster),
            ))
            if v_len(avg_dir) < 0.001:
                avg_dir = cluster[0]["direction"]
            left = (-avg_dir[1], avg_dir[0])
            leftmost = max(cluster, key=lambda lane: dot(lane["center"], left))
            allowed.add((road, leftmost["lane"]))

    return allowed


def road_lane_counts(lane_info):
    counts = defaultdict(set)
    for road, lane in lane_info.keys():
        counts[road].add(lane)
    return {road: len(lanes) for road, lanes in counts.items()}


def road_profiles(points, lane_info):
    grouped = defaultdict(list)
    for point in points.values():
        if point["disabled"] or is_connector(point) or point["lane"] < 0:
            continue
        grouped[point["road"]].append(point)

    profiles = {}
    counts = road_lane_counts(lane_info)
    for road, road_points in grouped.items():
        xs = [p["x"] for p in road_points]
        zs = [p["z"] for p in road_points]
        span_x = max(xs) - min(xs)
        span_z = max(zs) - min(zs)
        span = max(span_x, span_z)
        width = min(span_x, span_z)
        profiles[road] = {
            "lane_count": counts.get(road, 0),
            "span": span,
            "width": width,
            "axis": "x" if span_x >= span_z else "z",
            "axis_2x4": counts.get(road, 0) == 4 and span >= 100.0 and width <= 36.0,
            "internal_axis_2x4": (
                counts.get(road, 0) == 4 and
                road not in EXCLUDED_INTERNAL_UTURN_ROADS and
                span >= 50.0 and
                width <= 36.0),
        }
    return profiles


def lane_direction(point, lane_info):
    return lane_info.get((point["road"], point["lane"]), {}).get("direction", (0.0, 0.0))


def endpoint_candidates(points, neighbors, incoming, lane_info):
    exits = []
    entries = []
    for idx, p in points.items():
        if p["disabled"] or is_connector(p) or p["lane"] < 0:
            continue
        out_diff = []
        for n in neighbors.get(idx, []):
            if n not in points or points[n]["disabled"]:
                continue
            q = points[n]
            if is_connector(q) or not same_lane(p, q):
                out_diff.append(q)
        in_diff = []
        for prev in incoming.get(idx, []):
            if prev not in points or points[prev]["disabled"]:
                continue
            q = points[prev]
            if is_connector(q) or not same_lane(q, p):
                in_diff.append(q)

        direction = lane_direction(p, lane_info)
        if out_diff or p["name"].endswith("-Out"):
            exits.append({**p, "dir": direction})
        if in_diff or p["name"].endswith("-In"):
            entries.append({**p, "dir": direction})
    return exits, entries


def line_intersection(a, ad, b, bd):
    denom = ad[0] * bd[1] - ad[1] * bd[0]
    if abs(denom) < 0.0001:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
    dx = b[0] - a[0]
    dz = b[1] - a[1]
    t = (dx * bd[1] - dz * bd[0]) / denom
    return a[0] + ad[0] * t, a[1] + ad[1] * t


def existing_reachable_turn(points, neighbors, start_idx, end_idx, max_depth=8):
    stack = [(start_idx, 0)]
    seen = {start_idx}
    while stack:
        idx, depth = stack.pop()
        if idx == end_idx:
            return True
        if depth >= max_depth:
            continue
        for n in neighbors.get(idx, []):
            if n in seen or n not in points:
                continue
            seen.add(n)
            stack.append((n, depth + 1))
    return False


def turn_search_radius(source, target, lane_counts):
    source_lanes = lane_counts.get(source["road"], 0)
    target_lanes = lane_counts.get(target["road"], 0)
    if source_lanes >= 4 or target_lanes >= 4:
        return MULTILANE_INTERSECTION_RADIUS
    return INTERSECTION_RADIUS


def make_synthetic_turns(points, neighbors, incoming, lane_info, allowed_leftmost, lane_counts, profiles):
    exits, entries = endpoint_candidates(points, neighbors, incoming, lane_info)
    turns = []
    seen = set()
    for e in exits:
        if (e["road"], e["lane"]) not in allowed_leftmost:
            continue
        in_dir = e["dir"]
        if v_len(in_dir) < 0.001:
            continue
        for target in entries:
            if target["idx"] == e["idx"]:
                continue
            if (e["road"], target["road"]) in EXCLUDED_SYNTHETIC_ROAD_PAIRS:
                continue
            if (e["idx"], target["idx"]) in EXCLUDED_SYNTHETIC_WAYPOINT_PAIRS:
                continue
            if e["road"] == target["road"] and e["lane"] == target["lane"]:
                continue
            dist = math.hypot(target["x"] - e["x"], target["z"] - e["z"])
            if dist > turn_search_radius(e, target, lane_counts):
                continue
            out_dir = target["dir"]
            if v_len(out_dir) < 0.001:
                continue
            angle = signed_angle(in_dir, out_dir)
            abs_angle = abs(angle)
            if abs_angle < MIN_TURN_DEGREES or abs_angle > MAX_TURN_DEGREES:
                continue
            # In the game's lane graph/projection, the useful "left turn" candidates
            # are the positive-angle branches. Negative-angle branches are not added.
            if angle <= 0:
                continue
            if existing_reachable_turn(points, neighbors, e["idx"], target["idx"]):
                continue
            key = (e["idx"], target["idx"])
            if key in seen:
                continue
            seen.add(key)
            control = line_intersection((e["x"], e["z"]), in_dir, (target["x"], target["z"]), out_dir)
            turns.append({
                "from": e,
                "to": target,
                "control": control,
                "angle": angle,
                "maneuver": "left",
                "source": "generated_leftmost_lane_rule_user_left_only",
            })
    add_parallel_corridor_uturns(points, neighbors, exits, entries, allowed_leftmost, turns, seen)
    return turns


def corridor_opposite_road(road):
    for road_a, road_b in CORRIDOR_UTURN_ROAD_PAIRS:
        if road == road_a:
            return road_b
        if road == road_b:
            return road_a
    return None


def best_corridor_uturn_target(source, entries, target_road, points, neighbors):
    in_dir = source["dir"]
    if v_len(in_dir) < 0.001:
        return None

    best = None
    best_score = float("inf")
    for target in entries:
        if target["road"] != target_road:
            continue

        out_dir = target["dir"]
        if v_len(out_dir) < 0.001 or dot(in_dir, out_dir) > -0.85:
            continue

        dist = math.hypot(target["x"] - source["x"], target["z"] - source["z"])
        if dist > PARALLEL_UTURN_MAX_DIST:
            continue

        angle = signed_angle(in_dir, out_dir)
        if abs(abs(angle) - 180.0) > 12.0:
            continue
        if existing_reachable_turn(points, neighbors, source["idx"], target["idx"]):
            continue

        score = dist + abs(source["lane"] - target["lane"]) * 2.0 + abs(abs(angle) - 180.0)
        if score < best_score:
            best_score = score
            best = (target, angle, dist)
    return best


def corridor_exit_station(exit_point):
    return round(exit_point["x"] / 10.0) * 10


def pick_corridor_uturn_exits(exits, entries, allowed_leftmost, points, neighbors):
  # One U-turn source per road/intersection: leftmost Out when present, else best Out.
    by_station = defaultdict(list)
    for e in exits:
        if e["road"] not in CORRIDOR_UTURN_ROADS or "Out" not in e["name"]:
            continue
        by_station[(e["road"], corridor_exit_station(e))].append(e)

    picked = []
    for (_road, _station), group in by_station.items():
        opposite_road = corridor_opposite_road(group[0]["road"])
        if opposite_road is None:
            continue

        leftmost = [e for e in group if (e["road"], e["lane"]) in allowed_leftmost]
        candidates = leftmost if leftmost else group

        best_exit = None
        best_score = float("inf")
        for e in candidates:
            match = best_corridor_uturn_target(e, entries, opposite_road, points, neighbors)
            if match is None:
                continue
            _target, _angle, dist = match
            score = dist + (0.0 if e in leftmost else 4.0)
            if score < best_score:
                best_score = score
                best_exit = e
        if best_exit is not None:
            picked.append(best_exit)
    return picked


def add_parallel_corridor_uturns(points, neighbors, exits, entries, allowed_leftmost, turns, seen):
    # Bidirectional ~180° links between paired parallel carriageways (9 intersections).
    for e in pick_corridor_uturn_exits(exits, entries, allowed_leftmost, points, neighbors):
        opposite_road = corridor_opposite_road(e["road"])
        match = best_corridor_uturn_target(e, entries, opposite_road, points, neighbors)
        if match is None:
            continue

        target, angle, dist = match
        key = (e["idx"], target["idx"])
        if key in seen:
            continue
        seen.add(key)
        control = uturn_control_point(e, target, e["dir"], dist)
        turns.append({
            "from": e,
            "to": target,
            "control": control,
            "angle": angle,
            "maneuver": "uturn",
            "source": "generated_parallel_corridor_uturn",
        })


def add_synthetic_uturns(points, neighbors, exits, entries, allowed_leftmost, profiles, turns, seen):
    for e in exits:
        if not profiles.get(e["road"], {}).get("axis_2x4", False):
            continue
        if (e["road"], e["lane"]) not in allowed_leftmost:
            continue

        in_dir = e["dir"]
        if v_len(in_dir) < 0.001:
            continue

        best = None
        best_score = float("inf")
        for target in entries:
            if target["idx"] == e["idx"]:
                continue
            if target["road"] != e["road"]:
                continue
            if (target["road"], target["lane"]) not in allowed_leftmost:
                continue

            out_dir = target["dir"]
            if v_len(out_dir) < 0.001 or dot(in_dir, out_dir) > -0.65:
                continue

            dist = math.hypot(target["x"] - e["x"], target["z"] - e["z"])
            if dist > MULTILANE_INTERSECTION_RADIUS:
                continue

            angle = signed_angle(in_dir, out_dir)
            abs_angle = abs(angle)
            if abs_angle < MIN_UTURN_DEGREES or abs_angle > MAX_UTURN_DEGREES:
                continue
            if existing_reachable_turn(points, neighbors, e["idx"], target["idx"]):
                continue

            score = dist + abs(180.0 - abs_angle) * 0.8
            if score < best_score:
                best_score = score
                best = (target, angle, dist)

        if best is None:
            continue

        target, angle, dist = best
        key = (e["idx"], target["idx"])
        if key in seen:
            continue

        seen.add(key)
        control = uturn_control_point(e, target, in_dir, dist)
        turns.append({
            "from": e,
            "to": target,
            "control": control,
            "angle": angle,
            "maneuver": "uturn",
            "source": "generated_axis_2x4_uturn",
        })


def uturn_control_point(source, target, in_dir, dist):
    mid = ((source["x"] + target["x"]) * 0.5, (source["z"] + target["z"]) * 0.5)
    lead = max(9.0, min(16.0, dist * 2.0))
    return mid[0] + in_dir[0] * lead, mid[1] + in_dir[1] * lead


def add_internal_axis_uturns(points, lane_info, allowed_leftmost, profiles, turns, seen):
    connectors_by_road = defaultdict(list)
    points_by_road = defaultdict(list)
    for point in points.values():
        if point["disabled"] or point["lane"] < 0:
            continue
        profile = profiles.get(point["road"])
        if not profile or not profile.get("internal_axis_2x4", False):
            continue
        points_by_road[point["road"]].append(point)
        if "Connect" in point["name"]:
            connectors_by_road[point["road"]].append(point)

    for road, connector_points in connectors_by_road.items():
        profile = profiles[road]
        axis = profile["axis"]
        groups = group_by_station(connector_points, axis)
        for group in groups:
            station = sum(p["x" if axis == "x" else "z"] for p in group) / len(group)
            candidates = nearest_allowed_lane_points(
                points_by_road[road], station, axis, allowed_leftmost)
            add_internal_uturns_for_group(candidates, lane_info, turns, seen)


def nearest_allowed_lane_points(points, station, axis, allowed_leftmost):
    coord = "x" if axis == "x" else "z"
    best_by_lane = {}
    for point in points:
        if (point["road"], point["lane"]) not in allowed_leftmost:
            continue
        delta = abs(point[coord] - station)
        if delta > 14.0:
            continue
        current = best_by_lane.get(point["lane"])
        if current is None or delta < current[0]:
            best_by_lane[point["lane"]] = (delta, point)

    return [entry[1] for entry in best_by_lane.values()]


def group_by_station(points, axis):
    coord = "x" if axis == "x" else "z"
    points = sorted(points, key=lambda p: p[coord])
    groups = []
    for point in points:
        station = point[coord]
        if not groups or abs(groups[-1]["station"] - station) > 9.0:
            groups.append({"station": station, "points": [point]})
            continue
        group = groups[-1]
        group["points"].append(point)
        group["station"] = sum(p[coord] for p in group["points"]) / len(group["points"])
    return [group["points"] for group in groups if len(group["points"]) >= 2]


def add_internal_uturns_for_group(group, lane_info, turns, seen):
    candidates = []
    for point in group:
        direction = lane_direction(point, lane_info)
        if v_len(direction) < 0.001:
            continue
        candidates.append({**point, "dir": direction})

    for source in candidates:
        best = None
        best_score = float("inf")
        for target in candidates:
            if source["idx"] == target["idx"] or source["lane"] == target["lane"]:
                continue
            if dot(source["dir"], target["dir"]) > -0.65:
                continue
            dist = math.hypot(target["x"] - source["x"], target["z"] - source["z"])
            if dist < 3.0 or dist > 34.0:
                continue
            score = dist
            if score < best_score:
                best_score = score
                best = target

        if best is None:
            continue

        key = (source["idx"], best["idx"])
        if key in seen:
            continue
        seen.add(key)
        control = uturn_control_point(source, best, source["dir"], best_score)
        turns.append({
            "from": source,
            "to": best,
            "control": control,
            "angle": signed_angle(source["dir"], best["dir"]),
            "maneuver": "uturn",
            "source": "generated_axis_2x4_internal_uturn",
        })


def write_enhanced_csv(output_csv, points, neighbors, lane_info, allowed_leftmost, turns):
    fields = [
        "edgeId", "edgeType", "maneuver", "fromIndex", "fromName", "fromRoad", "fromLane",
        "fromX", "fromY", "fromZ", "toIndex", "toName", "toRoad", "toLane", "toX", "toY", "toZ",
        "controlX", "controlY", "controlZ", "angleDegrees", "fromLaneIsLeftmostTurnLane", "source",
    ]
    edge_id = 0
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for idx, ns in neighbors.items():
            if idx not in points:
                continue
            a = points[idx]
            for n in ns:
                if n not in points:
                    continue
                b = points[n]
                writer.writerow(edge_row(edge_id, "base", "base", a, b, None, 0.0, (a["road"], a["lane"]) in allowed_leftmost, "gley"))
                edge_id += 1
        for turn in turns:
            a = turn["from"]
            b = turn["to"]
            cy = (a["y"] + b["y"]) * 0.5
            writer.writerow(edge_row(
                edge_id,
                "synthetic_turn",
                turn["maneuver"],
                a,
                b,
                (turn["control"][0], cy, turn["control"][1]),
                turn["angle"],
                True,
                turn.get("source", "generated_leftmost_lane_rule_user_left_only"),
            ))
            edge_id += 1


def edge_row(edge_id, edge_type, maneuver, a, b, control, angle, leftmost, source):
    if control is None:
        control = ("", "", "")
    return {
        "edgeId": edge_id,
        "edgeType": edge_type,
        "maneuver": maneuver,
        "fromIndex": a["idx"],
        "fromName": a["name"],
        "fromRoad": a["road"],
        "fromLane": a["lane"],
        "fromX": f'{a["x"]:.3f}',
        "fromY": f'{a["y"]:.3f}',
        "fromZ": f'{a["z"]:.3f}',
        "toIndex": b["idx"],
        "toName": b["name"],
        "toRoad": b["road"],
        "toLane": b["lane"],
        "toX": f'{b["x"]:.3f}',
        "toY": f'{b["y"]:.3f}',
        "toZ": f'{b["z"]:.3f}',
        "controlX": "" if control[0] == "" else f"{control[0]:.3f}",
        "controlY": "" if control[1] == "" else f"{control[1]:.3f}",
        "controlZ": "" if control[2] == "" else f"{control[2]:.3f}",
        "angleDegrees": f"{angle:.2f}",
        "fromLaneIsLeftmostTurnLane": 1 if leftmost else 0,
        "source": source,
    }


def transform_factory(points):
    xs = [p["x"] for p in points.values()]
    zs = [p["z"] for p in points.values()]
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    scale = min((WIDTH - MARGIN * 2) / (max_x - min_x), (HEIGHT - MARGIN * 2) / (max_z - min_z))
    used_w = (max_x - min_x) * scale
    used_h = (max_z - min_z) * scale
    off_x = (WIDTH - used_w) / 2
    off_y = (HEIGHT - used_h) / 2

    def t(x, z):
        return off_x + (x - min_x) * scale, off_y + (max_z - z) * scale
    return t, (min_x, max_x, min_z, max_z)


def svg_path_for_base(points, neighbors, transform):
    chunks = []
    for idx, ns in neighbors.items():
        if idx not in points:
            continue
        a = points[idx]
        for n in ns:
            if n not in points:
                continue
            b = points[n]
            ax, ay = transform(a["x"], a["z"])
            bx, by = transform(b["x"], b["z"])
            chunks.append(f"M{ax:.1f},{ay:.1f} L{bx:.1f},{by:.1f}")
    return " ".join(chunks)


def road_path_id(road):
    return f"road-{road}"


def svg_road_path_elements(points, neighbors, transform, lane_counts):
    by_road = defaultdict(list)
    for idx, ns in neighbors.items():
        if idx not in points:
            continue
        a = points[idx]
        for n in ns:
            if n not in points:
                continue
            b = points[n]
            ax, ay = transform(a["x"], a["z"])
            bx, by = transform(b["x"], b["z"])
            by_road[a["road"]].append(f"M{ax:.1f},{ay:.1f} L{bx:.1f},{by:.1f}")

    def road_sort_key(road):
        return (0, int(road)) if str(road).isdigit() else (1, str(road))

    lines = []
    for road in sorted(by_road.keys(), key=road_sort_key):
        lanes = lane_counts.get(road, 0)
        path_id = road_path_id(road)
        label = f"{path_id} | Road {road} | {lanes} voie(s)"
        lines.append(
            f'    <path class="road" id="{path_id}" '
            f'data-road="{html.escape(str(road))}" '
            f'data-label="{html.escape(label)}" '
            f'd="{" ".join(by_road[road])}"/>'
        )
    return "\n".join(lines)


def turn_path_id(turn):
    return f"turn-{turn['from']['idx']}-{turn['to']['idx']}"


def svg_turn_path_element(turn, transform):
    a = turn["from"]
    b = turn["to"]
    cx, cz = turn["control"]
    ax, ay = transform(a["x"], a["z"])
    bx, by = transform(b["x"], b["z"])
    sx, sy = transform(cx, cz)
    d = f"M{ax:.1f},{ay:.1f} Q{sx:.1f},{sy:.1f} {bx:.1f},{by:.1f}"
    path_id = turn_path_id(turn)
    css_class = {
        "left": "left-turn turn",
        "uturn": "uturn turn",
        "right": "right-turn turn",
    }.get(turn["maneuver"], "turn")
    title = (
        f"{path_id} | Road {a['road']} -> Road {b['road']} | "
        f"{a['idx']} -> {b['idx']}"
    )
    return (
        f'  <path id="{path_id}" class="{css_class}" '
        f'd="{d}" '
        f'data-from="{a["idx"]}" data-to="{b["idx"]}" '
        f'data-from-road="{a["road"]}" data-to-road="{b["road"]}">\n'
        f'    <title>{html.escape(title)}</title>\n'
        f'  </path>'
    )


def svg_turn_path_elements(turns, transform):
    # One SVG path per synthetic turn so each curve has a stable id for exclusion lists.
    return "\n".join(svg_turn_path_element(turn, transform) for turn in turns)


def write_svg(output_svg, points, neighbors, turns, lane_counts):
    transform, bounds = transform_factory(points)
    base_d = svg_path_for_base(points, neighbors, transform)
    turn_paths = svg_turn_path_elements(turns, transform)
    left_count = sum(1 for turn in turns if turn["maneuver"] == "left")
    uturn_count = sum(1 for turn in turns if turn["maneuver"] == "uturn")
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">
  <title>Big Ambitions - Enhanced Route Graph</title>
  <desc>Base Gley graph plus generated synthetic left turns from leftmost lanes only. Each green curve has id turn-FROM-TO.</desc>
  <defs>
    <style>
      .bg {{ fill: #0e151b; }}
      .base {{ fill: none; stroke: #8ea2ad; stroke-width: 1.2; stroke-linecap: round; opacity: 0.55; }}
      .left-turn {{ fill: none; stroke: #36d982; stroke-width: 2.6; stroke-linecap: round; stroke-linejoin: round; opacity: 0.9; }}
      .uturn {{ fill: none; stroke: #ff9f43; stroke-width: 2.9; stroke-linecap: round; stroke-linejoin: round; opacity: 0.95; }}
      .right-turn {{ fill: none; stroke: #36d982; stroke-width: 2.3; stroke-linecap: round; stroke-linejoin: round; opacity: 0.0; }}
      .turn:hover {{ stroke: #ff6b6b; stroke-width: 4; opacity: 1; }}
      .label {{ fill: #e5eef3; font: 18px Arial, sans-serif; }}
      .small {{ fill: #aabdc8; font: 13px Arial, sans-serif; }}
      .legend {{ fill: rgba(255,255,255,0.07); stroke: #526a78; stroke-width: 1; }}
    </style>
  </defs>
  <rect class="bg" x="0" y="0" width="{WIDTH}" height="{HEIGHT}"/>
  <path class="base" d="{base_d}"/>
  <g id="synthetic-turns">
{turn_paths}
  </g>
  <g>
    <rect class="legend" x="40" y="40" width="760" height="145" rx="10"/>
    <text class="label" x="60" y="74">Big Ambitions - graphe routier enrichi</text>
    <text class="small" x="60" y="102">Verts = virages gauche | Orange = demi-tour | id turn-FROM-TO au survol.</text>
    <text class="small" x="60" y="126">Synthetic turns: {len(turns)} (left {left_count}, uturn {uturn_count}) | Bounds X {bounds[0]:.1f}..{bounds[1]:.1f}, Z {bounds[2]:.1f}..{bounds[3]:.1f}</text>
    <path class="base" d="M62 150 H125"/><text class="small" x="140" y="154">edges Gley originaux</text>
    <path class="left-turn" d="M320 150 Q345 126 370 150"/><text class="small" x="385" y="154">virage gauche (id sur chaque courbe)</text>
  </g>
</svg>
'''
    Path(output_svg).write_text(svg, encoding="utf-8")
    write_picker_html(
        output_svg.with_name(output_svg.stem + "_picker.html"),
        turns,
        points,
        neighbors,
        lane_counts,
    )


def write_picker_html(output_html, turns, points, neighbors, lane_counts):
    transform, bounds = transform_factory(points)
    road_lines = svg_road_path_elements(points, neighbors, transform, lane_counts)
    turn_lines = []
    for turn in turns:
        a = turn["from"]
        b = turn["to"]
        cx, cz = turn["control"]
        ax, ay = transform(a["x"], a["z"])
        bx, by = transform(b["x"], b["z"])
        sx, sy = transform(cx, cz)
        path_id = turn_path_id(turn)
        css_class = "turn uturn" if turn["maneuver"] == "uturn" else "turn left"
        turn_lines.append(
            f'    <path class="{css_class}" id="{path_id}" '
            f'data-label="{html.escape(path_id)} | Road {a["road"]} -&gt; Road {b["road"]} | {a["idx"]} -&gt; {b["idx"]}" '
            f'd="M{ax:.1f},{ay:.1f} Q{sx:.1f},{sy:.1f} {bx:.1f},{by:.1f}"/>'
        )
    turns_markup = "\n".join(turn_lines)
    page = f'''<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="utf-8"/>
  <title>Enhanced map picker</title>
  <style>
    body {{ margin: 0; font: 14px/1.4 Arial, sans-serif; background: #0e151b; color: #e5eef3; }}
    #bar {{ padding: 10px 14px; background: #152029; border-bottom: 1px solid #526a78; display: flex; flex-wrap: wrap; gap: 8px 14px; align-items: center; }}
    #bar code {{ background: #24333d; padding: 2px 6px; border-radius: 4px; }}
    #bar .hint {{ color: #aabdc8; font-size: 12px; }}
    #bar button {{ background: #2d95ff; color: #fff; border: 0; border-radius: 4px; padding: 5px 10px; cursor: pointer; }}
    #bar button.secondary {{ background: #3a4f5c; }}
    #bar button.active {{ background: #1f7a45; }}
    #stage {{ width: 100%; height: calc(100vh - 96px); overflow: hidden; cursor: grab; }}
    #stage.panning {{ cursor: grabbing; }}
    #map {{ width: 100%; height: 100%; display: block; user-select: none; }}
    .bg {{ fill: #0e151b; }}
    .road {{ fill: none; stroke: #8ea2ad; stroke-width: 2.4; stroke-linecap: round; opacity: 0.55; }}
    .turn.left {{ fill: none; stroke: #36d982; stroke-width: 2.6; stroke-linecap: round; cursor: pointer; pointer-events: stroke; }}
    .turn.uturn {{ fill: none; stroke: #ff9f43; stroke-width: 2.8; stroke-linecap: round; cursor: pointer; pointer-events: stroke; }}
    .turn:hover, .turn.selected {{ stroke: #ff6b6b; stroke-width: 5; }}
    .mode-turns .road {{ pointer-events: none; }}
    .mode-roads .turn {{ pointer-events: none; }}
    .mode-roads .road {{ pointer-events: stroke; cursor: pointer; opacity: 0.72; }}
    .mode-roads .road:hover {{ stroke: #ffb454; stroke-width: 4.5; opacity: 1; }}
    .mode-roads .road.selected {{ stroke: #ff6b6b; stroke-width: 6; opacity: 1; }}
  </style>
</head>
<body>
  <div id="bar">
    <button type="button" id="mode-turns" class="active">Mode virages</button>
    <button type="button" id="mode-roads" class="secondary">Mode routes</button>
    <span>Selection: <code id="selection">(aucun)</code></span>
    <button type="button" id="copy">Copier</button>
    <button type="button" id="clear" class="secondary">Effacer</button>
    <button type="button" id="zoom-in" class="secondary">Zoom +</button>
    <button type="button" id="zoom-out" class="secondary">Zoom -</button>
    <button type="button" id="reset" class="secondary">Reset vue</button>
    <span class="hint" id="hint">Virages: clic trait vert | Routes: clic route grise (Ctrl = multi) | Glisser fond = deplacer</span>
  </div>
  <div id="stage" class="mode-turns">
    <svg id="map" viewBox="0 0 {WIDTH} {HEIGHT}" xmlns="http://www.w3.org/2000/svg">
      <rect class="bg" width="{WIDTH}" height="{HEIGHT}"/>
      <g id="content">
        <g id="roads">
{road_lines}
        </g>
        <g id="turns">
{turns_markup}
        </g>
      </g>
    </svg>
  </div>
  <script>
    const map = document.getElementById("map");
    const stage = document.getElementById("stage");
    const label = document.getElementById("selection");
    const copyBtn = document.getElementById("copy");
    const clearBtn = document.getElementById("clear");
    const modeTurnsBtn = document.getElementById("mode-turns");
    const modeRoadsBtn = document.getElementById("mode-roads");
    const hint = document.getElementById("hint");
    const VB_W = {WIDTH};
    const VB_H = {HEIGHT};
    const MIN_VB_W = 80;
    let vb = {{ x: 0, y: 0, w: VB_W, h: VB_H }};
    let mode = "turns";
    let selectedTurn = null;
    let selectedRoads = new Set();
    let panning = false;
    let panStart = null;
    let dragMoved = false;

    function applyViewBox() {{
      map.setAttribute("viewBox", `${{vb.x}} ${{vb.y}} ${{vb.w}} ${{vb.h}}`);
    }}

    function clientToSvg(clientX, clientY) {{
      const rect = map.getBoundingClientRect();
      const nx = (clientX - rect.left) / rect.width;
      const ny = (clientY - rect.top) / rect.height;
      return {{ x: vb.x + nx * vb.w, y: vb.y + ny * vb.h }};
    }}

    function zoomAt(clientX, clientY, factor) {{
      const p = clientToSvg(clientX, clientY);
      const nextW = Math.max(MIN_VB_W, Math.min(VB_W, vb.w * factor));
      const nextH = nextW * (VB_H / VB_W);
      const rx = (p.x - vb.x) / vb.w;
      const ry = (p.y - vb.y) / vb.h;
      vb.w = nextW;
      vb.h = nextH;
      vb.x = p.x - rx * vb.w;
      vb.y = p.y - ry * vb.h;
      clampViewBox();
      applyViewBox();
    }}

    function clampViewBox() {{
      if (vb.w >= VB_W) {{
        vb.w = VB_W;
        vb.h = VB_H;
        vb.x = 0;
        vb.y = 0;
        return;
      }}
      vb.x = Math.max(0, Math.min(VB_W - vb.w, vb.x));
      vb.y = Math.max(0, Math.min(VB_H - vb.h, vb.y));
    }}

    function resetView() {{
      vb = {{ x: 0, y: 0, w: VB_W, h: VB_H }};
      applyViewBox();
    }}

    function updateSelectionLabel() {{
      if (mode === "turns") {{
        label.textContent = selectedTurn
          ? (selectedTurn.dataset.label || selectedTurn.id)
          : "(aucun virage)";
        return;
      }}
      if (selectedRoads.size === 0) {{
        label.textContent = "(aucune route)";
        return;
      }}
      const ids = Array.from(selectedRoads).sort((a, b) => {{
        const na = parseInt(a.replace("road-", ""), 10);
        const nb = parseInt(b.replace("road-", ""), 10);
        if (!Number.isNaN(na) && !Number.isNaN(nb)) return na - nb;
        return a.localeCompare(b);
      }});
      label.textContent = ids.join(", ");
    }}

    function copyPayload() {{
      if (mode === "turns") return selectedTurn ? selectedTurn.id : "";
      return Array.from(selectedRoads).sort().join("\\n");
    }}

    function setMode(nextMode) {{
      mode = nextMode;
      stage.classList.toggle("mode-turns", mode === "turns");
      stage.classList.toggle("mode-roads", mode === "roads");
      modeTurnsBtn.classList.toggle("active", mode === "turns");
      modeRoadsBtn.classList.toggle("active", mode === "roads");
      modeTurnsBtn.classList.toggle("secondary", mode !== "turns");
      modeRoadsBtn.classList.toggle("secondary", mode !== "roads");
      hint.textContent = mode === "turns"
        ? "Virages: clic trait vert | Glisser fond = deplacer"
        : "Routes: clic route grise | Ctrl = multi | Envoyer la liste pour U-turns cibles";
      updateSelectionLabel();
    }}

    function selectTurn(el, additive) {{
      if (!additive) {{
        document.querySelectorAll(".turn.selected").forEach((n) => n.classList.remove("selected"));
        selectedTurn = el;
        el.classList.add("selected");
      }} else {{
        selectedTurn = el;
        document.querySelectorAll(".turn.selected").forEach((n) => n.classList.remove("selected"));
        el.classList.add("selected");
      }}
      updateSelectionLabel();
    }}

    function toggleRoad(el) {{
      const id = el.id;
      if (selectedRoads.has(id)) {{
        selectedRoads.delete(id);
        el.classList.remove("selected");
      }} else {{
        selectedRoads.add(id);
        el.classList.add("selected");
      }}
      updateSelectionLabel();
    }}

    function clearSelection() {{
      selectedTurn = null;
      selectedRoads.clear();
      document.querySelectorAll(".turn.selected, .road.selected").forEach((n) => n.classList.remove("selected"));
      updateSelectionLabel();
    }}

    document.querySelectorAll(".turn").forEach((el) => {{
      el.addEventListener("click", (event) => {{
        if (dragMoved) return;
        event.stopPropagation();
        selectTurn(el, event.ctrlKey || event.metaKey);
      }});
    }});

    document.querySelectorAll(".road").forEach((el) => {{
      el.addEventListener("click", (event) => {{
        if (dragMoved || mode !== "roads") return;
        event.stopPropagation();
        if (event.ctrlKey || event.metaKey) {{
          toggleRoad(el);
          return;
        }}
        clearSelection();
        selectedRoads.add(el.id);
        el.classList.add("selected");
        updateSelectionLabel();
      }});
    }});

    modeTurnsBtn.addEventListener("click", () => setMode("turns"));
    modeRoadsBtn.addEventListener("click", () => setMode("roads"));
    clearBtn.addEventListener("click", clearSelection);

    map.addEventListener("wheel", (event) => {{
      event.preventDefault();
      const factor = event.deltaY < 0 ? 0.82 : 1.22;
      zoomAt(event.clientX, event.clientY, factor);
    }}, {{ passive: false }});

    map.addEventListener("pointerdown", (event) => {{
      if (event.target.closest(".turn") || event.target.closest(".road")) return;
      panning = true;
      dragMoved = false;
      panStart = {{ x: event.clientX, y: event.clientY, vbX: vb.x, vbY: vb.y }};
      stage.classList.add("panning");
      map.setPointerCapture(event.pointerId);
    }});

    map.addEventListener("pointermove", (event) => {{
      if (!panning || !panStart) return;
      const rect = map.getBoundingClientRect();
      const dx = event.clientX - panStart.x;
      const dy = event.clientY - panStart.y;
      if (Math.abs(dx) > 3 || Math.abs(dy) > 3) dragMoved = true;
      vb.x = panStart.vbX - (dx / rect.width) * vb.w;
      vb.y = panStart.vbY - (dy / rect.height) * vb.h;
      clampViewBox();
      applyViewBox();
    }});

    function endPan(event) {{
      if (!panning) return;
      panning = false;
      panStart = null;
      stage.classList.remove("panning");
      if (map.hasPointerCapture(event.pointerId)) map.releasePointerCapture(event.pointerId);
      setTimeout(() => {{ dragMoved = false; }}, 0);
    }}
    map.addEventListener("pointerup", endPan);
    map.addEventListener("pointercancel", endPan);

    document.getElementById("zoom-in").addEventListener("click", () => {{
      const rect = map.getBoundingClientRect();
      zoomAt(rect.left + rect.width / 2, rect.top + rect.height / 2, 0.75);
    }});
    document.getElementById("zoom-out").addEventListener("click", () => {{
      const rect = map.getBoundingClientRect();
      zoomAt(rect.left + rect.width / 2, rect.top + rect.height / 2, 1.33);
    }});
    document.getElementById("reset").addEventListener("click", resetView);

    copyBtn.addEventListener("click", async () => {{
      const payload = copyPayload();
      if (!payload) return;
      await navigator.clipboard.writeText(payload);
      copyBtn.textContent = "Copie!";
      setTimeout(() => {{ copyBtn.textContent = "Copier"; }}, 1200);
    }});

    setMode("turns");
    applyViewBox();
  </script>
</body>
</html>
'''
    Path(output_html).write_text(page, encoding="utf-8")


def main():
    if len(sys.argv) != 4:
        print("Usage: generate_enhanced_route_graph.py <waypoints.csv> <enhanced.csv> <enhanced.svg>", file=sys.stderr)
        return 2
    source_csv = Path(sys.argv[1])
    output_csv = Path(sys.argv[2])
    output_svg = Path(sys.argv[3])
    points, neighbors, incoming = parse_waypoints(source_csv)
    lane_info = build_lane_info(points, neighbors)
    allowed_leftmost = mark_leftmost_lanes(lane_info)
    lane_counts = road_lane_counts(lane_info)
    profiles = road_profiles(points, lane_info)
    turns = make_synthetic_turns(points, neighbors, incoming, lane_info, allowed_leftmost, lane_counts, profiles)
    write_enhanced_csv(output_csv, points, neighbors, lane_info, allowed_leftmost, turns)
    write_svg(output_svg, points, neighbors, turns, lane_counts)
    print(f"Wrote {output_csv} and {output_svg} ({len(turns)} synthetic turns)")


if __name__ == "__main__":
    raise SystemExit(main())
