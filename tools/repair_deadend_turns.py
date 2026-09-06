"""Audited road-end repairs; never connect a street to a nearby bridge by proximity.

Run after bridge preprocessing. Only the six reviewed identities below can change.
Inbound tails are trimmed at the turn so nearest-node routing cannot select a sink
past it. Coordinates are verified before any write; a new game dump needs review.
"""
import argparse
import csv
import math
from pathlib import Path

import generate_enhanced_route_graph as graph

SOURCE = "audited_deadend_uturn"
# terminal, return entry, turn start, turn end, road, terminal X/Z
REPAIRS = (
    (10294, 1860, 10345, 14484, "237", -2538.379, -982.847),
    (15685, 305, 5888, 14398, "222", -2804.638, -1035.410),
    (6289, 13701, 9644, 6935, "230", -2877.386, -1047.679),
    (8315, 13821, 17491, 3923, "210", -2390.312, -1577.938),
    (9069, 9767, 8144, 2495, "233", -2598.166, -986.245),
    (7292, 2488, 8963, 8311, "246", -3273.312, -1597.040),
)
# This native hairpin uses Waypoint names, so the runtime Out -> In rule does
# not authorize it. Replace it with the explicit upstream U-turn as well.
REPLACED_NATIVE_TURNS = {7292: (6656, 12813)}


def load(path):
    with open(path, newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def point(row, prefix):
    return graph.point_from_edge_row(row, prefix)


def xy(p):
    return p["x"], p["z"]


def plan(rows):
    points, forward, incoming = graph.parse_enhanced_route_edges(iter(rows))
    changes = []
    for terminal, entry, start, end, road, x, z in REPAIRS:
        existing = [r for r in rows if r["source"] == SOURCE
                    and int(r["fromIndex"]) == start and int(r["toIndex"]) == end]
        if existing:
            if len(existing) != 1 or terminal in points:
                raise ValueError(f"Partial/duplicate repair at {terminal}")
            continue
        tip, a, b = (points[k] for k in (terminal, start, end))
        if (tip["road"] != road or a["road"] != road or b["road"] != road
                or abs(tip["x"] - x) > .01 or abs(tip["z"] - z) > .01
                or not tip["name"].endswith("-Out")
                or not points[entry]["name"].endswith("-In")):
            raise ValueError(f"Waypoint identities changed at {terminal}; re-audit the dump")
        if any(int(r["fromIndex"]) == terminal for r in rows):
            raise ValueError(f"Terminal {terminal} already has an exit; re-audit")
        tail = []
        cursor = terminal
        while cursor != start:
            if cursor in tail or not graph.same_lane(tip, points[cursor]):
                raise ValueError(f"Invalid inbound tail at {terminal}")
            tail.append(cursor)
            previous = incoming.get(cursor, [])
            if len(previous) != 1:
                raise ValueError(f"Branched inbound tail at {terminal}")
            cursor = previous[0]
        removed = [r for r in rows if int(r["fromIndex"]) in tail or int(r["toIndex"]) in tail]
        expected_pairs = set(zip([start] + list(reversed(tail))[:-1], reversed(tail)))
        if terminal in REPLACED_NATIVE_TURNS:
            expected_pairs.add(REPLACED_NATIVE_TURNS[terminal])
        actual_pairs = {(int(r["fromIndex"]), int(r["toIndex"])) for r in removed}
        if (actual_pairs != expected_pairs or len(removed) != len(expected_pairs)
                or any(r["edgeType"] != "base" for r in removed)):
            raise ValueError(f"Tail {terminal} has additional connections; re-audit")
        # Confirm the chosen return point belongs to the existing outgoing lane.
        cursor, visited = entry, set()
        while cursor != end:
            if cursor in visited or not graph.same_lane(b, points[cursor]):
                raise ValueError(f"Invalid return lane at {terminal}")
            visited.add(cursor)
            following = forward.get(cursor, [])
            if len(following) != 1:
                raise ValueError(f"Branched return lane at {terminal}")
            cursor = following[0]
        previous = points[incoming[start][0]]
        following = points[forward[end][0]]
        heading = graph.v_norm(graph.v_sub(xy(a), xy(previous)))
        outgoing = graph.v_norm(graph.v_sub(xy(following), xy(b)))
        angle = graph.signed_angle(heading, outgoing)
        width = math.dist(xy(a), xy(b))
        if abs(angle) < 145 or not 1.5 <= width <= 18 or abs(a["y"] - b["y"]) > 1.5:
            raise ValueError(f"Unsafe turn geometry at {terminal}")
        # Quadratic Bezier convex hull stays behind the terminal plane, with a
        # two-metre margin. Do not use the generic 9-16m forward control lead.
        normal = graph.v_norm(graph.v_sub(xy(tip), xy(a)))
        control = tuple((u + v) / 2 + h * min(6, width * .8)
                        for u, v, h in zip(xy(a), xy(b), heading))
        projection = graph.dot(graph.v_sub(control, xy(tip)), normal)
        if projection > -2:
            control = tuple(c - n * (projection + 2) for c, n in zip(control, normal))
        if any(graph.dot(graph.v_sub(p, xy(tip)), normal) > -2 + 1e-6
               for p in (xy(a), control, xy(b))):
            raise ValueError(f"Turn extends beyond terminal plane at {terminal}")
        row = graph.edge_row(0, "synthetic_turn", "uturn", a, b,
                             (control[0], (a["y"] + b["y"]) / 2, control[1]),
                             angle, True, SOURCE)
        changes.append(dict(terminal=terminal, entry=entry, start=start, end=end,
                            road=road, tail=tail, removed=removed, row=row,
                            stop=(x, z), normal=normal))
    return changes


def repair(path):
    fields, rows = load(path)
    changes = plan(rows)  # Validate every repair before writing anything.
    if not changes:
        return changes
    removed_ids = {r["edgeId"] for c in changes for r in c["removed"]}
    result = [r for r in rows if r["edgeId"] not in removed_ids]
    next_id = max(int(r["edgeId"]) for r in rows) + 1
    for offset, change in enumerate(changes):
        row = change["row"]
        row["edgeId"] = next_id + offset
        result.append({field: row.get(field, "") for field in fields})
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(result)
    temporary.replace(path)
    return changes


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    changes = repair(args.csv)
    print(f"Repaired {len(changes)} audited terminals")
    for change in changes:
        print(f"Road {change['road']}: {change['start']} -> {change['end']}, "
              f"trimmed {len(change['tail'])} inbound tail nodes")
