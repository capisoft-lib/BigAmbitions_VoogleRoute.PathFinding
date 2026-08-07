#!/usr/bin/env python3
"""Carry forward hand-authored route edges after Gley waypoint indices change.

The legacy enhanced graph stores manual corrections by numeric waypoint index. A
game update can reorder every index while leaving the waypoint identities and
positions intact. This tool resolves each legacy endpoint through the legacy raw
dump, finds the nearest same-named waypoint in the new raw dump, and appends the
translated edge to a freshly generated enhanced graph.

Center-deck and Road 1703 collapse edges are intentionally excluded because the
dedicated preprocessors recreate them after bridge road remapping.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path


SKIPPED_SOURCE_PREFIXES = (
    "manual_bridge_center_",
    "manual_bridge_1703_collapse_",
)


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def raw_by_index(rows: list[dict[str, str]]) -> dict[int, dict[str, str]]:
    return {int(row["listIndex"]): row for row in rows}


def raw_by_name(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    result: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        result[row["name"]].append(row)
    return result


def position(row: dict[str, str], prefix: str = "pos") -> tuple[float, float, float]:
    return tuple(float(row[prefix + axis]) for axis in ("X", "Y", "Z"))


def squared_distance(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return sum((a - b) ** 2 for a, b in zip(left, right))


def resolve_new_waypoint(
    legacy_waypoint: dict[str, str],
    new_by_name: dict[str, list[dict[str, str]]],
) -> tuple[dict[str, str], float]:
    candidates = new_by_name.get(legacy_waypoint["name"], [])
    if not candidates:
        raise KeyError("Waypoint removed from new dump: " + legacy_waypoint["name"])

    old_position = position(legacy_waypoint)
    best = min(candidates, key=lambda row: squared_distance(old_position, position(row)))
    return best, math.sqrt(squared_distance(old_position, position(best)))


def parse_road_lane(name: str) -> tuple[str, str]:
    road = re.search(r"Road_(\d+)", name)
    lane = re.search(r"Lane_(\d+)", name)
    return (road.group(1) if road else "unknown", lane.group(1) if lane else "-1")


def translated_control(
    edge: dict[str, str],
    old_from: dict[str, str],
    old_to: dict[str, str],
    new_from: dict[str, str],
    new_to: dict[str, str],
) -> tuple[str, str, str]:
    if not edge.get("controlX"):
        return "", "", ""

    old_mid = tuple((a + b) * 0.5 for a, b in zip(position(old_from), position(old_to)))
    new_mid = tuple((a + b) * 0.5 for a, b in zip(position(new_from), position(new_to)))
    old_control = tuple(float(edge["control" + axis]) for axis in ("X", "Y", "Z"))
    translated = tuple(control + new - old for control, new, old in zip(old_control, new_mid, old_mid))
    return tuple(f"{value:.3f}" for value in translated)


def endpoint_fields(prefix: str, waypoint: dict[str, str]) -> dict[str, str]:
    road, lane = parse_road_lane(waypoint["name"])
    return {
        prefix + "Index": waypoint["listIndex"],
        prefix + "Name": waypoint["name"],
        prefix + "Road": road,
        prefix + "Lane": lane,
        prefix + "X": waypoint["posX"],
        prefix + "Y": waypoint["posY"],
        prefix + "Z": waypoint["posZ"],
    }


def migrate(
    legacy_graph: Path,
    legacy_raw: Path,
    new_raw: Path,
    target_graph: Path,
) -> tuple[int, int, float]:
    fields, target_rows = read_rows(target_graph)
    _, legacy_edges = read_rows(legacy_graph)
    _, legacy_waypoints = read_rows(legacy_raw)
    _, new_waypoints = read_rows(new_raw)

    if "bridgePart" not in fields:
        fields.append("bridgePart")
    for row in target_rows:
        row.setdefault("bridgePart", "")

    legacy_by_index = raw_by_index(legacy_waypoints)
    new_by_name = raw_by_name(new_waypoints)
    existing = {(int(row["fromIndex"]), int(row["toIndex"])) for row in target_rows}
    migrated: list[dict[str, str]] = []
    duplicate_count = 0
    max_distance = 0.0

    for edge in legacy_edges:
        source = edge.get("source", "")
        if not source.startswith("manual_") or source.startswith(SKIPPED_SOURCE_PREFIXES):
            continue

        old_from = legacy_by_index[int(edge["fromIndex"])]
        old_to = legacy_by_index[int(edge["toIndex"])]
        new_from, from_distance = resolve_new_waypoint(old_from, new_by_name)
        new_to, to_distance = resolve_new_waypoint(old_to, new_by_name)
        max_distance = max(max_distance, from_distance, to_distance)

        key = (int(new_from["listIndex"]), int(new_to["listIndex"]))
        if key in existing:
            duplicate_count += 1
            continue

        translated = {field: edge.get(field, "") for field in fields}
        translated.update(endpoint_fields("from", new_from))
        translated.update(endpoint_fields("to", new_to))
        control_x, control_y, control_z = translated_control(
            edge, old_from, old_to, new_from, new_to
        )
        translated["controlX"] = control_x
        translated["controlY"] = control_y
        translated["controlZ"] = control_z
        translated["edgeId"] = ""
        migrated.append(translated)
        existing.add(key)

    migrated.sort(key=lambda row: (row["source"], int(row["fromIndex"]), int(row["toIndex"])))
    target_rows.extend(migrated)
    for edge_id, row in enumerate(target_rows):
        row["edgeId"] = str(edge_id)

    with target_graph.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(target_rows)

    return len(migrated), duplicate_count, max_distance


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("legacy_graph", type=Path)
    parser.add_argument("legacy_raw", type=Path)
    parser.add_argument("new_raw", type=Path)
    parser.add_argument("target_graph", type=Path)
    args = parser.parse_args()

    migrated, duplicates, max_distance = migrate(
        args.legacy_graph,
        args.legacy_raw,
        args.new_raw,
        args.target_graph,
    )
    print(
        f"Migrated {migrated} manual edges; skipped {duplicates} edges already present; "
        f"maximum endpoint movement {max_distance:.3f} m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
