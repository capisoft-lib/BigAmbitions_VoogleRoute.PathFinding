# VoogleRoute.Pathfinding

Shared **netstandard2.1** routing library for [Voogle Route](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute) (Unity mod) and [Voogle Route Web](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute.Web).

| Property | Value |
|----------|-------|
| **Target** | .NET Standard 2.1 |
| **Assembly** | `VoogleRoute.Pathfinding.dll` |
| **Algorithm** | A* on a precomputed traffic waypoint graph |

## What this library does

- Loads the **enhanced route graph** from `big_ambitions_enhanced_routes.csv`
- Finds vehicle routes with `WaypointPathfinder` (A*, turn penalties, U-turn whitelist)
- Builds display polylines with `RoutePolylineBuilder`
- Optional corridor / line-detection helpers for debug overlays

**On-foot** routing uses Unity NavMesh inside the mod — not this library.

## Build

```powershell
dotnet build VoogleRoute.Pathfinding.csproj -c Release
```

Output: `bin/Release/netstandard2.1/VoogleRoute.Pathfinding.dll`

## Consumers

| Project | Integration |
|---------|-------------|
| **[VoogleRoute mod](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute)** | git submodule at `VoogleRoute/PathFinding/` → `VoogleRoute/tools/build-pathfinding.ps1` copies the DLL to `Dependencies/` |
| **[VoogleRoute.Web](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute.Web)** | `<ProjectReference>` to this `.csproj` |

## Enhanced driving graph

Vanilla **Gley Traffic System** waypoints model forward lane connectivity well, but they do **not** expose every **left turn** or **U-turn** a driver needs at intersections. Voogle Route ships a precomputed graph extension so vehicle routing can turn onto cross streets instead of only going straight.

### Pipeline overview

```
In-game Gley waypoints (CurrentSceneData.allWaypoints)
        │
        ▼  export to CSV (listIndex, name, position, neighbors, …)
        │
        ▼  BigAmbitions_VoogleRoute/tools/generate_enhanced_route_graph.py
        │     • keep all base Gley edges (edgeType=base, source=gley)
        │     • detect intersection exits/entries per road lane
        │     • add synthetic_turn / left  (green curves on map)
        │     • add synthetic_turn / uturn (orange curves on map)
        │
        ▼  VoogleRoute/Data/big_ambitions_enhanced_routes.csv  (shipped with mod)
        │
        ▼  runtime: CsvRouteGraphLoader → RouteGraph → WaypointPathfinder
```

Graph generation scripts and QA maps live in the **[VoogleRoute repository](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute)** (`tools/`, `docs/`). This repository owns the **loader, graph model, and pathfinder**.

### Step 1 — Extract the base graph

Dump the city's **Gley** `Waypoint[]` graph to a CSV with one row per waypoint:

- `listIndex`, `name`, `posX` / `posY` / `posZ`, `neighbors` (semicolon-separated indices), `disabled`

Connectors (`Connector`, `CConnect`) and disabled nodes are filtered during enhancement.

### Step 2 — Generate synthetic turns

[`generate_enhanced_route_graph.py`](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute/blob/main/tools/generate_enhanced_route_graph.py) writes:

- `big_ambitions_enhanced_routes.csv` — base edges + synthetic maneuvers
- `big_ambitions_enhanced_route_graph.svg` — visual QA map

**Left turns (`maneuver=left`)**

- Consider only **leftmost driving lanes** at each road (lane-direction clustering).
- At each intersection **exit** waypoint, pair with nearby **entry** waypoints on other roads.
- Keep candidates where the signed turn angle is **+28° to +142°**.
- Skip pairs already reachable through the base graph (short BFS).
- Store a quadratic **control point** (Bezier) for smooth on-ground rendering.

**U-turns (`maneuver=uturn`)**

- **Parallel corridor pairs** (e.g. Roads 10↔11, 47↔48): one authorized ~180° link per intersection station.
- **Internal multi-lane roads**: U-turn from leftmost exit back to leftmost entry when geometry is ~145°–181°.
- U-turn edges are whitelisted at runtime — generic ~180° turns on the base graph remain blocked.

Regenerate after a game update that changes city traffic data:

```bash
python tools/generate_enhanced_route_graph.py <waypoints_dump.csv> VoogleRoute/Data/big_ambitions_enhanced_routes.csv docs/big_ambitions_enhanced_route_graph.svg
```

(Run from a clone of **BigAmbitions_VoogleRoute**.)

### Step 3 — Runtime (mod)

At city load the mod calls `RouteGraphStore.WarmUp()` once. `CsvRouteGraphLoader` parses the shipped CSV into a `RouteGraph`. `RoutePathfinder` builds a `RouteQuery` from the player pose and destination, then `WaypointPathfinder.TryFindBestRoute` returns the waypoint path.

The graph is **not** reloaded on every destination change — only the A* query runs again.

### Map visualization

Grey polylines = original **Gley** edges. Green curves = **left turns**. Orange curves = **U-turns**.

See [docs/big_ambitions_enhanced_route_graph.svg](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute/blob/main/docs/big_ambitions_enhanced_route_graph.svg) in the VoogleRoute repo.

## Source layout

```text
Graph/          CsvRouteGraphLoader, RouteGraph, lane-change expansion
Routing/        WaypointPathfinder, turn analysis, penalties
Geometry/       polylines, corridor, line detection
DiagRunner/     offline route diagnostics (optional)
```

## License

MIT
