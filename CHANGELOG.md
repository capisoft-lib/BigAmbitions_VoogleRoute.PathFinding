# Changelog

## [Unreleased]

- Add six audited terminal U-turns on Roads 210, 222, 230, 233, 237 and 246; trim inbound tails so the route turns before the old terminal plane.
- Add an idempotent, identity-checked repair pass, twelve routing regressions, six Python checks, and a 23-point illustrated before/after audit with actual C# route polylines.
- The exact bridge-wall complaint and physical vehicle clearance remain unverified in game; see `docs/navigation-deadends/README.md` for unresolved locations.

## [0.11.2] - 2026-06-12

### Changed

- Enhanced route graph: bridge connectors, center-deck elevation, and downtown deck merges for reliable cross-river vehicle paths
- Graph loader and pathfinder aligned with updated `big_ambitions_enhanced_routes.csv`

## [0.11.1] - 2026-06-09

### Changed

- CSV graph loader and routing index aligned with in-game vehicle pathfinder
- Turn analyzer and penalty tuning for synthetic left turns and U-turns
- Updated route graph SVG documentation

## [0.11.0] - 2026-06-07

### Added

- Initial `VoogleRoute.Pathfinding` library (netstandard2.1)
- A* waypoint routing, enhanced route CSV loader, graph tooling

[0.11.2]: https://github.com/capisoft-lib/BigAmbitions_VoogleRoute.PathFinding/releases/tag/v0.11.2
[0.11.1]: https://github.com/capisoft-lib/BigAmbitions_VoogleRoute.PathFinding/releases/tag/v0.11.1
[0.11.0]: https://github.com/capisoft-lib/BigAmbitions_VoogleRoute.PathFinding/releases/tag/v0.11.0
