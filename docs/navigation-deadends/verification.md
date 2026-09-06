# Validation — 2026-09-06

- `dotnet test Tests/VoogleRoute.Pathfinding.Tests.csproj -c Release -m:1`: **215 passed**, including 12 new terminal routing regressions.
- `python -m unittest discover -s tools -p test_repair_deadend_turns.py`: **6 passed**. Includes byte-exact regeneration from the recorded baseline rows, idempotence, all sampled curves behind the terminal plane, duplicate rejection and no write on coordinate drift.
- Baseline and revised polylines exported with the same C# library, fixed origin/target waypoint pairs, `AllowUturnAtStart=false`, no pose, normal destination arrival. They measure local return connectivity, not every possible building-side arrival.
- 15 base rows replaced by 6 explicit U-turn rows. All other CSV rows remain identical. Fourteen inbound tail nodes cease being navigation candidates; no new waypoint IDs or cross-road edges are introduced.
- Parallel-lane fingerprint reviewed: 51 changed rows; changed pairs involve deleted tails or the newly marked turn anchors. Unrelated pairs remain unchanged. Counts: 8293 → 8283 nonempty rows, 66432 → 66342 directed candidate pairs.
- `build-pathfinding.ps1 -SkipTests`: net48 player dependency built successfully after the full tests. Two existing `InvalidDataException` polyfill warnings.
- VoogleRoute `compile-install-voogle-route.ps1 -NoInstall`: player-mode compilation and package verification pass with the preserved local shortcut/map changes. Three existing TMP obsolete-property warnings.
- Graph SHA256 in PathFinding, VoogleRoute Data and packaged Output: `4edca0e11bf4d30d0c364247af2bf295144fe82f209feeb29682c0f35edea540`.
- Inspected the rendered comparison images, complete detours and unchanged-point sheets. No game installation, Steam publication, screenshot validation, collision test or confirmed location for the quoted bridge-wall complaint.

The six turn envelopes are bounded by the **old waypoint terminal plane**, not a surveyed wall. On Roads 237, 222 and 233 another native lane can continue into a connector: these are reviewed inner-lane return opportunities, not proof that the entire street is a physical cul-de-sac. Roads 235 and the three synthetic-only exits listed in the audit still need terrain confirmation.
