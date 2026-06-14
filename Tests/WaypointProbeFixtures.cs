namespace VoogleRoute.Pathfinding.Tests;

/// <summary>Forced waypoint-to-waypoint probes (DiagRunner parity).</summary>
public sealed class WaypointProbe
{
    public required string Id { get; init; }
    public int Start { get; init; }
    public int End { get; init; }
    public bool MustSucceed { get; init; } = true;
    public int? ExpectedEndWaypoint { get; init; }
    public float? MaxCostMeters { get; init; }
    public float? MaxCostToDirectRatio { get; init; }
}

public static class WaypointProbeFixtures
{
    public static IReadOnlyList<WaypointProbe> Bridge { get; } =
    [
        new() { Id = "bridge_1706_city_industrial", Start = 529, End = 7935, ExpectedEndWaypoint = 7935, MaxCostMeters = 3200f },
        new() { Id = "bridge_1703_city_industrial", Start = 7679, End = 12992, ExpectedEndWaypoint = 12992, MaxCostMeters = 4000f },
        new() { Id = "bridge_1705_city_industrial", Start = 3093, End = 703, ExpectedEndWaypoint = 703, MaxCostMeters = 2000f },
        new() { Id = "bridge_1706_1708_L0", Start = 6847, End = 6711, ExpectedEndWaypoint = 6711, MaxCostMeters = 10f },
        new() { Id = "bridge_1708_L0_corridor", Start = 6711, End = 2098, ExpectedEndWaypoint = 2098, MaxCostMeters = 1600f },
        new() { Id = "bridge_1708_L3_corridor", Start = 2152, End = 6028, ExpectedEndWaypoint = 6028, MaxCostMeters = 1600f },
        new() { Id = "bridge_deck_south_north_L0", Start = 7446, End = 1382, ExpectedEndWaypoint = 1382, MaxCostMeters = 200f },
        new() { Id = "bridge_deck_south_north_L1", Start = 7088, End = 8913, ExpectedEndWaypoint = 8913, MaxCostMeters = 200f },
        new() { Id = "bridge_city_1706_deck_south", Start = 529, End = 7446, ExpectedEndWaypoint = 7446, MaxCostMeters = 2200f },
    ];

    public static IReadOnlyList<WaypointProbe> Industrial { get; } =
    [
        new() { Id = "industrial_deck_south_zone", Start = 7446, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 1100f },
        new() { Id = "industrial_deck_south_168_L3", Start = 7446, End = 3572, ExpectedEndWaypoint = 3572, MaxCostMeters = 1100f },
        new() { Id = "industrial_city_bridge_zone", Start = 6847, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 3000f },
        new() { Id = "industrial_city_bridge_deck_south", Start = 6847, End = 7446, ExpectedEndWaypoint = 7446, MaxCostMeters = 2000f },
        new() { Id = "industrial_downtown_deck_south", Start = 516, End = 7446, ExpectedEndWaypoint = 7446, MaxCostMeters = 3500f },
        new() { Id = "industrial_downtown_zone", Start = 516, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 4500f },
    ];

    public static IReadOnlyList<WaypointProbe> North { get; } =
    [
        new() { Id = "north_deck_north_industrial", Start = 1382, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 900f },
        new() { Id = "north_deck_south_industrial", Start = 7446, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 1200f },
        new() { Id = "north_bridge_city_industrial", Start = 6847, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 3000f },
        new() { Id = "north_bridge_city_end_industrial", Start = 529, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 3200f },
        new() { Id = "north_downtown_industrial", Start = 516, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 4500f },
        new() { Id = "north_ne_corner_industrial", Start = 1133, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 5000f },
        new() { Id = "north_ne_deck_south", Start = 1133, End = 7446, ExpectedEndWaypoint = 7446, MaxCostMeters = 4500f },
        new() { Id = "north_limit_sw_pocket", Start = 7733, End = 3149, MustSucceed = false },
        new() { Id = "north_limit_nw_dead_end", Start = 4929, End = 3149, MustSucceed = false },
        new() { Id = "north_limit_se_dead_end", Start = 3891, End = 3149, MustSucceed = false },
    ];

    public static IReadOnlyList<WaypointProbe> Core { get; } =
    [
        new() { Id = "core_ne_industrial_anchor", Start = 1133, End = 13382, ExpectedEndWaypoint = 13382, MaxCostMeters = 5500f },
        new() { Id = "core_downtown_industrial", Start = 516, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 4500f },
        new() { Id = "core_bridge_city_industrial", Start = 6847, End = 3149, ExpectedEndWaypoint = 3149, MaxCostMeters = 3000f },
    ];

    public static IEnumerable<WaypointProbe> All =>
        Bridge.Concat(Industrial).Concat(North).Concat(Core);
}
