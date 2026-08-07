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
        new() { Id = "bridge_1706_city_industrial", Start = 719, End = 10658, ExpectedEndWaypoint = 10658, MaxCostMeters = 3200f },
        new() { Id = "bridge_1703_city_industrial", Start = 10312, End = 17374, ExpectedEndWaypoint = 17374, MaxCostMeters = 4000f },
        new() { Id = "bridge_1705_city_industrial", Start = 4155, End = 957, ExpectedEndWaypoint = 957, MaxCostMeters = 2000f },
        new() { Id = "bridge_1706_1708_L0", Start = 9179, End = 8988, ExpectedEndWaypoint = 8988, MaxCostMeters = 10f },
        new() { Id = "bridge_1708_L0_corridor", Start = 8988, End = 2844, ExpectedEndWaypoint = 2844, MaxCostMeters = 1600f },
        new() { Id = "bridge_1708_L3_corridor", Start = 2914, End = 8072, ExpectedEndWaypoint = 8072, MaxCostMeters = 1600f },
        new() { Id = "bridge_deck_south_north_L0", Start = 9981, End = 1891, ExpectedEndWaypoint = 1891, MaxCostMeters = 200f },
        new() { Id = "bridge_deck_south_north_L1", Start = 9505, End = 11992, ExpectedEndWaypoint = 11992, MaxCostMeters = 200f },
        new() { Id = "bridge_city_1706_deck_south", Start = 719, End = 9981, ExpectedEndWaypoint = 9981, MaxCostMeters = 2200f },
    ];

    public static IReadOnlyList<WaypointProbe> Industrial { get; } =
    [
        new() { Id = "industrial_deck_south_zone", Start = 9981, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 1100f },
        new() { Id = "industrial_deck_south_168_L3", Start = 9981, End = 4752, ExpectedEndWaypoint = 4752, MaxCostMeters = 1100f },
        new() { Id = "industrial_city_bridge_zone", Start = 9179, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 3000f },
        new() { Id = "industrial_city_bridge_deck_south", Start = 9179, End = 9981, ExpectedEndWaypoint = 9981, MaxCostMeters = 2000f },
        new() { Id = "industrial_downtown_deck_south", Start = 697, End = 9981, ExpectedEndWaypoint = 9981, MaxCostMeters = 3500f },
        new() { Id = "industrial_downtown_zone", Start = 697, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 4500f },
    ];

    public static IReadOnlyList<WaypointProbe> North { get; } =
    [
        new() { Id = "north_deck_north_industrial", Start = 1891, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 900f },
        new() { Id = "north_deck_south_industrial", Start = 9981, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 1200f },
        new() { Id = "north_bridge_city_industrial", Start = 9179, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 3000f },
        new() { Id = "north_bridge_city_end_industrial", Start = 719, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 3200f },
        new() { Id = "north_downtown_industrial", Start = 697, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 4500f },
        new() { Id = "north_ne_corner_industrial", Start = 1560, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 5000f },
        new() { Id = "north_ne_deck_south", Start = 1560, End = 9981, ExpectedEndWaypoint = 9981, MaxCostMeters = 4500f },
        new() { Id = "north_limit_sw_pocket", Start = 3949, End = 4226, MustSucceed = false },
        new() { Id = "north_limit_nw_dead_end", Start = 6589, End = 4226, MustSucceed = false },
        new() { Id = "north_limit_se_dead_end", Start = 5181, End = 4226, MustSucceed = false },
    ];

    public static IReadOnlyList<WaypointProbe> Core { get; } =
    [
        new() { Id = "core_ne_industrial_anchor", Start = 1560, End = 17901, ExpectedEndWaypoint = 17901, MaxCostMeters = 5500f },
        new() { Id = "core_downtown_industrial", Start = 697, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 4500f },
        new() { Id = "core_bridge_city_industrial", Start = 9179, End = 4226, ExpectedEndWaypoint = 4226, MaxCostMeters = 3000f },
    ];

    public static IEnumerable<WaypointProbe> All =>
        Bridge.Concat(Industrial).Concat(North).Concat(Core);
}
