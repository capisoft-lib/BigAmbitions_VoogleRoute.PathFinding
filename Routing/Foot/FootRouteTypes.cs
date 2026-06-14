namespace VoogleRoute.Pathfinding.Routing.Foot;

using System;
using VoogleRoute.Pathfinding.Geometry;

public sealed class FootRouteSegment
{
    public FootRouteSegmentKind Kind { get; init; }
    public required IReadOnlyList<Vec3> Points { get; init; }
}

public sealed class FootSubwayHint
{
    public bool Active { get; init; }
    public string BoardStationName { get; init; } = string.Empty;
    public string ExitStationName { get; init; } = string.Empty;
    public Vec3 BoardNavPosition { get; init; }
    public Vec3 ExitNavPosition { get; init; }
    public Vec3 BoardWorldPosition { get; init; }
    public Vec3 ExitWorldPosition { get; init; }

    public static FootSubwayHint None { get; } = new();
}

public sealed class FootRouteResult
{
    public bool Success { get; init; }
    public bool IsPartial { get; init; }
    public IReadOnlyList<Vec3> Points { get; init; } = Array.Empty<Vec3>();
    public IReadOnlyList<FootRouteSegment> Segments { get; init; } = Array.Empty<FootRouteSegment>();
    public FootSubwayHint Subway { get; init; } = FootSubwayHint.None;

    public bool UsesSubway => Subway.Active;

    public static FootRouteResult None { get; } = new();
}

public sealed class FootLegResult
{
    public bool Success { get; init; }
    public bool IsPartial { get; init; }
    public IReadOnlyList<Vec3> Points { get; init; } = Array.Empty<Vec3>();

    public float WalkMeters =>
        Points.Count >= 2 ? RoutePolylineMetrics.FlatLength(Points) : 0f;
}

public sealed class FootRouteOptions
{
    public bool UseSubwayEnabled { get; init; } = true;
    public bool AllowSubwayPlanning { get; init; } = true;
    public bool ShowPartialPaths { get; init; } = true;
    public int MaxStationCandidates { get; init; } = 5;
    public float MaxStationPickMeters { get; init; } = 900f;
}
