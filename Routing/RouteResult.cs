using System;

namespace VoogleRoute.Pathfinding.Routing;

public sealed class RouteResult
{
    public required IReadOnlyList<int> Path { get; init; }
    public required int StartWaypoint { get; init; }
    public required int EndWaypoint { get; init; }
    public float GraphCostMeters { get; init; }
    public float AccessStartMeters { get; init; }
    public float AccessEndMeters { get; init; }
    public float TotalCostMeters => GraphCostMeters + AccessStartMeters + AccessEndMeters;
    public int NodesExplored { get; init; }
    public IReadOnlyList<PathTurn> Turns { get; init; } = Array.Empty<PathTurn>();
    public TurnSummary TurnSummary { get; init; }
}

public sealed class RouteCompareResult
{
    public RouteResult? WithPenalties { get; init; }
    public RouteResult? WithoutPenalties { get; init; }
    public RouteResult? ShortestDistance { get; init; }
    public string? Error { get; init; }
}
