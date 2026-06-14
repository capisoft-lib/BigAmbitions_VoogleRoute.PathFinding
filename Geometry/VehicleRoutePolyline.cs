using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;

namespace VoogleRoute.Pathfinding.Geometry;

/// <summary>How the polyline tail is closed after A* (single source of truth for mod + DiagRunner).</summary>
public enum VehicleRouteAppendMode
{
    /// <summary>Append building GPS — default route line reaches the destination icon.</summary>
    DestinationGps,

    /// <summary>Append graph end lane waypoint — route stops on the curb lane (no chord to building).</summary>
    EndLaneWaypoint
}

/// <summary>Full vehicle route: A* path + display polyline. Mod layer only applies Y offset / rendering.</summary>
public sealed class VehicleRoutePolylineResult
{
    public required IReadOnlyList<Vec3> Points { get; init; }

    public required RouteResult Route { get; init; }

    public Vec3 AppendPoint { get; init; }

    public VehicleRouteAppendMode AppendMode { get; init; }

    public float PolylineLengthMeters { get; init; }

    public float GraphCostMeters => Route.TotalCostMeters;
}

public static class VehicleRoutePolyline
{
    public static bool TryBuild(
        IRoutingGraph graph,
        RouteQuery query,
        out VehicleRoutePolylineResult result)
    {
        result = null!;

        if (!WaypointPathfinder.TryFindBestRoute(graph, query, out var route) ||
            route.Path == null ||
            route.Path.Count == 0)
            return false;

        var appendMode = query.PreferBuildingSideArrival
            ? VehicleRouteAppendMode.EndLaneWaypoint
            : VehicleRouteAppendMode.DestinationGps;

        Vec3 appendPoint;
        Vec3? appendForBuilder;
        if (appendMode == VehicleRouteAppendMode.EndLaneWaypoint && route.EndWaypoint >= 0)
        {
            appendPoint = graph.GetPosition(route.EndWaypoint);
            appendForBuilder = appendPoint;
        }
        else
        {
            appendPoint = query.Destination;
            appendForBuilder = query.Destination;
        }

        var points = RoutePolylineBuilder.BuildPoints(
            graph,
            route.Path,
            prependOrigin: query.Origin,
            appendDestination: appendForBuilder);

        if (points.Count < 2)
            return false;

        result = new VehicleRoutePolylineResult
        {
            Points = points,
            Route = route,
            AppendPoint = appendPoint,
            AppendMode = appendMode,
            PolylineLengthMeters = RoutePolylineMetrics.FlatLength(points)
        };
        return true;
    }
}
