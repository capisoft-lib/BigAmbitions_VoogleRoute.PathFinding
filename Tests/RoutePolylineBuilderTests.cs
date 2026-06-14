using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class RoutePolylineBuilderTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public RoutePolylineBuilderTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    [Fact]
    public void BuildPoints_ForcedWaypointRoute_HasAtLeastTwoPoints()
    {
        var start = 516;
        var end = 3149;
        var query = new RouteQuery
        {
            Origin = _graph.GetPosition(start),
            Destination = _graph.GetPosition(end),
            ForcedStartWaypoint = start,
            ForcedEndWaypoint = end,
        };

        Assert.True(WaypointPathfinder.TryFindBestRoute(_graph, query, out var route));
        var points = RoutePolylineBuilder.BuildPoints(_graph, route.Path, appendDestination: query.Destination);
        Assert.True(points.Count >= 2);
        Assert.True(RoutePolylineMetrics.FlatLength(points) > 100f);
    }

    [Fact]
    public void BuildPoints_EndLaneAppend_StopsOnLaneNotDestination()
    {
        var scenario = VehicleRouteScenarios.All.First(s => s.Id == "third45");
        var query = new RouteQuery
        {
            Origin = scenario.Origin,
            Destination = scenario.Destination,
            Forward = scenario.Forward,
            HasPose = true,
            PreferBuildingSideArrival = true,
            AllowUturnAtStart = false,
        };

        Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out var built));
        Assert.Equal(VehicleRouteAppendMode.EndLaneWaypoint, built.AppendMode);
        var lane = _graph.GetPosition(built.Route.EndWaypoint);
        Assert.True(Vec3.FlatDistSq(built.Points[^1], lane) < 1f);
    }

    [Fact]
    public void FlatLength_StraightLine_EqualsDistance()
    {
        var a = new Vec3(0f, 0f, 0f);
        var b = new Vec3(100f, 0f, 0f);
        var points = new List<Vec3> { a, b };
        Assert.InRange(RoutePolylineMetrics.FlatLength(points), 99.9f, 100.1f);
    }
}
