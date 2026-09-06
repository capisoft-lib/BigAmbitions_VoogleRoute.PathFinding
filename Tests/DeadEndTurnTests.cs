using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class DeadEndTurnTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;
    public DeadEndTurnTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    [Theory]
    [InlineData(15987, 10345, 14484, 11060, 10294)]
    [InlineData(3206, 5888, 14398, 6237, 15685)]
    [InlineData(10331, 9644, 6935, 3803, 6289)]
    [InlineData(13386, 17491, 3923, 4695, 8315)]
    [InlineData(12509, 8144, 2495, 17628, 9069)]
    [InlineData(7194, 8963, 8311, 11789, 7292)]
    public void Approach_CanTurnBackLocallyWithoutSelectingTheOldSink(
        int approach, int from, int to, int destination, int oldSink)
    {
        Assert.True(_graph.IsAuthorizedUturnEdge(from, to));
        Assert.DoesNotContain(oldSink, _graph.ValidIndices.ToArray());
        foreach (var allowStartTurn in new[] { false, true })
        {
            var query = new RouteQuery
            {
                Origin = _graph.GetPosition(approach),
                Destination = _graph.GetPosition(destination),
                Forward = _graph.GetPosition(from) - _graph.GetPosition(approach),
                HasPose = false,
                ForcedStartWaypoint = approach,
                ForcedEndWaypoint = destination,
                AllowUturnAtStart = allowStartTurn,
                PreferBuildingSideArrival = false,
            };
            Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out var built));
            Assert.InRange(built.GraphCostMeters, 1, 150);
            Assert.Contains(from, built.Route.Path!);
            Assert.Contains(to, built.Route.Path!);
            Assert.DoesNotContain(oldSink, built.Route.Path!);
            Assert.InRange(built.PolylineLengthMeters, 1, 100);
        }
    }

    [Theory]
    [InlineData(-2538.379f, -982.847f, 11060)]
    [InlineData(-2804.638f, -1035.410f, 6237)]
    [InlineData(-2877.386f, -1047.679f, 3803)]
    [InlineData(-2390.312f, -1577.938f, 4695)]
    [InlineData(-2598.166f, -986.245f, 17628)]
    [InlineData(-3273.312f, -1597.040f, 11789)]
    public void WorldOriginAtOldTerminal_StillFindsAReachableReturn(float x, float z, int target)
    {
        var query = new RouteQuery
        {
            Origin = new Vec3(x, 0, z),
            Destination = _graph.GetPosition(target),
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = target,
            AllowUturnAtStart = true,
        };
        Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out var built));
        Assert.InRange(built.PolylineLengthMeters, 1, 100);
    }
}
