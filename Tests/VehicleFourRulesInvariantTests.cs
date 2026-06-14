using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using VoogleRoute.Pathfinding.Routing.Foot;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class VehicleFourRulesInvariantTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public VehicleFourRulesInvariantTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void UturnOff_NeverUsesAuthorizedUturnAsFirstStep(bool preferSide, bool _)
    {
        var scenario = VehicleRouteScenarios.All.First(s => s.Id == "third45");
        var query = new RouteQuery
        {
            Origin = scenario.Origin,
            Destination = scenario.Destination,
            Forward = scenario.Forward,
            HasPose = true,
            AllowUturnAtStart = false,
            PreferBuildingSideArrival = preferSide,
        };

        Assert.True(WaypointPathfinder.TryFindBestRoute(_graph, query, out var route));
        if (route.Path.Count < 2)
            return;

        var a = route.Path[0];
        var b = route.Path[1];
        if (_graph.IsAuthorizedUturnEdge(a, b))
            Assert.Fail($"first step must not be CSV uturn when AllowUturnAtStart=false (wp {a}->{b}).");
    }

    [Fact]
    public void EighthStreet_LongTrip_CurrentlyUnreachable()
    {
        // DiagRunner eighth_8_FAIL — documents known graph limit; fails if routing improves.
        var origin = new Vec3(131.28f, 0.44f, 121.01f);
        var dest = new Vec3(-1740.94f, 0.41f, -1163.29f);
        var query = new RouteQuery
        {
            Origin = origin,
            Destination = dest,
            Forward = new Vec3(0f, 0f, -1f),
            HasPose = true,
        };

        Assert.False(VehicleRoutePolyline.TryBuild(_graph, query, out _),
            "8th St extreme trip should remain unreachable until graph is extended.");
    }
}
