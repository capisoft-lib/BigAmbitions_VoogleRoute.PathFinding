using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class StartManeuverPolicyTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public StartManeuverPolicyTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    [Fact]
    public void AuthorizedUturnEdge_IsBlockedAtStart()
    {
        var edge = FindAuthorizedUturnEdge();
        Assert.True(edge.HasValue, "graph must contain at least one CSV uturn edge.");
        var (from, to) = edge.Value;
        Assert.True(_graph.IsAuthorizedUturnEdge(from, to));
        Assert.True(StartManeuverPolicy.IsBlockedManeuverAtStart(_graph, from, to));
    }

    [Fact]
    public void Third45_FirstStep_WithUturnDisabled_IsNotAuthorizedUturn()
    {
        var scenario = VehicleRouteScenarios.All.First(s => s.Id == "third45");
        var query = new RouteQuery
        {
            Origin = scenario.Origin,
            Destination = scenario.Destination,
            Forward = scenario.Forward,
            HasPose = true,
            AllowUturnAtStart = false,
            PreferBuildingSideArrival = true,
        };

        Assert.True(WaypointPathfinder.TryFindBestRoute(_graph, query, out var route));
        Assert.True(route.Path.Count >= 2);
        var first = route.Path[0];
        var second = route.Path[1];
        Assert.False(_graph.IsAuthorizedUturnEdge(first, second));
    }

    [Fact]
    public void UturnAtStart_Allowed_ProducesShorterThird45SideOffRoute()
    {
        var scenario = VehicleRouteScenarios.All.First(s => s.Id == "third45");
        var withUturn = BuildCost(scenario, allowUturn: true);
        var noUturn = BuildCost(scenario, allowUturn: false);
        Assert.True(withUturn < noUturn);
    }

    private float BuildCost(VehicleRouteScenario scenario, bool allowUturn)
    {
        var query = new RouteQuery
        {
            Origin = scenario.Origin,
            Destination = scenario.Destination,
            Forward = scenario.Forward,
            HasPose = true,
            AllowUturnAtStart = allowUturn,
            PreferBuildingSideArrival = false,
        };

        Assert.True(WaypointPathfinder.TryFindBestRoute(_graph, query, out var route));
        return route.TotalCostMeters;
    }

    private (int From, int To)? FindAuthorizedUturnEdge()
    {
        foreach (var idx in _graph.ValidIndices)
        {
            foreach (var next in _graph.GetForwardNeighbors(idx))
            {
                if (_graph.IsAuthorizedUturnEdge(idx, next))
                    return (idx, next);
            }
        }

        return null;
    }
}
