using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class WaypointProbeTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public WaypointProbeTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    public static IEnumerable<object[]> AllProbes() =>
        WaypointProbeFixtures.All.Select(p => new object[] { p });

    [Theory]
    [MemberData(nameof(AllProbes))]
    public void ForcedWaypointProbe_MatchesExpectation(WaypointProbe probe)
    {
        var query = new RouteQuery
        {
            Origin = _graph.GetPosition(probe.Start),
            Destination = _graph.GetPosition(probe.End),
            ForcedStartWaypoint = probe.Start,
            ForcedEndWaypoint = probe.End,
        };

        var direct = _graph.FlatDistance(query.Origin, query.Destination);
        var found = WaypointPathfinder.TryFindBestRoute(_graph, query, out var route);

        if (!probe.MustSucceed)
        {
            Assert.False(found, $"[{probe.Id}] expected no route from wp {probe.Start} to {probe.End}.");
            return;
        }

        Assert.True(found, $"[{probe.Id}] route must exist wp {probe.Start} -> {probe.End} (direct {direct:F0}m).");
        Assert.NotNull(route.Path);
        Assert.True(route.Path.Count >= 2, $"[{probe.Id}] path too short.");
        Assert.Equal(probe.Start, route.StartWaypoint);

        if (probe.ExpectedEndWaypoint is int endWp)
            Assert.Equal(endWp, route.EndWaypoint);

        if (probe.MaxCostMeters is float maxCost)
            Assert.True(route.TotalCostMeters <= maxCost,
                $"[{probe.Id}] cost {route.TotalCostMeters:F1}m > max {maxCost:F0}m.");

        if (probe.MaxCostToDirectRatio is float maxRatio && direct > 1f)
        {
            var ratio = route.TotalCostMeters / direct;
            Assert.True(ratio <= maxRatio,
                $"[{probe.Id}] cost/direct ratio {ratio:F2} > {maxRatio:F2}.");
        }
    }
}
