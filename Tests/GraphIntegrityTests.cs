using VoogleRoute.Pathfinding.Graph;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class GraphIntegrityTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;
    private readonly string _csvPath;

    public GraphIntegrityTests(RouteGraphFixture fixture)
    {
        _graph = fixture.Graph;
        _csvPath = RouteGraphFixture.ResolveGraphCsv();
    }

    [Fact]
    public void ShippedGraph_HasExpectedNodeCount()
    {
        Assert.Equal(17921, _graph.Size);
    }

    [Fact]
    public void ShippedGraph_HasValidWaypointSpan()
    {
        var valid = _graph.ValidIndices;
        Assert.True(valid.Length > 1000);
        Assert.True(ContainsWaypoint(valid, 697));
        Assert.True(ContainsWaypoint(valid, 4226));
        Assert.True(ContainsWaypoint(valid, 9179));
        Assert.True(ContainsWaypoint(valid, 9710));
    }

    private static bool ContainsWaypoint(ReadOnlySpan<int> indices, int waypoint)
    {
        for (var i = 0; i < indices.Length; i++)
        {
            if (indices[i] == waypoint)
                return true;
        }

        return false;
    }

    [Fact]
    public void Csv_ContainsBaseAndSyntheticEdges()
    {
        var baseCount = 0;
        var leftCount = 0;
        var uturnCount = 0;

        foreach (var line in File.ReadLines(_csvPath))
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith("edge", StringComparison.OrdinalIgnoreCase))
                continue;

            var cols = line.Split(',');
            if (cols.Length < 3)
                continue;

            if (cols[1] == "base")
                baseCount++;
            else if (cols[1] == "synthetic_turn")
            {
                if (cols[2] == "left")
                    leftCount++;
                else if (cols[2] == "uturn")
                    uturnCount++;
            }
        }

        Assert.True(baseCount > 12000, $"base edges={baseCount}");
        Assert.True(leftCount > 300, $"left turns={leftCount}");
        Assert.True(uturnCount > 30, $"uturn connectors={uturnCount}");
    }

    [Fact]
    public void CriticalPairs_AreReachable()
    {
        AssertReachable(697, 4226, "downtown->industrial");
        AssertReachable(9179, 4226, "bridge city->industrial");
        AssertReachable(1560, 17901, "NE->industrial anchor");
        AssertReachable(719, 9981, "city 1706->deck south");
    }

    [Fact]
    public void IsolatedComponents_AreNotReachableToIndustrialZone()
    {
        AssertNotReachable(3949, 4226, "SW pocket");
        AssertNotReachable(6589, 4226, "NW dead-end");
    }

    private void AssertReachable(int start, int end, string label)
    {
        var q = new Routing.RouteQuery
        {
            Origin = _graph.GetPosition(start),
            Destination = _graph.GetPosition(end),
            ForcedStartWaypoint = start,
            ForcedEndWaypoint = end,
        };

        Assert.True(
            Routing.WaypointPathfinder.TryFindBestRoute(_graph, q, out var route),
            $"{label}: wp {start}->{end} must be reachable.");
        Assert.Equal(end, route.EndWaypoint);
    }

    private void AssertNotReachable(int start, int end, string label)
    {
        var q = new Routing.RouteQuery
        {
            Origin = _graph.GetPosition(start),
            Destination = _graph.GetPosition(end),
            ForcedStartWaypoint = start,
            ForcedEndWaypoint = end,
        };

        Assert.False(
            Routing.WaypointPathfinder.TryFindBestRoute(_graph, q, out _),
            $"{label}: wp {start}->{end} must NOT be reachable.");
    }
}
