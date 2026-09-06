using System.Diagnostics;
using System.Security.Cryptography;
using VoogleRoute.Pathfinding.Geometry;
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
    public void ShippedGraph_ParallelLaneIndexMatchesApprovedFingerprint()
    {
        var buffer = new int[_graph.Size];
        var nonEmptyRows = 0;
        long directedPairs = 0;

        using var stream = new MemoryStream();
        using (var writer = new BinaryWriter(stream, System.Text.Encoding.UTF8, leaveOpen: true))
        {
            for (var index = 0; index < _graph.Size; index++)
            {
                buffer[0] = index;
                var count = _graph.ExpandLaneCandidates(buffer, 1, buffer.Length, default);
                var laneCount = count - 1;

                writer.Write(index);
                writer.Write(laneCount);
                if (laneCount > 0)
                    nonEmptyRows++;

                directedPairs += laneCount;
                for (var i = 1; i < count; i++)
                    writer.Write(buffer[i]);
            }
        }

        var fingerprint = Convert.ToHexString(SHA256.HashData(stream.ToArray()));
        Assert.Equal(8293, nonEmptyRows);
        Assert.Equal(66432, directedPairs);
        Assert.Equal(
            "9422028A56EC6C61C81C626A0AE775BC4654D3129BFCD443B42471DEEFF13A88",
            fingerprint);
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

    [Fact]
    public void IndustryCity_LongRoute_ReusesSearchWorkspace()
    {
        var query = IndustryQuery(new Vec3(-1740.94f, 0.41f, -1163.29f));
        Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out _));

        var before = GC.GetAllocatedBytesForCurrentThread();
        var timer = Stopwatch.StartNew();
        var successes = 0;
        for (var i = 0; i < 3; i++)
        {
            if (VehicleRoutePolyline.TryBuild(_graph, query, out _))
                successes++;
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Console.WriteLine($"valid industry x3: {timer.ElapsedMilliseconds} ms, {allocated} bytes allocated");
        Assert.Equal(3, successes);
        Assert.True(allocated < 6_000_000, $"Expected bounded reusable search allocations, got {allocated} bytes.");
    }

    [Fact]
    public void IndustryCity_Road236Fallback_IsBoundedAndCancelable()
    {
        var query = IndustryQuery(_graph.GetPosition(5286));
        Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out var first));
        Assert.True(first.Route.UsedFallbackArrival);
        Assert.InRange(first.Route.AccessEndMeters, 60f, 120f);

        var before = GC.GetAllocatedBytesForCurrentThread();
        var timer = Stopwatch.StartNew();
        var successes = 0;
        for (var i = 0; i < 3; i++)
        {
            if (VehicleRoutePolyline.TryBuild(_graph, query, out _))
                successes++;
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Console.WriteLine($"fallback industry x3: {timer.ElapsedMilliseconds} ms, {allocated} bytes allocated");
        Assert.Equal(3, successes);
        Assert.True(allocated < 6_000_000, $"Expected bounded reusable search allocations, got {allocated} bytes.");

        using var canceled = new CancellationTokenSource();
        canceled.Cancel();
        var canceledQuery = query with { CancellationToken = canceled.Token };
        Assert.False(VehicleRoutePolyline.TryBuild(_graph, canceledQuery, out _));
    }

    [Fact]
    public void IndustryCity_RepairedRoad213NearestCandidates_AreReachable()
    {
        var destination = new Vec3(-2185.557f, 0f, -1386.553f);
        var candidates = new int[32];
        var count = _graph.CollectNearest(destination, 250f, candidates);

        Assert.True(count >= 24, $"Expected at least 24 Industry arrival candidates, got {count}.");
        var reachability = new bool[Math.Min(count, 24)];
        for (var i = 0; i < reachability.Length; i++)
        {
            var end = candidates[i];
            var query = IndustryQuery(destination) with { ForcedEndWaypoint = end };
            var reachable = Routing.WaypointPathfinder.TryFindBestRoute(_graph, query, out var route);
            reachability[i] = reachable;
            var position = _graph.GetPosition(end);
            var distance = _graph.FlatDistance(position, destination);
            Console.WriteLine(
                $"candidate={i + 1:00} wp={end} distance={distance:F2} " +
                $"reachable={reachable} graphEnd={(reachable ? route.EndWaypoint : -1)} " +
                $"position=({position.X:F3},{position.Y:F3},{position.Z:F3})");
        }

        Assert.True(_graph.HasForwardEdge(3779, 14092));
        Assert.True(_graph.IsAuthorizedUturnEdge(3779, 14092));
        var generatedRows = File.ReadLines(_csvPath)
            .Where(line => line.Contains(",uturn,3779,Road_213-Lane_1-Out,") &&
                           line.Contains(",14092,Road_213-Lane_0-In,") &&
                           line.Contains(",generated_terminal_same_road_uturn,"))
            .ToArray();
        Assert.Single(generatedRows);
        Assert.All(reachability.Take(6), Assert.True);
    }

    [Fact]
    public void IndustryCity_Road213AndRoad236Targets_AreReachable()
    {
        var targetWaypoints = new[]
        {
            17894, 10479, 9922, 8884, 8163, 3703, // Road 213 lane 0
            925, 1118, 5286, 11600, 1309, 5617,   // Road 236 lane 0
        };

        foreach (var waypoint in targetWaypoints)
        {
            var destination = _graph.GetPosition(waypoint);
            var query = IndustryQuery(destination);
            Assert.True(
                VehicleRoutePolyline.TryBuild(_graph, query, out var built),
                $"Expected a reachable Industry arrival for waypoint {waypoint} at {destination}.");
            Assert.InRange(built.Route.AccessEndMeters, 0f, 120.01f);
            Assert.InRange(built.PolylineLengthMeters, 1f, 6000f);
        }
    }

    [Fact]
    public void IndustryCity_RepairedRoad213SupportsHeadingAndArrivalOptions()
    {
        var destination = new Vec3(-2185.557f, 0f, -1386.553f);
        var headings = new[]
        {
            new Vec3(0f, 0f, 1f),
            new Vec3(1f, 0f, 0f),
            new Vec3(0f, 0f, -1f),
            new Vec3(-1f, 0f, 0f),
        };

        foreach (var heading in headings)
        foreach (var allowUturn in new[] { false, true })
        foreach (var preferBuildingSide in new[] { false, true })
        {
            var query = IndustryQuery(destination) with
            {
                Forward = heading,
                AllowUturnAtStart = allowUturn,
                PreferBuildingSideArrival = preferBuildingSide,
            };

            Assert.True(
                VehicleRoutePolyline.TryBuild(_graph, query, out var built),
                $"Road 213 route failed for heading {heading}, uturn={allowUturn}, side={preferBuildingSide}.");
            Assert.False(built.Route.UsedFallbackArrival);
            Assert.InRange(built.Route.AccessEndMeters, 0f, 120.01f);
            Assert.InRange(built.PolylineLengthMeters, 1f, 6000f);
        }
    }

    private static Routing.RouteQuery IndustryQuery(Vec3 destination) => new()
    {
        Origin = new Vec3(131.28f, 0.44f, 121.01f),
        Destination = destination,
        Forward = new Vec3(0f, 0f, 1f),
        HasPose = true,
        ForcedStartWaypoint = -1,
        ForcedEndWaypoint = -1,
        AllowUturnAtStart = false,
        PreferBuildingSideArrival = false,
    };

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
