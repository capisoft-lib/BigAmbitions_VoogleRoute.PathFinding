using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Routing.Foot;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class FootRouteSubwayTests
{
    private static readonly SubwayNetwork Subway = new();

    private static readonly Vec3 DowntownOrigin = new(110f, 0f, 110f);
    private static readonly Vec3 IndustrialTarget = new(910f, 0f, -910f);
    private static readonly Vec3 ShortOrigin = new(50f, 0f, 50f);
    private static readonly Vec3 ShortTarget = new(80f, 0f, 55f);

    [Fact]
    public void DirectOnly_WhenSubwayDisabled_ReturnsFootWithoutSubwaySegment()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 150f)
            .AddLeg(ShortOrigin, board.NavPosition, 80f)
            .AddLeg(exit.NavPosition, ShortTarget, 90f);

        var options = new FootRouteOptions { UseSubwayEnabled = false };
        var stations = new[] { board, exit };

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot, stations, Subway, options, out var result));

        Assert.False(result.UsesSubway);
        Assert.Single(result.Segments);
        Assert.Equal(FootRouteSegmentKind.Foot, result.Segments[0].Kind);
        Assert.InRange(MeasureFootWalk(result), 140f, 160f);
    }

    [Fact]
    public void DirectOnly_WhenAllowSubwayPlanningFalse_SkipsSubwayEvenIfEnabled()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(DowntownOrigin, IndustrialTarget, 2800f)
            .AddLeg(DowntownOrigin, board.NavPosition, 50f)
            .AddLeg(exit.NavPosition, IndustrialTarget, 60f);

        var options = new FootRouteOptions
        {
            UseSubwayEnabled = true,
            AllowSubwayPlanning = false
        };

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            DowntownOrigin, IndustrialTarget, DowntownOrigin, foot,
            new[] { board, exit }, Subway, options, out var result));

        Assert.False(result.UsesSubway);
    }

    [Fact]
    public void DirectChosen_WhenDirectExists_SubwayIgnoredEvenIfWalkShorter()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(DowntownOrigin, IndustrialTarget, 2800f)
            .AddLeg(DowntownOrigin, board.NavPosition, 80f)
            .AddLeg(exit.NavPosition, IndustrialTarget, 120f);

        var options = new FootRouteOptions { UseSubwayEnabled = true };

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            DowntownOrigin, IndustrialTarget, DowntownOrigin, foot,
            new[] { board, exit }, Subway, options, out var result));

        Assert.False(result.UsesSubway);
        Assert.Single(result.Segments);
        Assert.Equal(FootRouteSegmentKind.Foot, result.Segments[0].Kind);
        Assert.InRange(MeasureFootWalk(result), 2790f, 2810f);
    }

    [Fact]
    public void DirectChosen_WhenWalkOnlyLongerThanDirect()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 150f)
            .AddLeg(ShortOrigin, board.NavPosition, 200f)
            .AddLeg(exit.NavPosition, ShortTarget, 250f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot,
            new[] { board, exit }, Subway, new FootRouteOptions(), out var result));

        Assert.False(result.UsesSubway);
        Assert.InRange(MeasureFootWalk(result), 140f, 160f);
    }

    [Fact]
    public void SubwayChosen_WhenNoDirectPathExists()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(DowntownOrigin, board.NavPosition, 70f)
            .AddLeg(exit.NavPosition, IndustrialTarget, 85f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            DowntownOrigin, IndustrialTarget, DowntownOrigin, foot,
            new[] { board, exit }, Subway, new FootRouteOptions(), out var result));

        Assert.True(result.UsesSubway);
        Assert.InRange(MeasureFootWalk(result), 150f, 160f);
    }

    [Fact]
    public void PicksBestStationPair_WhenMultipleCandidates()
    {
        var board = FootTestStations.DowntownBoard;
        var mid = FootTestStations.MidTown;
        var exit = FootTestStations.IndustrialExit;
        var origin = new Vec3(105f, 0f, 105f);
        var target = new Vec3(905f, 0f, -905f);

        var foot = new FakeFootPathProvider()
            .AddLeg(origin, board.NavPosition, 60f)
            .AddLeg(origin, mid.NavPosition, 350f)
            .AddLeg(mid.NavPosition, target, 400f)
            .AddLeg(exit.NavPosition, target, 90f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            origin, target, origin, foot,
            new[] { board, mid, exit }, Subway, new FootRouteOptions(), out var result));

        Assert.True(result.UsesSubway);
        Assert.Equal("Test_Downtown", result.Subway.BoardStationName);
        Assert.Equal("Test_Industrial", result.Subway.ExitStationName);
    }

    [Fact]
    public void NeverUsesSameStationForBoardAndExit()
    {
        var only = FootTestStations.DowntownBoard;
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 500f)
            .AddLeg(ShortOrigin, only.NavPosition, 40f)
            .AddLeg(only.NavPosition, ShortTarget, 50f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot,
            new[] { only }, Subway, new FootRouteOptions(), out var result));

        Assert.False(result.UsesSubway);
    }

    [Fact]
    public void SubwayNetwork_BuildsBridgePath_WhenCrossingToIndustrialCity()
    {
        var network = new SubwayNetwork();
        network.SetBridgePaths(
            new[] { new Vec3(10f, 0f, 0f), new Vec3(20f, 0f, 0f) },
            new[] { new Vec3(30f, 0f, 0f), new Vec3(40f, 0f, 0f) });

        var from = new SubwayStation
        {
            Index = 0,
            StationName = "LM",
            Neighborhood = "ba:neighborhood_downtown",
            NavPosition = new Vec3(0f, 0f, 0f),
            WorldPosition = new Vec3(0f, 0f, 0f)
        };
        var to = new SubwayStation
        {
            Index = 1,
            StationName = "IC",
            Neighborhood = SubwayNetwork.IndustryCityNeighborhood,
            NavPosition = new Vec3(100f, 0f, -100f),
            WorldPosition = new Vec3(100f, 0f, -100f)
        };

        var path = network.BuildDisplayPath(from, to);
        Assert.True(path.Count >= 3);
        Assert.Equal(10f, path[1].X);
        Assert.Equal(100f, path[^1].X);
    }

    [Fact]
    public void CsvSubwayStationLoader_ParsesTestFixture()
    {
        var csv = Path.Combine(AppContext.BaseDirectory, "data", "test_subway_stations.csv");
        var stations = CsvSubwayStationLoader.LoadFromCsv(csv);
        Assert.Equal(3, stations.Count);
        Assert.Equal("Fixture_A", stations[0].StationName);
        Assert.Equal(12.5f, stations[0].NavPosition.X);
    }

    [Fact]
    public void CsvSubwayStationLoader_ParsesPackagedFallback()
    {
        var csv = Path.Combine(AppContext.BaseDirectory, "data", "subway_stations.csv");
        var stations = CsvSubwayStationLoader.LoadFromCsv(csv);

        Assert.Equal(20, stations.Count);
        Assert.Equal(20, stations.Select(station => station.StationName).Distinct(StringComparer.Ordinal).Count());
        Assert.Contains(stations, station => station.StationName == "TheHamptonsStation");
        Assert.All(stations, station =>
        {
            Assert.False(string.IsNullOrWhiteSpace(station.StationName));
            Assert.NotEqual(new Vec3(0f, 0f, 0f), station.WorldPosition);
            Assert.NotEqual(new Vec3(0f, 0f, 0f), station.NavPosition);
        });
    }

    [Fact]
    public void DirectChosen_WhenWalkMetersEqual_SubwayNotPreferred()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 200f)
            .AddLeg(ShortOrigin, board.NavPosition, 100f)
            .AddLeg(exit.NavPosition, ShortTarget, 100f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot,
            new[] { board, exit }, Subway, new FootRouteOptions(), out var result));

        Assert.False(result.UsesSubway);
    }

    [Fact]
    public void PartialDirect_FallsBackToSubway_WhenConnectorLegsExist()
    {
        var board = FootTestStations.DowntownBoard;
        var exit = FootTestStations.IndustrialExit;
        var foot = new FakeFootPathProvider()
            .AddLeg(DowntownOrigin, IndustrialTarget, 500f, isPartial: true)
            .AddLeg(DowntownOrigin, board.NavPosition, 70f)
            .AddLeg(exit.NavPosition, IndustrialTarget, 85f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            DowntownOrigin, IndustrialTarget, DowntownOrigin, foot,
            new[] { board, exit }, Subway, new FootRouteOptions(), out var result));

        Assert.True(result.UsesSubway);
    }

    [Fact]
    public void PartialLegRejected_WhenShowPartialPathsFalse()
    {
        var board = FootTestStations.DowntownBoard;
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 100f, isPartial: true);

        Assert.False(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot,
            new[] { board }, Subway, new FootRouteOptions { ShowPartialPaths = false }, out _));
    }

    [Fact]
    public void StationBeyondPickRadius_IsIgnored()
    {
        var far = new SubwayStation
        {
            Index = 9,
            StationName = "Far_Station",
            WorldPosition = new Vec3(5000f, 0f, 5000f),
            NavPosition = new Vec3(5000f, 0f, 5000f)
        };
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 120f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot,
            new[] { far }, Subway, new FootRouteOptions(), out var result));

        Assert.False(result.UsesSubway);
        Assert.True(result.Success);
    }

    [Fact]
    public void NoStations_FallsBackToDirect()
    {
        var foot = new FakeFootPathProvider()
            .AddLeg(ShortOrigin, ShortTarget, 130f);

        Assert.True(FootSubwayRoutePlanner.TryBuildRoute(
            ShortOrigin, ShortTarget, ShortOrigin, foot,
            Array.Empty<SubwayStation>(), Subway, new FootRouteOptions(), out var result));

        Assert.False(result.UsesSubway);
        Assert.Single(result.Segments);
    }

    [Fact]
    public void SubwayNetwork_NoBridge_UsesDirectToDestination()
    {
        var from = FootTestStations.DowntownBoard;
        var to = FootTestStations.IndustrialExit;
        var path = Subway.BuildDisplayPath(from, to);
        Assert.Equal(2, path.Count);
        Assert.Equal(from.NavPosition, path[0]);
        Assert.Equal(to.NavPosition, path[1]);
    }

    [Fact]
    public void CsvSubwayStationLoader_EmptyFile_ReturnsEmpty()
    {
        var empty = Path.Combine(Path.GetTempPath(), "empty_subway_" + Guid.NewGuid().ToString("N") + ".csv");
        File.WriteAllText(empty, "station_name,neighborhood,x,y,z,nav_x,nav_y,nav_z\n");
        try
        {
            Assert.Empty(CsvSubwayStationLoader.LoadFromCsv(empty));
        }
        finally
        {
            File.Delete(empty);
        }
    }

    private static float MeasureFootWalk(FootRouteResult result)
    {
        var total = 0f;
        foreach (var segment in result.Segments)
        {
            if (segment.Kind != FootRouteSegmentKind.Foot)
                continue;

            total += Geometry.RoutePolylineMetrics.FlatLength(segment.Points);
        }

        return total;
    }
}
