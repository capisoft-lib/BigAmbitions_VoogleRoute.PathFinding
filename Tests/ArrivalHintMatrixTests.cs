using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class ArrivalHintMatrixTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public ArrivalHintMatrixTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    private static readonly Vec3 Origin = new(221.20f, 0.01f, -256.45f);
    private static readonly Vec3 Dest = new(214.21f, 0.09f, -136.95f);
    private static readonly Vec3 Forward = new(0f, 0f, -1f);
    private static readonly Vec3 WrongEastHint = new(225.40f, 0.01f, -138.82f);
    private static readonly Vec3 CorrectWestHint = new(221.40f, 0.01f, -138.82f);

    [Fact]
    public void WrongEastHint_SideOn_DoesNotForceEastLane()
    {
        Assert.True(TryBuild(true, false, true, WrongEastHint, out var built));
        Assert.Equal(9710, built.Route.EndWaypoint);
        AssertWestThirdStreet(built.Points);
    }

    [Fact]
    public void CorrectWestHint_SideOn_KeepsWestLane()
    {
        Assert.True(TryBuild(true, false, true, CorrectWestHint, out var built));
        Assert.Equal(9710, built.Route.EndWaypoint);
        AssertWestThirdStreet(built.Points);
    }

    [Fact]
    public void HintIgnored_WhenPreferSideOff()
    {
        Assert.True(TryBuild(false, true, true, WrongEastHint, out var built));
        Assert.Equal(17916, built.Route.EndWaypoint);
    }

    [Fact]
    public void NoHint_SideOn_MatchesBaseline()
    {
        Assert.True(TryBuild(true, false, false, default, out var built));
        Assert.Equal(9710, built.Route.EndWaypoint);
    }

    private bool TryBuild(
        bool preferSide,
        bool allowUturn,
        bool hasHint,
        Vec3 hint,
        out VehicleRoutePolylineResult built)
    {
        var query = new RouteQuery
        {
            Origin = Origin,
            Destination = Dest,
            Forward = Forward,
            HasPose = true,
            AllowUturnAtStart = allowUturn,
            PreferBuildingSideArrival = preferSide,
            HasArrivalRoadHint = hasHint,
            ArrivalRoadHint = hint,
        };

        return VehicleRoutePolyline.TryBuild(_graph, query, out built);
    }

    private static void AssertWestThirdStreet(IReadOnlyList<Vec3> points)
    {
        var onThird = points.Where(p => p.Z > -280f && p.Z < -120f).ToList();
        if (onThird.Count == 0)
            return;

        Assert.True(onThird.Max(p => p.X) <= 222.5f);
    }
}
