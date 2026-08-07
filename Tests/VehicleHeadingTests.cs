using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class VehicleHeadingTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public VehicleHeadingTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    private static readonly Vec3 Third45Origin = new(220.98f, 0.01f, -235.04f);
    private static readonly Vec3 Third45Dest = new(214.21f, 0.09f, -136.95f);

    [Theory]
    [InlineData(0)]
    [InlineData(90)]
    [InlineData(180)]
    [InlineData(270)]
    public void Third45_SideOnUturnOff_EndWaypointStableAcrossHeadings(int headingDeg)
    {
        var forward = HeadingToForward(headingDeg);
        var query = new RouteQuery
        {
            Origin = Third45Origin,
            Destination = Third45Dest,
            Forward = forward,
            HasPose = true,
            AllowUturnAtStart = false,
            PreferBuildingSideArrival = true,
        };

        Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out var built),
            $"heading={headingDeg}° must produce a route.");
        Assert.Equal(9710, built.Route.EndWaypoint);
        AssertThirdStreetWestLane(built.Points);
    }

    [Fact]
    public void FourthStreet_LogPose_South_FindsRoute()
    {
        // DiagRunner RunLogRepro: 4th_14_log
        var origin = new Vec3(174.44f, 0.46f, -25.72f);
        var dest = new Vec3(255.54f, 0.09f, -6.44f);
        var query = new RouteQuery
        {
            Origin = origin,
            Destination = dest,
            Forward = new Vec3(0f, 0f, -1f),
            HasPose = true,
            AllowUturnAtStart = true,
            PreferBuildingSideArrival = false,
        };

        Assert.True(VehicleRoutePolyline.TryBuild(_graph, query, out var built));
        Assert.True(built.GraphCostMeters > 0f);
        Assert.True(built.Points.Count >= 2);
    }

    private static Vec3 HeadingToForward(float headingDeg)
    {
        var rad = headingDeg * (MathF.PI / 180f);
        return new Vec3(MathF.Sin(rad), 0f, MathF.Cos(rad));
    }

    private static void AssertThirdStreetWestLane(IReadOnlyList<Vec3> polyline)
    {
        var onThird = polyline.Where(p => p.Z > -280f && p.Z < -120f).ToList();
        if (onThird.Count == 0)
            return;

        Assert.True(onThird.Max(p => p.X) <= 222.5f);
    }
}
