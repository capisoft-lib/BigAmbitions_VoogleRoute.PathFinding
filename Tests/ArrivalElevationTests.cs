using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

/// <summary>
/// Ground addresses under bridge decks must route to street-level lanes, not stacked deck waypoints.
/// Repro: 21 11th Street (Road 129) under bridge cross roads 1705/1706 (~Y 12–16 m).
/// </summary>
public class ArrivalElevationTests : IClassFixture<RouteGraphFixture>
{
    private const float StreetLevelMaxY = 5f;
    private const float BridgeDeckMinY = 8f;

    private readonly RouteGraph _graph;

    public ArrivalElevationTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    /// <summary>Building entrance height near 21 11th Street (stacked under bridge deck).</summary>
    private static readonly Vec3 EleventhStreetBuilding = new(-475.5f, 1.0f, -294.5f);

    /// <summary>Building entrance height near 26 11th Street (stacked under bridge deck further north).</summary>
    private static readonly Vec3 EleventhStreet26Building = new(-475.5f, 0.01f, -273.5f);

    /// <summary>In-game door/POI height from DestinationResolver (~0.01 m).</summary>
    private static readonly Vec3 EleventhStreetBuildingInGame = new(-475.5f, 0.01f, -294.5f);

    /// <summary>Typical vehicle origin on 11th St between 4th and 5th (screenshot repro).</summary>
    private static readonly Vec3 EleventhStreetOrigin = new(-520f, 0.01f, -280f);

    private static readonly Vec3 NorthOnEleventh = new(0f, 0f, 1f);

    private static readonly Vec3 DowntownForward = new(0f, 0f, -1f);

    private static readonly int[] GroundAnchorsRoad129 = { 8383, 11372, 12875 };

    [Theory]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void EleventhStreet_GroundDestination_EndWaypointIsStreetLevel(
        bool preferBuildingSide,
        bool allowUturn)
    {
        Assert.True(
            TryBuildRoute(preferBuildingSide, allowUturn, EleventhStreetBuilding, out var built),
            "Route to 11th Street under bridge must succeed.");

        var endPos = _graph.GetPosition(built.Route.EndWaypoint);
        Assert.True(
            endPos.Y <= StreetLevelMaxY,
            $"endWp={built.Route.EndWaypoint} at Y={endPos.Y:F2} — expected street level (Road 129), not bridge deck.");

        var last = built.Points[built.Points.Count - 1];
        Assert.True(
            last.Y <= StreetLevelMaxY,
            $"Polyline tail Y={last.Y:F2} should stay at street level.");
    }

    [Fact]
    public void EleventhStreet_GroundDestination_PreferSideOff_UsesStreetLevelEnd_WhenPoseRelaxed()
    {
        var query = new RouteQuery
        {
            Origin = _graph.GetPosition(697),
            Destination = EleventhStreetBuilding,
            HasPose = false,
            AllowUturnAtStart = true,
            PreferBuildingSideArrival = false,
        };

        Assert.True(WaypointPathfinder.TryFindBestRoute(_graph, query, out var route));

        var endPos = _graph.GetPosition(route.EndWaypoint);
        Assert.True(endPos.Y <= StreetLevelMaxY,
            $"endWp={route.EndWaypoint} Y={endPos.Y:F2} should be street level when preferSide=off.");
    }

    [Fact]
    public void EleventhStreet_GroundDestination_PreferredEndNearRoad129()
    {
        Assert.True(
            TryBuildRoute(preferBuildingSide: true, allowUturn: false, EleventhStreetBuilding, out var built),
            "Route must build.");

        // Road 129 ground lane anchors in CSV near the crossing (Y ~ 0.01).
        Assert.Contains(built.Route.EndWaypoint, GroundAnchorsRoad129);
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(true, true)]
    [InlineData(false, true)]
    public void EleventhStreet_InGameHeight_EndWaypointIsStreetLevel(
        bool preferBuildingSide,
        bool allowUturn)
    {
        Assert.True(
            TryBuildRoute(EleventhStreetOrigin, NorthOnEleventh, preferBuildingSide, allowUturn,
                EleventhStreetBuildingInGame, out var built),
            "Route to 21 11th St (Y=0.01) from 11th St origin must succeed.");

        var endPos = _graph.GetPosition(built.Route.EndWaypoint);
        Assert.True(
            endPos.Y <= StreetLevelMaxY,
            $"endWp={built.Route.EndWaypoint} at Y={endPos.Y:F2} — expected street level, not bridge deck.");

        var bridgePts = 0;
        foreach (var p in built.Points)
        {
            if (p.Y >= BridgeDeckMinY)
                bridgePts++;
        }

        Assert.True(
            bridgePts == 0,
            $"Polyline must not traverse bridge deck (found {bridgePts} elevated points).");

        var last = built.Points[built.Points.Count - 1];
        Assert.True(last.Y <= StreetLevelMaxY,
            $"Polyline tail Y={last.Y:F2} should stay at street level.");
    }

    [Fact]
    public void EleventhStreet_InGameHeight_PreferSideOn_DoesNotSwapToBridgeDeck()
    {
        Assert.True(
            TryBuildRoute(EleventhStreetOrigin, NorthOnEleventh, preferBuildingSide: true, allowUturn: false,
                EleventhStreetBuildingInGame, out var built),
            "Route must build with preferSide on and in-game entrance height.");

        Assert.Contains(built.Route.EndWaypoint, GroundAnchorsRoad129);
        Assert.True(_graph.GetPosition(built.Route.EndWaypoint).Y <= StreetLevelMaxY);
    }

    private static readonly int[] GroundAnchorsRoad129Near26 = { 5592, 9468, 8147, 8529 };

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, false)]
    [InlineData(false, true)]
    public void EleventhStreet26_InGameHeight_EndWaypointIsStreetLevel(bool preferBuildingSide, bool allowUturn)
    {
        Assert.True(
            TryBuildRoute(EleventhStreetOrigin, NorthOnEleventh, preferBuildingSide, allowUturn,
                EleventhStreet26Building, out var built),
            "Route to 26 11th St (Y=0.01) from 11th St origin must succeed.");

        var endPos = _graph.GetPosition(built.Route.EndWaypoint);
        Assert.True(
            endPos.Y <= StreetLevelMaxY,
            $"endWp={built.Route.EndWaypoint} at Y={endPos.Y:F2} — expected street level, not bridge deck.");

        var bridgePts = 0;
        foreach (var p in built.Points)
        {
            if (p.Y >= BridgeDeckMinY)
                bridgePts++;
        }

        Assert.True(
            bridgePts == 0,
            $"Polyline must not traverse bridge deck (found {bridgePts} elevated points).");

        var last = built.Points[built.Points.Count - 1];
        Assert.True(last.Y <= StreetLevelMaxY,
            $"Polyline tail Y={last.Y:F2} should stay at street level.");

        if (preferBuildingSide)
            Assert.Contains(built.Route.EndWaypoint, GroundAnchorsRoad129Near26);
    }

    [Fact]
    public void BridgeDeckDestination_StillEndsOnElevatedLane()
    {
        var bridgeDest = _graph.GetPosition(9809);
        Assert.True(bridgeDest.Y >= BridgeDeckMinY, "fixture: wp 9809 must be bridge deck");

        var origin = _graph.GetPosition(9179);
        var query = new RouteQuery
        {
            Origin = origin,
            Destination = bridgeDest,
            Forward = DowntownForward,
            HasPose = true,
            AllowUturnAtStart = true,
            PreferBuildingSideArrival = false,
        };

        Assert.True(WaypointPathfinder.TryFindBestRoute(_graph, query, out var route));

        var endPos = _graph.GetPosition(route.EndWaypoint);
        Assert.True(
            endPos.Y >= BridgeDeckMinY,
            $"Bridge destination must keep deck end (endWp={route.EndWaypoint} Y={endPos.Y:F2}).");
    }

    private bool TryBuildRoute(
        bool preferBuildingSide,
        bool allowUturn,
        Vec3 destination,
        out VehicleRoutePolylineResult built) =>
        TryBuildRoute(_graph.GetPosition(697), DowntownForward, preferBuildingSide, allowUturn, destination, out built);

    private bool TryBuildRoute(
        Vec3 origin,
        Vec3 forward,
        bool preferBuildingSide,
        bool allowUturn,
        Vec3 destination,
        out VehicleRoutePolylineResult built)
    {
        var query = new RouteQuery
        {
            Origin = origin,
            Destination = destination,
            Forward = forward,
            HasPose = true,
            AllowUturnAtStart = allowUturn,
            PreferBuildingSideArrival = preferBuildingSide,
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = -1,
        };

        return VehicleRoutePolyline.TryBuild(_graph, query, out built);
    }
}
