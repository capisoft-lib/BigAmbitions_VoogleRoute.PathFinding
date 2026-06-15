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

    private static readonly Vec3 DowntownForward = new(0f, 0f, -1f);

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
            Origin = _graph.GetPosition(516),
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
        var groundAnchors = new[] { 6277, 8469, 9578 };
        Assert.Contains(built.Route.EndWaypoint, groundAnchors);
    }

    [Fact]
    public void BridgeDeckDestination_StillEndsOnElevatedLane()
    {
        var bridgeDest = _graph.GetPosition(7319);
        Assert.True(bridgeDest.Y >= BridgeDeckMinY, "fixture: wp 7319 must be bridge deck");

        var origin = _graph.GetPosition(6847);
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
        out VehicleRoutePolylineResult built)
    {
        var query = new RouteQuery
        {
            Origin = _graph.GetPosition(516),
            Destination = destination,
            Forward = DowntownForward,
            HasPose = true,
            AllowUturnAtStart = allowUturn,
            PreferBuildingSideArrival = preferBuildingSide,
        };

        return VehicleRoutePolyline.TryBuild(_graph, query, out built);
    }
}
