using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

/// <summary>
/// BA 1.0 beta mansion entrances captured from CityManager on game build 3647.
/// Every address is exercised as both route origin and destination, with the
/// normal GPS append mode and building-side arrival enabled.
/// </summary>
public sealed class HamptonsMansionRouteTests : IClassFixture<RouteGraphFixture>
{
    private const float MaxRouteCostMeters = 10_000f;
    private const float MaxEndpointSnapMeters = 100f;

    private static readonly MansionAddress[] Mansions =
    [
        new("legacy_1", "Legacy Avenue", 1, new Vec3(-2986.300f, 0.000f, -1343.670f)),
        new("legacy_2", "Legacy Avenue", 2, new Vec3(-2992.500f, 0.000f, -1404.750f)),
        new("legacy_3", "Legacy Avenue", 3, new Vec3(-3096.500f, 0.000f, -1351.250f)),
        new("legacy_4", "Legacy Avenue", 4, new Vec3(-3097.650f, 0.000f, -1358.370f)),
        new("legacy_5", "Legacy Avenue", 5, new Vec3(-3210.060f, 0.010f, -1348.950f)),
        new("legacy_6", "Legacy Avenue", 6, new Vec3(-3174.750f, 0.000f, -1363.550f)),
        new("legacy_8", "Legacy Avenue", 8, new Vec3(-3252.000f, 0.000f, -1369.750f)),
        new("legacy_10", "Legacy Avenue", 10, new Vec3(-3334.500f, 0.000f, -1348.250f)),
        new("harbor_1", "Harbor Street", 1, new Vec3(-2941.706f, 0.090f, -1040.998f)),
        new("harbor_3", "Harbor Street", 3, new Vec3(-2913.650f, 0.010f, -1327.250f)),
        new("harbor_5", "Harbor Street", 5, new Vec3(-2913.150f, 0.000f, -1412.880f)),
        new("harbor_7", "Harbor Street", 7, new Vec3(-2886.000f, 0.000f, -1575.250f)),
        new("harbor_9", "Harbor Street", 9, new Vec3(-2900.000f, 0.000f, -1691.000f)),
        new("cottage_1", "Cottage Road", 1, new Vec3(-3032.000f, 0.000f, -1518.000f)),
        new("cottage_2", "Cottage Road", 2, new Vec3(-3001.250f, 0.000f, -1593.750f)),
        new("cottage_3", "Cottage Road", 3, new Vec3(-3073.000f, 0.000f, -1555.000f)),
        new("cottage_4", "Cottage Road", 4, new Vec3(-3054.500f, 0.000f, -1592.500f)),
        new("cottage_6", "Cottage Road", 6, new Vec3(-3200.500f, 0.000f, -1598.750f)),
    ];

    private readonly RouteGraph _graph;

    public HamptonsMansionRouteTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    public static IEnumerable<object[]> MansionDirectionCases()
    {
        foreach (var mansion in Mansions)
        {
            foreach (var fromMansion in new[] { false, true })
            foreach (var preferBuildingSide in new[] { false, true })
            {
                yield return
                [
                    mansion.Id,
                    mansion.Street,
                    mansion.Number,
                    mansion.Position.X,
                    mansion.Position.Y,
                    mansion.Position.Z,
                    fromMansion,
                    preferBuildingSide,
                ];
            }
        }
    }

    [Fact]
    public void MansionFixtures_CoverAllCapturedBetaAddresses()
    {
        Assert.Equal(18, Mansions.Length);
        Assert.Equal(18, Mansions.Select(m => $"{m.Street}|{m.Number}").Distinct().Count());

        Assert.Equal([1, 2, 3, 4, 5, 6, 8, 10], StreetNumbers("Legacy Avenue"));
        Assert.Equal([1, 3, 5, 7, 9], StreetNumbers("Harbor Street"));
        Assert.Equal([1, 2, 3, 4, 6], StreetNumbers("Cottage Road"));
    }

    [Fact]
    public void LegacyAvenue8_CulDeSacTurn_IsAuthorized()
    {
        const int inboundOut = 6732;
        const int outboundIn = 15709;

        Assert.True(_graph.HasForwardEdge(inboundOut, outboundIn));
        Assert.True(_graph.IsAuthorizedUturnEdge(inboundOut, outboundIn));
    }

    [Theory]
    [MemberData(nameof(MansionDirectionCases))]
    public void VehicleRoute_FromAndToEveryMansion_IsReachable(
        string id,
        string street,
        int number,
        float x,
        float y,
        float z,
        bool fromMansion,
        bool preferBuildingSide)
    {
        var mansion = new Vec3(x, y, z);
        var cityAnchor = _graph.GetPosition(697);
        var origin = fromMansion ? mansion : cityAnchor;
        var destination = fromMansion ? cityAnchor : mansion;
        var label = $"{id} ({number} {street})/{(fromMansion ? "from" : "to")}/" +
                    (preferBuildingSide ? "side_on" : "side_off");

        var query = new RouteQuery
        {
            Origin = origin,
            Destination = destination,
            HasPose = false,
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = -1,
            AllowUturnAtStart = false,
            PreferBuildingSideArrival = preferBuildingSide,
        };

        Assert.True(
            VehicleRoutePolyline.TryBuild(_graph, query, out var built),
            $"[{label}] route must be reachable.");

        Assert.True(built.Route.Path.Count >= 2, $"[{label}] graph path is too short.");
        Assert.True(built.Points.Count >= 2, $"[{label}] polyline is too short.");
        Assert.InRange(built.GraphCostMeters, 0.01f, MaxRouteCostMeters);

        var startGap = _graph.FlatDistance(_graph.GetPosition(built.Route.StartWaypoint), origin);
        var endGap = _graph.FlatDistance(_graph.GetPosition(built.Route.EndWaypoint), destination);
        Assert.True(startGap <= MaxEndpointSnapMeters,
            $"[{label}] start waypoint gap {startGap:F1}m exceeds {MaxEndpointSnapMeters:F0}m.");
        Assert.True(endGap <= MaxEndpointSnapMeters,
            $"[{label}] end waypoint gap {endGap:F1}m exceeds {MaxEndpointSnapMeters:F0}m.");

        var expectedAppend = preferBuildingSide
            ? VehicleRouteAppendMode.EndLaneWaypoint
            : VehicleRouteAppendMode.DestinationGps;
        Assert.Equal(expectedAppend, built.AppendMode);

        if (!preferBuildingSide)
            Assert.True(_graph.FlatDistance(built.Points[^1], destination) <= 0.1f,
                $"[{label}] GPS append must finish at the requested destination.");
    }

    private static int[] StreetNumbers(string street) =>
        Mansions.Where(m => m.Street == street).Select(m => m.Number).Order().ToArray();

    private readonly record struct MansionAddress(string Id, string Street, int Number, Vec3 Position);
}
