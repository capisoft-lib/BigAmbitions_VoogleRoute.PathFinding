using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class VehicleRouteFourRulesTests : IClassFixture<RouteGraphFixture>
{
    private readonly RouteGraph _graph;

    public VehicleRouteFourRulesTests(RouteGraphFixture fixture) => _graph = fixture.Graph;

    public static IEnumerable<object[]> ScenarioRuleCases()
    {
        foreach (var scenario in VehicleRouteScenarios.All)
        foreach (var combo in VehicleRuleCombo.AllFour)
            yield return new object[] { scenario, combo };
    }

    [Theory]
    [MemberData(nameof(ScenarioRuleCases))]
    public void VehicleRoute_AllScenarios_AllFourRules_BuildValidRoute(
        VehicleRouteScenario scenario,
        VehicleRuleCombo combo)
    {
        var expectation = scenario.GetExpectation(combo);

        Assert.True(
            TryBuild(scenario, combo, out var built),
            $"[{scenario.Id}/{combo.Id}] VehicleRoutePolyline.TryBuild failed.");

        Assert.NotNull(built.Points);
        Assert.True(built.Points.Count >= 2,
            $"[{scenario.Id}/{combo.Id}] polyline needs >= 2 points, got {built.Points.Count}.");

        Assert.True(built.GraphCostMeters > 0f,
            $"[{scenario.Id}/{combo.Id}] graph cost must be positive.");

        Assert.True(built.GraphCostMeters <= scenario.MaxCostAnyRuleMeters,
            $"[{scenario.Id}/{combo.Id}] cost {built.GraphCostMeters:F1}m exceeds scenario cap {scenario.MaxCostAnyRuleMeters:F0}m.");

        var expectedAppend = combo.PreferBuildingSideArrival
            ? VehicleRouteAppendMode.EndLaneWaypoint
            : VehicleRouteAppendMode.DestinationGps;
        Assert.Equal(expectedAppend, built.AppendMode);

        if (expectation.EndWaypoint is int endWp)
            Assert.Equal(endWp, built.Route.EndWaypoint);

        if (expectation.MaxCostMeters is float maxCost)
            Assert.True(built.GraphCostMeters <= maxCost,
                $"[{scenario.Id}/{combo.Id}] cost {built.GraphCostMeters:F1}m > max {maxCost:F0}m.");

        if (expectation.MinCostMeters is float minCost)
            Assert.True(built.GraphCostMeters >= minCost,
                $"[{scenario.Id}/{combo.Id}] cost {built.GraphCostMeters:F1}m < min {minCost:F0}m.");

        if (expectation.MaxXOnThirdStreet is float maxX)
            AssertThirdStreetLane(scenario.Id, combo.Id, built.Points, maxX,
                expectation.ThirdStreetZMin, expectation.ThirdStreetZMax);
    }

    [Fact]
    public void Third45_PreferSide_ChangesEndWaypoint_AndUturn_ChangesCost()
    {
        Assert.True(TryBuildById("third45", new VehicleRuleCombo(false, true), out var offOn));
        Assert.True(TryBuildById("third45", new VehicleRuleCombo(true, false), out var onOff));

        Assert.NotEqual(offOn.Route.EndWaypoint, onOff.Route.EndWaypoint);
        Assert.Equal(13393, offOn.Route.EndWaypoint);
        Assert.Equal(7242, onOff.Route.EndWaypoint);

        Assert.True(offOn.GraphCostMeters < onOff.GraphCostMeters,
            "U-turn at start should shorten side_off route on third45.");
    }

    [Fact]
    public void Third45_AllFourRules_ProduceDistinctRoutes_WhenOptionsDiffer()
    {
        var results = new Dictionary<string, (int EndWp, float Cost, int PolyCount)>();
        foreach (var combo in VehicleRuleCombo.AllFour)
        {
            Assert.True(TryBuildById("third45", combo, out var built));
            results[combo.Id] = (built.Route.EndWaypoint, built.GraphCostMeters, built.Points.Count);
        }

        Assert.NotEqual(results["side_off_uturn_on"].EndWp, results["side_on_uturn_off"].EndWp);
        Assert.NotEqual(results["side_off_uturn_on"].Cost, results["side_off_uturn_off"].Cost);
        Assert.NotEqual(results["side_on_uturn_on"].Cost, results["side_on_uturn_off"].Cost);
    }

    [Fact]
    public void Third45WrongHint_IgnoresEastLaneSnap()
    {
        Assert.True(TryBuildById("third45_wrong_hint", new VehicleRuleCombo(true, false), out var built));
        Assert.Equal(7242, built.Route.EndWaypoint);
        AssertThirdStreetLane("third45_wrong_hint", "side_on_uturn_off", built.Points, 222.5f, -280f, -120f);
    }

    private bool TryBuildById(string scenarioId, VehicleRuleCombo combo, out VehicleRoutePolylineResult built)
    {
        var scenario = VehicleRouteScenarios.All.First(s => s.Id == scenarioId);
        return TryBuild(scenario, combo, out built);
    }

    private bool TryBuild(
        VehicleRouteScenario scenario,
        VehicleRuleCombo combo,
        out VehicleRoutePolylineResult built)
    {
        var query = new RouteQuery
        {
            Origin = scenario.Origin,
            Destination = scenario.Destination,
            Forward = scenario.Forward,
            HasPose = scenario.HasPose,
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = -1,
            AllowUturnAtStart = combo.AllowUturnAtStart,
            PreferBuildingSideArrival = combo.PreferBuildingSideArrival,
            HasArrivalRoadHint = scenario.HasArrivalRoadHint,
            ArrivalRoadHint = scenario.ArrivalRoadHint,
        };

        return VehicleRoutePolyline.TryBuild(_graph, query, out built);
    }

    private static void AssertThirdStreetLane(
        string scenarioId,
        string comboId,
        IReadOnlyList<Vec3> polyline,
        float maxX,
        float zMin,
        float zMax)
    {
        var onThird = polyline.Where(p => p.Z > zMin && p.Z < zMax).ToList();
        if (onThird.Count == 0)
            return;

        var actualMaxX = onThird.Max(p => p.X);
        Assert.True(actualMaxX <= maxX,
            $"[{scenarioId}/{comboId}] maxX on 3rd segment={actualMaxX:F2} want <={maxX:F1} (west lane, not east ~225).");
    }
}
