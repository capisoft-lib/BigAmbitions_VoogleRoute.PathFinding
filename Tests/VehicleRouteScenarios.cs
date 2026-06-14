using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;

namespace VoogleRoute.Pathfinding.Tests;

/// <summary>Fixed vehicle scenarios × four routing rules. Goldens from DiagRunner third45 (2026-06-14).</summary>
public static class VehicleRouteScenarios
{
    public static IReadOnlyList<VehicleRouteScenario> All { get; } = BuildAll();

    private static IReadOnlyList<VehicleRouteScenario> BuildAll()
    {
        // Lazy graph positions for cross-city probes (waypoint anchors).
        var graph = new RouteGraphFixture().Graph;

        return
        [
            Third45(),
            Third45InGame(),
            Third45WrongHint(),
            CrossCity("downtown_industrial", graph.GetPosition(516), graph.GetPosition(3149),
                maxCost: 5_500f, sideOnEndWp: 10382, sideOnMaxCost: 4500f),
            CrossCity("bridge_city_industrial", graph.GetPosition(6847), graph.GetPosition(3149),
                maxCost: 4_500f, sideOnEndWp: 10382, sideOnMaxCost: 4600f),
            CrossCity("ne_corner_industrial", graph.GetPosition(1133), graph.GetPosition(13382),
                maxCost: 6_500f, sideOnEndWp: 4715, sideOnMaxCost: 4900f),
            ShortUrban("4th_st_wordsmith",
                new Vec3(134f, 0.44f, 55f),
                new Vec3(145f, 0.41f, -8f),
                maxCost: 950f),
            ShortUrban("1st_st_north",
                new Vec3(131.28f, 0.44f, 121.01f),
                new Vec3(404.27f, 0.09f, 444.64f),
                maxCost: 6_000f),
        ];
    }

  private static VehicleRouteScenario Third45()
    {
        var dest = new Vec3(214.21f, 0.09f, -136.95f);
        return new VehicleRouteScenario
        {
            Id = "third45",
            Origin = new Vec3(220.98f, 0.01f, -235.04f),
            Destination = dest,
            Forward = new Vec3(0f, 0f, -1f),
            MaxCostAnyRuleMeters = 1_000f,
            Expectations = new Dictionary<string, VehicleRuleExpectation>
            {
                ["side_off_uturn_on"] = new()
                {
                    EndWaypoint = 13393,
                    MaxCostMeters = 140f,
                },
                ["side_off_uturn_off"] = new()
                {
                    EndWaypoint = 13393,
                    MaxCostMeters = 700f,
                },
                ["side_on_uturn_on"] = new()
                {
                    EndWaypoint = 7242,
                    MaxCostMeters = 750f,
                },
                ["side_on_uturn_off"] = new()
                {
                    EndWaypoint = 7242,
                    MaxCostMeters = 900f,
                    MaxXOnThirdStreet = 222.5f,
                },
            },
        };
    }

    private static VehicleRouteScenario Third45InGame()
    {
        return new VehicleRouteScenario
        {
            Id = "third45_ingame",
            Origin = new Vec3(221.36f, 0.01f, -279.60f),
            Destination = new Vec3(214.21f, 0.09f, -136.95f),
            Forward = new Vec3(0f, 0f, -1f),
            MaxCostAnyRuleMeters = 1_200f,
            Expectations = new Dictionary<string, VehicleRuleExpectation>
            {
                ["side_on_uturn_off"] = new()
                {
                    EndWaypoint = 7242,
                    MaxCostMeters = 950f,
                    MaxXOnThirdStreet = 222.5f,
                },
            },
        };
    }

    private static VehicleRouteScenario Third45WrongHint()
    {
        return new VehicleRouteScenario
        {
            Id = "third45_wrong_hint",
            Origin = new Vec3(221.20f, 0.01f, -256.45f),
            Destination = new Vec3(214.21f, 0.09f, -136.95f),
            Forward = new Vec3(0f, 0f, -1f),
            HasArrivalRoadHint = true,
            ArrivalRoadHint = new Vec3(225.40f, 0.01f, -138.82f),
            MaxCostAnyRuleMeters = 1_200f,
            Expectations = new Dictionary<string, VehicleRuleExpectation>
            {
                ["side_on_uturn_off"] = new()
                {
                    EndWaypoint = 7242,
                    MaxCostMeters = 950f,
                    MaxXOnThirdStreet = 222.5f,
                },
            },
        };
    }

    private static VehicleRouteScenario CrossCity(
        string id,
        Vec3 origin,
        Vec3 destination,
        float maxCost,
        int? sideOnEndWp,
        float? sideOnMaxCost)
    {
        var expectations = new Dictionary<string, VehicleRuleExpectation>();
        foreach (var combo in VehicleRuleCombo.AllFour)
        {
            var rule = new VehicleRuleExpectation { MaxCostMeters = maxCost };
            if (combo.PreferBuildingSideArrival)
            {
                if (sideOnEndWp is int endWp)
                    rule = new VehicleRuleExpectation { EndWaypoint = endWp, MaxCostMeters = sideOnMaxCost ?? maxCost };
            }

            expectations[combo.Id] = rule;
        }

        return new VehicleRouteScenario
        {
            Id = id,
            Origin = origin,
            Destination = destination,
            Forward = SouthHeading(origin, destination),
            MaxCostAnyRuleMeters = maxCost,
            Expectations = expectations,
        };
    }

    private static VehicleRouteScenario ShortUrban(string id, Vec3 origin, Vec3 destination, float maxCost)
    {
        var expectations = new Dictionary<string, VehicleRuleExpectation>();
        foreach (var combo in VehicleRuleCombo.AllFour)
            expectations[combo.Id] = new VehicleRuleExpectation { MaxCostMeters = maxCost };

        return new VehicleRouteScenario
        {
            Id = id,
            Origin = origin,
            Destination = destination,
            Forward = SouthHeading(origin, destination),
            MaxCostAnyRuleMeters = maxCost,
            Expectations = expectations,
        };
    }

    private static Vec3 SouthHeading(Vec3 origin, Vec3 destination)
    {
        var dx = destination.X - origin.X;
        var dz = destination.Z - origin.Z;
        if (dx * dx + dz * dz < 0.01f)
            return new Vec3(0f, 0f, -1f);

        var len = MathF.Sqrt(dx * dx + dz * dz);
        return new Vec3(dx / len, 0f, dz / len);
    }
}
