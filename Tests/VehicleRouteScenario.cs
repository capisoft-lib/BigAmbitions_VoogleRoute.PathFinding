using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Tests;

/// <summary>One of the four vehicle routing option combinations.</summary>
public readonly record struct VehicleRuleCombo(bool PreferBuildingSideArrival, bool AllowUturnAtStart)
{
    public string Id => PreferBuildingSideArrival
        ? (AllowUturnAtStart ? "side_on_uturn_on" : "side_on_uturn_off")
        : (AllowUturnAtStart ? "side_off_uturn_on" : "side_off_uturn_off");

    public static IEnumerable<VehicleRuleCombo> AllFour { get; } =
    [
        new(false, false),
        new(false, true),
        new(true, false),
        new(true, true),
    ];
}

/// <summary>Per-combo golden or bound assertions. Null fields are not checked.</summary>
public sealed class VehicleRuleExpectation
{
    public int? EndWaypoint { get; init; }
    public float? MaxCostMeters { get; init; }
    public float? MinCostMeters { get; init; }

    /// <summary>When set, polyline points with Z in (ThirdStreetZMin, ThirdStreetZMax) must have X below this.</summary>
    public float? MaxXOnThirdStreet { get; init; }
    public float ThirdStreetZMin { get; init; } = -280f;
    public float ThirdStreetZMax { get; init; } = -120f;
}

/// <summary>World-space vehicle probe: origin, destination, heading.</summary>
public sealed class VehicleRouteScenario
{
    public required string Id { get; init; }
    public required Vec3 Origin { get; init; }
    public required Vec3 Destination { get; init; }
    public required Vec3 Forward { get; init; }
    public bool HasPose { get; init; } = true;
    public bool HasArrivalRoadHint { get; init; }
    public Vec3 ArrivalRoadHint { get; init; }
    public float MaxCostAnyRuleMeters { get; init; } = 15_000f;

    /// <summary>Keyed by rule combo id (side_on_uturn_off, …).</summary>
    public IReadOnlyDictionary<string, VehicleRuleExpectation> Expectations { get; init; }
        = new Dictionary<string, VehicleRuleExpectation>();

    public VehicleRuleExpectation GetExpectation(VehicleRuleCombo combo) =>
        Expectations.TryGetValue(combo.Id, out var e) ? e : new VehicleRuleExpectation();
}
