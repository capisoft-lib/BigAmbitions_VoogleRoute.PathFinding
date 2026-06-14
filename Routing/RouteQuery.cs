using System;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing;

public readonly struct RouteQuery
{
    public Vec3 Origin { get; init; }
    public Vec3 Destination { get; init; }
    public Vec3 Forward { get; init; }
    public bool HasPose { get; init; }
    public int ForcedStartWaypoint { get; init; }
    public int ForcedEndWaypoint { get; init; }

    /// <summary>When false, the first graph step cannot use CSV U-turn connectors (parallel-lane reversals).</summary>
    public bool AllowUturnAtStart { get; init; }

    /// <summary>Prefer ending on the lane closest to the building (correct street side).</summary>
    public bool PreferBuildingSideArrival { get; init; }

    /// <summary>Game/navmesh snap on the building-side lane (from mod layer). Falls back to Destination.</summary>
    public bool HasArrivalRoadHint { get; init; }

    public Vec3 ArrivalRoadHint { get; init; }

    public static RouteQuery FromWorldCoords(float startX, float startZ, float headingDeg, float destX, float destZ)
    {
        var hasPose = !float.IsNaN(headingDeg);
        var forward = hasPose ? HeadingToVector(headingDeg) : default;
        return new RouteQuery
        {
            Origin = new Vec3(startX, 0, startZ),
            Destination = new Vec3(destX, 0, destZ),
            Forward = forward,
            HasPose = hasPose,
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = -1,
            AllowUturnAtStart = true
        };
    }

    private static Vec3 HeadingToVector(float headingDeg)
    {
        var rad = headingDeg * (MathF.PI / 180f);
        return new Vec3(MathF.Sin(rad), 0, MathF.Cos(rad));
    }
}
