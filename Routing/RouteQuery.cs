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
    /// <summary>Prefer arrival lanes on the same street side as the destination.</summary>
    public bool ForceBuildingSide { get; init; }

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
            ForcedEndWaypoint = -1
        };
    }

    private static Vec3 HeadingToVector(float headingDeg)
    {
        var rad = headingDeg * (MathF.PI / 180f);
        return new Vec3(MathF.Sin(rad), 0, MathF.Cos(rad));
    }
}
