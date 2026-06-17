using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Routing.Foot;

namespace VoogleRoute.Pathfinding.Tests;

/// <summary>Test double: returns scripted foot legs keyed by origin→target (flat XZ, 0.5m tolerance).</summary>
internal sealed class FakeFootPathProvider : IFootPathProvider
{
    private readonly List<(Vec3 Origin, Vec3 Target, FootLegResult Leg)> _legs = new();

    internal FakeFootPathProvider AddLeg(Vec3 origin, Vec3 target, float walkMeters, bool isPartial = false)
    {
        var points = StraightLine(origin, target, walkMeters);
        _legs.Add((origin, target, new FootLegResult
        {
            Success = true,
            IsPartial = isPartial,
            Points = points
        }));
        return this;
    }

    internal static IReadOnlyList<Vec3> StraightLine(Vec3 from, Vec3 to, float lengthMeters)
    {
        var dir = Vec3.FlatDir(from, to);
        if (dir.SqrMagnitude < 0.01f)
            return new[] { from, new Vec3(from.X, from.Y, from.Z + lengthMeters) };

        var end = new Vec3(
            from.X + dir.X * lengthMeters,
            from.Y,
            from.Z + dir.Z * lengthMeters);
        return new[] { from, end };
    }

    public bool TryBuildFootLeg(
        Vec3 origin,
        Vec3 target,
        Vec3 sampleOrigin,
        FootLegPurpose purpose,
        out FootLegResult leg)
    {
        for (var i = 0; i < _legs.Count; i++)
        {
            var entry = _legs[i];
            if (Near(entry.Origin, origin) && Near(entry.Target, target))
            {
                leg = entry.Leg;
                return true;
            }
        }

        leg = new FootLegResult();
        return false;
    }

    private static bool Near(Vec3 a, Vec3 b) => Vec3.FlatDistSq(a, b) < 0.25f;
}

internal static class FootTestStations
{
    internal static SubwayStation DowntownBoard => new()
    {
        Index = 0,
        StationName = "Test_Downtown",
        Neighborhood = "ba:neighborhood_downtown",
        WorldPosition = new Vec3(100f, 0f, 100f),
        NavPosition = new Vec3(100f, 0f, 100f)
    };

    internal static SubwayStation IndustrialExit => new()
    {
        Index = 1,
        StationName = "Test_Industrial",
        Neighborhood = SubwayNetwork.IndustryCityNeighborhood,
        WorldPosition = new Vec3(900f, 0f, -900f),
        NavPosition = new Vec3(900f, 0f, -900f)
    };

    internal static SubwayStation MidTown => new()
    {
        Index = 2,
        StationName = "Test_Mid",
        Neighborhood = "ba:neighborhood_mid",
        WorldPosition = new Vec3(400f, 0f, 0f),
        NavPosition = new Vec3(400f, 0f, 0f)
    };
}
