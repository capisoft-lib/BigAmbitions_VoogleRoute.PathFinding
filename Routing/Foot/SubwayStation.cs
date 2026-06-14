using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing.Foot;

public sealed class SubwayStation
{
    public int Index { get; init; }
    public required string StationName { get; init; }
    public string Neighborhood { get; init; } = string.Empty;
    public Vec3 WorldPosition { get; init; }
    public Vec3 NavPosition { get; init; }

    public float HorizontalDistanceTo(Vec3 world) => Vec3.FlatLength(WorldPosition, world);
}
