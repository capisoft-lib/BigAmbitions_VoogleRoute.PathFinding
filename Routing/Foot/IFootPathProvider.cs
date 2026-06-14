using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing.Foot;

/// <summary>NavMesh or test double: builds one outdoor foot leg between world positions.</summary>
public interface IFootPathProvider
{
    bool TryBuildFootLeg(Vec3 origin, Vec3 target, Vec3 sampleOrigin, out FootLegResult leg);
}
