using System;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing.Foot;

/// <summary>Subway travel display paths (complete graph + optional Manhattan bridge connectors).</summary>
public sealed class SubwayNetwork
{
    public const string IndustryCityNeighborhood = "ba:neighborhood_industriacity";

    private Vec3[] _bridgeLmToIc = Array.Empty<Vec3>();
    private Vec3[] _bridgeIcToLm = Array.Empty<Vec3>();

    public void SetBridgePaths(IReadOnlyList<Vec3> lmToIc, IReadOnlyList<Vec3> icToLm)
    {
        _bridgeLmToIc = CopyVec3Array(lmToIc);
        _bridgeIcToLm = CopyVec3Array(icToLm);
    }

    public IReadOnlyList<Vec3> BuildTravelPoints(SubwayStation from, SubwayStation to)
    {
        if (from.Index == to.Index)
            return new[] { from.NavPosition };

        var destination = to.NavPosition;

        if (CrossesManhattanBridge(from.Neighborhood, to.Neighborhood))
        {
            if (from.Neighborhood == IndustryCityNeighborhood && _bridgeIcToLm.Length >= 2)
            {
                return new[]
                {
                    _bridgeIcToLm[0],
                    _bridgeIcToLm[1],
                    destination
                };
            }

            if (_bridgeLmToIc.Length >= 2)
            {
                return new[]
                {
                    _bridgeLmToIc[0],
                    _bridgeLmToIc[1],
                    destination
                };
            }
        }

        return new[] { destination };
    }

    public IReadOnlyList<Vec3> BuildDisplayPath(SubwayStation from, SubwayStation to)
    {
        var travel = BuildTravelPoints(from, to);
        if (travel.Count == 0)
            return Array.Empty<Vec3>();

        var points = new Vec3[travel.Count + 1];
        points[0] = from.NavPosition;
        for (var i = 0; i < travel.Count; i++)
            points[i + 1] = travel[i];

        return points;
    }

    public static bool CrossesManhattanBridge(string fromNeighborhood, string toNeighborhood)
    {
        if (string.IsNullOrEmpty(fromNeighborhood) || string.IsNullOrEmpty(toNeighborhood))
            return false;

        if (toNeighborhood == IndustryCityNeighborhood && fromNeighborhood != IndustryCityNeighborhood)
            return true;

        return toNeighborhood != IndustryCityNeighborhood &&
               fromNeighborhood == IndustryCityNeighborhood;
    }

    private static Vec3[] CopyVec3Array(IReadOnlyList<Vec3> source)
    {
        if (source.Count == 0)
            return Array.Empty<Vec3>();

        var copy = new Vec3[source.Count];
        for (var i = 0; i < source.Count; i++)
            copy[i] = source[i];

        return copy;
    }
}
