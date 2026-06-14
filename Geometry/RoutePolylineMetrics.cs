using System.Collections.Generic;

namespace VoogleRoute.Pathfinding.Geometry;

public static class RoutePolylineMetrics
{
    public static float FlatLength(IReadOnlyList<Vec3> points)
    {
        if (points == null || points.Count < 2)
            return 0f;

        var sum = 0f;
        for (var i = 1; i < points.Count; i++)
            sum += Vec3.FlatLength(points[i - 1], points[i]);

        return sum;
    }
}
