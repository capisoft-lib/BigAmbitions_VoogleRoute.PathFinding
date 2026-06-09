using System.Collections.Generic;

namespace VoogleRoute.Pathfinding.Geometry;

/// <summary>
/// Builds a flat corridor polygon around a centerline (left edge + reversed right edge).
/// </summary>
public static class RouteCorridorBuilder
{
    public static List<Vec3> BuildPolygon(IReadOnlyList<Vec3> centerline, float halfWidthMeters)
    {
        if (centerline == null || centerline.Count < 2 || halfWidthMeters <= 0f)
            return new List<Vec3>();

        var halfWidth = halfWidthMeters;
        var count = centerline.Count;
        var left = new Vec3[count];
        var right = new Vec3[count];

        for (var i = 0; i < count; i++)
        {
            var tangent = ResolveTangent(centerline, i);
            if (tangent.SqrMagnitude < 0.0001f)
            {
                left[i] = centerline[i];
                right[i] = centerline[i];
                continue;
            }

            var perp = new Vec3(-tangent.Z, 0f, tangent.X);
            left[i] = centerline[i] + perp * halfWidth;
            right[i] = centerline[i] - perp * halfWidth;
        }

        var polygon = new List<Vec3>(count * 2);
        for (var i = 0; i < count; i++)
            polygon.Add(left[i]);
        for (var i = count - 1; i >= 0; i--)
            polygon.Add(right[i]);

        return polygon;
    }

    private static Vec3 ResolveTangent(IReadOnlyList<Vec3> points, int index)
    {
        if (index <= 0)
            return Vec3.FlatDir(points[0], points[1]);

        if (index >= points.Count - 1)
            return Vec3.FlatDir(points[index - 1], points[index]);

        var incoming = Vec3.FlatDir(points[index - 1], points[index]);
        var outgoing = Vec3.FlatDir(points[index], points[index + 1]);
        var sum = incoming + outgoing;
        if (sum.SqrMagnitude < 0.0001f)
            return outgoing.SqrMagnitude > 0.0001f ? outgoing : incoming;

        var inv = 1f / MathF.Sqrt(sum.SqrMagnitude);
        return sum * inv;
    }
}
