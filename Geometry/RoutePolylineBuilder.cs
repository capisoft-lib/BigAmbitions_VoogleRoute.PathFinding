using System.Collections.Generic;
using VoogleRoute.Pathfinding.Graph;

namespace VoogleRoute.Pathfinding.Geometry;

/// <summary>
/// Construit une polyligne monde : segments droits + courbes quadratiques sur virages synthétiques CSV.
/// Même géométrie que le tracé SVG Blazor (RouteMap).
/// </summary>
public static class RoutePolylineBuilder
{
    public const int DefaultTurnSegments = 8;
    private const float MinPointSpacingSq = 2.25f;

    public static List<Vec3> BuildPoints(
        IRoutingGraph graph,
        IReadOnlyList<int> pathIndices,
        Vec3? prependOrigin = null,
        Vec3? appendDestination = null,
        int turnSegments = DefaultTurnSegments)
    {
        if (pathIndices.Count < 2)
            return new List<Vec3>();

        var simplified = RoutePathSimplifier.Simplify(graph, pathIndices);
        var points = new List<Vec3>(simplified.Count * (turnSegments + 2));

        if (prependOrigin.HasValue)
            points.Add(prependOrigin.Value);

        for (var i = 1; i < simplified.Count; i++)
        {
            var fromIdx = simplified[i - 1];
            var toIdx = simplified[i];
            var from = graph.GetPosition(fromIdx);
            var to = graph.GetPosition(toIdx);

            if (graph.TryGetSyntheticTurnControl(fromIdx, toIdx, out var control) &&
                !graph.IsLaneChangeEdge(fromIdx, toIdx))
            {
                AppendQuadratic(points, from, control, to, turnSegments);
                continue;
            }

            if (points.Count == 0 || !NearlySame(points[^1], from))
                points.Add(from);
            points.Add(to);
        }

        if (appendDestination.HasValue)
        {
            var dest = appendDestination.Value;
            if (points.Count == 0 || Vec3.FlatDistSq(points[^1], dest) >= MinPointSpacingSq)
                points.Add(dest);
        }

        return Deduplicate(points);
    }

    private static void AppendQuadratic(List<Vec3> points, Vec3 from, Vec3 control, Vec3 to, int segments)
    {
        if (segments < 2)
            segments = 2;

        for (var s = 1; s <= segments; s++)
        {
            var t = s / (float)segments;
            var q = Vec3.Lerp(Vec3.Lerp(from, control, t), Vec3.Lerp(control, to, t), t);
            if (points.Count > 0 && NearlySame(points[^1], q))
                continue;
            points.Add(q);
        }
    }

    private static List<Vec3> Deduplicate(List<Vec3> points)
    {
        if (points.Count == 0)
            return points;

        var result = new List<Vec3>(points.Count) { points[0] };
        for (var i = 1; i < points.Count; i++)
        {
            if (Vec3.FlatDistSq(points[i], result[^1]) >= MinPointSpacingSq)
                result.Add(points[i]);
        }

        if (result.Count < 2 && points.Count >= 2)
            return new List<Vec3> { points[0], points[^1] };

        return result;
    }

    private static bool NearlySame(Vec3 a, Vec3 b) => Vec3.FlatDistSq(a, b) < MinPointSpacingSq;
}
