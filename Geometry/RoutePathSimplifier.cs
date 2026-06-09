using System.Collections.Generic;
using VoogleRoute.Pathfinding.Graph;

namespace VoogleRoute.Pathfinding.Geometry;

/// <summary>
/// Réduit les waypoints intermédiaires alignés sur un même segment (même logique que RouteMap Blazor).
/// </summary>
public static class RoutePathSimplifier
{
    public static List<int> Simplify(IRoutingGraph graph, IReadOnlyList<int> path, float angleThresholdDeg = 4f)
    {
        if (path.Count <= 2)
            return new List<int>(path);

        var simplified = new List<int>(path.Count) { path[0] };
        for (var i = 1; i < path.Count; i++)
        {
            var to = path[i];
            var from = simplified[^1];

            if (graph.IsLaneChangeEdge(from, to) ||
                graph.TryGetSyntheticTurnControl(from, to, out _))
            {
                simplified.Add(to);
                continue;
            }

            if (i < path.Count - 1)
            {
                var next = path[i + 1];
                if (!graph.IsLaneChangeEdge(to, next) &&
                    !graph.TryGetSyntheticTurnControl(to, next, out _) &&
                    IsCollinear(
                        graph.GetPosition(from),
                        graph.GetPosition(to),
                        graph.GetPosition(next),
                        angleThresholdDeg))
                    continue;
            }

            simplified.Add(to);
        }

        return simplified;
    }

    private static bool IsCollinear(Vec3 a, Vec3 b, Vec3 c, float angleThresholdDeg)
    {
        var abx = b.X - a.X;
        var abz = b.Z - a.Z;
        var bcx = c.X - b.X;
        var bcz = c.Z - b.Z;
        var abLen = MathF.Sqrt(abx * abx + abz * abz);
        var bcLen = MathF.Sqrt(bcx * bcx + bcz * bcz);
        if (abLen < 0.05f || bcLen < 0.05f)
            return true;

        var dot = (abx * bcx + abz * bcz) / (abLen * bcLen);
        if (dot > 1f) dot = 1f;
        else if (dot < -1f) dot = -1f;
        var angleDeg = MathF.Acos(dot) * (180f / MathF.PI);
        return angleDeg < angleThresholdDeg;
    }
}
