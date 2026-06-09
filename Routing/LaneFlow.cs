using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;

namespace VoogleRoute.Pathfinding.Routing;

/// <summary>Lane direction and same-flow validation (no reversing on a lane, no cross-traffic lane changes).</summary>
public static class LaneFlow
{
    public const float SameDirectionMaxDeltaDeg = 25f;
    public const float AlongLaneMaxDeltaDeg = 95f;

    internal static bool SharesTravelDirection(IRoutingGraph graph, int from, int to) =>
        TryGetLaneForwardBearing(graph, from, out var fromBearing) &&
        TryGetLaneForwardBearing(graph, to, out var toBearing) &&
        Vec3.DeltaAngle(fromBearing, toBearing) <= SameDirectionMaxDeltaDeg;

    internal static bool SharesTravelDirection(Vec3[] positions, int[][] forwardEdges, int from, int to) =>
        TryGetLaneForwardBearing(positions, forwardEdges, from, out var fromBearing) &&
        TryGetLaneForwardBearing(positions, forwardEdges, to, out var toBearing) &&
        Vec3.DeltaAngle(fromBearing, toBearing) <= SameDirectionMaxDeltaDeg;

    internal static bool IsMotionAlongLaneFlow(IRoutingGraph graph, int from, int to)
    {
        if (!TryGetLaneForwardBearing(graph, from, out var laneBearing))
            return true;

        var travelBearing = Vec3.BearingDeg(graph.GetPosition(from), graph.GetPosition(to));
        return Vec3.DeltaAngle(travelBearing, laneBearing) <= AlongLaneMaxDeltaDeg;
    }

    internal static bool TryGetLaneForwardBearing(IRoutingGraph graph, int index, out float bearingDegrees)
    {
        bearingDegrees = 0f;
        var neighbors = graph.GetForwardNeighbors(index);
        if (neighbors.Length == 0)
            return false;

        var pos = graph.GetPosition(index);
        var found = false;
        var bestLen = -1f;

        // Prefer straight lane continuation over long synthetic turn chords at intersections.
        for (var pass = 0; pass < 2; pass++)
        {
            for (var i = 0; i < neighbors.Length; i++)
            {
                var next = neighbors[i];
                var isSynth = graph.IsSyntheticTurnEdge(index, next);
                if (pass == 0 && isSynth)
                    continue;
                if (pass == 1 && !found && !isSynth)
                    continue;

                var len = graph.FlatDistance(pos, graph.GetPosition(next));
                if (len <= bestLen)
                    continue;

                bestLen = len;
                bearingDegrees = Vec3.BearingDeg(pos, graph.GetPosition(next));
                found = true;
            }

            if (found)
                break;
        }

        return found;
    }

    internal static bool TryGetLaneForwardBearing(
        Vec3[] positions,
        int[][] forwardEdges,
        int index,
        out float bearingDegrees)
    {
        bearingDegrees = 0f;
        if (index < 0 || index >= forwardEdges.Length)
            return false;

        var edges = forwardEdges[index];
        if (edges == null || edges.Length == 0)
            return false;

        var from = positions[index];
        var bestSq = float.MaxValue;
        var found = false;

        for (var i = 0; i < edges.Length; i++)
        {
            var to = edges[i];
            if (to < 0 || to >= positions.Length)
                continue;

            var dx = positions[to].X - from.X;
            var dz = positions[to].Z - from.Z;
            var sq = dx * dx + dz * dz;
            if (sq >= bestSq)
                continue;

            bestSq = sq;
            bearingDegrees = MathF.Atan2(dx, dz) * (180f / MathF.PI);
            found = true;
        }

        return found;
    }

    public static bool IsForwardEdgeAllowed(IRoutingGraph graph, int incoming, int at, int next)
    {
        var isForward = ContainsForwardEdge(graph, at, next);
        if (graph.IsLaneChangeEdge(at, next) && !isForward)
            return SharesTravelDirection(graph, at, next);

        if (!isForward)
            return false;

        if (graph.IsSyntheticTurnEdge(at, next) || graph.IsAuthorizedUturnEdge(at, next))
        {
            if (incoming < 0)
                return true;

            var absTurn = TurnGeometry.AbsLaneTurnDegrees(graph, incoming, at, next);
            if (absTurn < TurnPenalties.UTurnBlockDegrees)
                return true;

            return graph.IsAuthorizedUturnEdge(at, next);
        }

        if (!IsMotionAlongLaneFlow(graph, at, next))
            return false;

        if (incoming < 0)
            return true;

        var turn = TurnGeometry.AbsLaneTurnDegrees(graph, incoming, at, next);
        if (turn < TurnPenalties.UTurnBlockDegrees)
            return true;

        return graph.IsAuthorizedUturnEdge(at, next);
    }

    private static bool ContainsForwardEdge(IRoutingGraph graph, int from, int to)
    {
        var edges = graph.GetForwardNeighbors(from);
        for (var i = 0; i < edges.Length; i++)
        {
            if (edges[i] == to)
                return true;
        }

        return false;
    }
}
