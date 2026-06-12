using System;
using System.Collections.Generic;
using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;

namespace VoogleRoute.Pathfinding.Routing;

/// <summary>
/// A* partagé — même logique que TrafficWaypointPathfinder (hors verrou de route).
/// </summary>
public static class WaypointPathfinder
{
    private const float DefaultSearchRadius = 120f;
    private const int MaxStartCandidates = 4;
    private const int MaxEndCandidates = 6;
    private const int MaxAStarNodes = 32768;
    private const float CostTieEpsilon = 0.05f;
    private const float OppositeBearingDegrees = 105f;
    private const float StrictStartAlignedSearchRadius = 220f;
    /// <summary>Préférer l'ancrage sans cap si le trajet est nettement plus court (évite les tours de pâté de maisons).</summary>
    private const float RelaxedPosePreferMarginMeters = 15f;
    private const float ShortCorridorMaxMeters = 120f;
    private const float CorridorEndSearchRadius = 50f;

    public static RouteResult? FindBestRoute(IRoutingGraph graph, RouteQuery query) =>
        TryFindBestRoute(graph, query, out var result) ? result : null;

    public static RouteCompareResult CompareRouteModes(IRoutingGraph graph, RouteQuery query)
    {
        if (!TryFindBestRoute(graph, query, out var withPenalties))
            return new RouteCompareResult { Error = "Aucun chemin trouvé avec les pénalités de virage." };

        RouteResult? withoutPenalties = null;
        if (TryAStar(graph, withPenalties.StartWaypoint, withPenalties.EndWaypoint, useGraphTravelCost: false,
                out var flatPath, out var explored))
        {
            var turns = TurnAnalyzer.AnalyzePath(graph, flatPath);
            withoutPenalties = new RouteResult
            {
                Path = flatPath,
                StartWaypoint = withPenalties.StartWaypoint,
                EndWaypoint = withPenalties.EndWaypoint,
                GraphCostMeters = SumFlatEdges(graph, flatPath),
                AccessStartMeters = withPenalties.AccessStartMeters,
                AccessEndMeters = withPenalties.AccessEndMeters,
                NodesExplored = explored,
                Turns = turns,
                TurnSummary = TurnAnalyzer.Summarize(turns)
            };
        }

        return new RouteCompareResult
        {
            WithPenalties = withPenalties,
            WithoutPenalties = withoutPenalties
        };
    }

    public static bool TryFindBestRoute(IRoutingGraph graph, RouteQuery query, out RouteResult result)
    {
        if (!TryFindBestRouteSingle(graph, query, out result))
        {
            if (!query.HasPose)
                return false;

            return TryFindBestRouteSingle(
                graph,
                new RouteQuery
                {
                    Origin = query.Origin,
                    Destination = query.Destination,
                    HasPose = false,
                    Forward = default,
                    ForcedStartWaypoint = -1,
                    ForcedEndWaypoint = -1
                },
                out result);
        }

        if (!query.HasPose)
            return true;

        if (!TryFindBestRouteSingle(
                graph,
                new RouteQuery
                {
                    Origin = query.Origin,
                    Destination = query.Destination,
                    HasPose = false,
                    Forward = default,
                    ForcedStartWaypoint = -1,
                    ForcedEndWaypoint = -1
                },
                out var relaxed))
            return true;

        var poseDist = GetFlatRouteDistance(graph, result);
        var relaxedDist = GetFlatRouteDistance(graph, relaxed);
        if (relaxedDist + RelaxedPosePreferMarginMeters < poseDist)
            result = relaxed;

        return true;
    }

    private static bool TryFindBestRouteSingle(IRoutingGraph graph, RouteQuery query, out RouteResult result)
    {
        result = null!;
        var origin = query.Origin;
        var destination = query.Destination;
        var hasPose = query.HasPose;
        var forward = query.Forward;

        var startBuf = new int[12];
        var endBuf = new int[12];
        var radius = graph.FlatDistance(origin, destination) < 55f ? 55f : DefaultSearchRadius;

        int startCount;
        if (query.ForcedStartWaypoint >= 0)
        {
            startBuf[0] = query.ForcedStartWaypoint;
            startCount = 1;
        }
        else
        {
            startCount = hasPose
                ? CollectNearestAligned(graph, origin, forward, radius, startBuf)
                : graph.CollectNearest(origin, radius, startBuf);

            if (startCount == 0)
            {
                if (!graph.TryFindNearest(origin, 200f, out var fallbackStart))
                    return false;
                startBuf[0] = fallbackStart;
                startCount = 1;
            }

            if (hasPose)
            {
                startCount = FilterFlowAligned(graph, startBuf, startCount, forward);
                if (startCount == 0)
                    startCount = CollectStrictAlignedStarts(graph, origin, forward, startBuf);
                if (startCount == 0)
                    return false;

                startCount = graph.ExpandLaneCandidates(startBuf, startCount, startBuf.Length, forward);
                startCount = FilterFlowAligned(graph, startBuf, startCount, forward);
            }
            else
            {
                startCount = graph.ExpandLaneCandidates(startBuf, startCount, startBuf.Length, forward);
            }

            if (startCount == 0)
                return false;

            startCount = Math.Min(startCount, MaxStartCandidates);
        }

        int endCount;
        if (query.ForcedEndWaypoint >= 0)
        {
            endBuf[0] = query.ForcedEndWaypoint;
            endCount = 1;
        }
        else
        {
            endCount = graph.CollectNearest(destination, radius, endBuf);
            if (endCount == 0)
            {
                if (!graph.TryFindNearest(destination, 200f, out var fallbackEnd))
                    return false;
                endBuf[0] = fallbackEnd;
                endCount = 1;
            }

            endCount = graph.ExpandLaneCandidates(endBuf, endCount, endBuf.Length, default);
            endCount = TrimEndCandidates(graph, endBuf, endCount, destination, MaxEndCandidates);
            if (graph.FlatDistance(origin, destination) < ShortCorridorMaxMeters)
                endCount = EnsureShortCorridorEnd(graph, origin, destination, endBuf, endCount);
        }

        var bestCost = float.MaxValue;
        List<int>? bestPath = null;
        var bestEnd = -1;
        var bestStart = -1;
        var explored = 0;

        for (var si = 0; si < startCount; si++)
        {
            var startIdx = startBuf[si];
            for (var ei = 0; ei < endCount; ei++)
            {
                var endIdx = endBuf[ei];
                if (!TryAStar(graph, startIdx, endIdx, out var path, out var nodeCount))
                    continue;

                explored = Math.Max(explored, nodeCount);
                var cost = EstimateRouteCost(graph, origin, destination, startIdx, endIdx, path);
                if (!ShouldPreferRoute(cost, path, bestCost, bestPath))
                    continue;

                bestCost = cost;
                bestPath = path;
                bestEnd = endIdx;
                bestStart = startIdx;
            }
        }

        if (bestPath == null || bestPath.Count == 0)
            return false;

        var turns = TurnAnalyzer.AnalyzePath(graph, bestPath);
        result = new RouteResult
        {
            Path = bestPath,
            StartWaypoint = bestStart,
            EndWaypoint = bestEnd,
            GraphCostMeters = graph.EstimatePathTravelCost(bestPath),
            AccessStartMeters = graph.FlatDistance(origin, graph.GetPosition(bestStart)),
            AccessEndMeters = graph.EstimateArrivalLegCost(bestEnd, destination),
            NodesExplored = explored,
            Turns = turns,
            TurnSummary = TurnAnalyzer.Summarize(turns)
        };
        return true;
    }

    public static int CollectNearestAligned(
        IRoutingGraph graph, Vec3 worldPos, Vec3 flatForward, float maxDistance, int[] buffer)
    {
        var count = graph.CollectNearest(worldPos, maxDistance, buffer);
        if (count <= 1 || flatForward.SqrMagnitude < 0.01f)
            return count;

        var flowCount = FilterFlowAligned(graph, buffer, count, flatForward);
        if (flowCount > 0)
            count = flowCount;

        Array.Sort(buffer, 0, count, Comparer<int>.Create((a, b) =>
            ScoreAligned(graph, a, worldPos, flatForward).CompareTo(ScoreAligned(graph, b, worldPos, flatForward))));

        return count;
    }

    public static int FilterFlowAligned(IRoutingGraph graph, int[] buffer, int count, Vec3 flatForward)
    {
        var kept = 0;
        for (var i = 0; i < count; i++)
        {
            if (!IsFlowAlignedWithHeading(graph, buffer[i], flatForward))
                continue;
            buffer[kept++] = buffer[i];
        }

        return kept;
    }

    private static bool TryAStar(
        IRoutingGraph graph, int start, int goal, out List<int> path, out int explored) =>
        TryAStar(graph, start, goal, useGraphTravelCost: true, out path, out explored);

    private static bool TryAStar(
        IRoutingGraph graph, int start, int goal, bool useGraphTravelCost, out List<int> path, out int explored)
    {
        path = new List<int>();
        explored = 0;

        var open = new List<int> { start };
        var openSet = new HashSet<int> { start };
        var cameFrom = new Dictionary<int, int>();
        var gScore = new Dictionary<int, float> { [start] = 0f };
        var fScore = new Dictionary<int, float> { [start] = Heuristic(graph, start, goal) };
        var closed = new HashSet<int>();

        while (open.Count > 0)
        {
            if (++explored > MaxAStarNodes)
                return false;

            var current = PopLowestF(open, openSet, fScore);
            if (current == goal)
            {
                Reconstruct(cameFrom, current, path);
                return path.Count >= 1;
            }

            closed.Add(current);
            var incoming = cameFrom.TryGetValue(current, out var prev) ? prev : -1;
            var gCurrent = gScore.TryGetValue(current, out var gc) ? gc : float.MaxValue;

            RelaxNeighbors(graph, current, incoming, gCurrent, goal, useGraphTravelCost, graph.GetForwardNeighbors(current),
                closed, open, openSet, cameFrom, gScore, fScore);
            RelaxNeighbors(graph, current, incoming, gCurrent, goal, useGraphTravelCost, graph.GetLaneChangeNeighbors(current),
                closed, open, openSet, cameFrom, gScore, fScore);
        }

        return false;
    }

    private static void RelaxNeighbors(
        IRoutingGraph graph,
        int current,
        int incoming,
        float gCurrent,
        int goal,
        bool useGraphTravelCost,
        ReadOnlySpan<int> neighbors,
        HashSet<int> closed,
        List<int> open,
        HashSet<int> openSet,
        Dictionary<int, int> cameFrom,
        Dictionary<int, float> gScore,
        Dictionary<int, float> fScore)
    {
        for (var i = 0; i < neighbors.Length; i++)
        {
            var next = neighbors[i];
            if (closed.Contains(next))
                continue;
            if (!graph.IsForwardEdgeAllowed(incoming, current, next))
                continue;

            var step = useGraphTravelCost
                ? graph.GetForwardTravelCost(current, next, incoming)
                : graph.FlatDistance(graph.GetPosition(current), graph.GetPosition(next));
            var tentative = gCurrent + step;
            if (gScore.TryGetValue(next, out var existing) && tentative >= existing)
                continue;

            cameFrom[next] = current;
            gScore[next] = tentative;
            fScore[next] = tentative + Heuristic(graph, next, goal);
            if (openSet.Add(next))
                open.Add(next);
        }
    }

    private static int PopLowestF(List<int> open, HashSet<int> openSet, Dictionary<int, float> fScore)
    {
        var best = 0;
        var bestF = float.MaxValue;
        for (var i = 0; i < open.Count; i++)
        {
            var idx = open[i];
            var f = fScore.TryGetValue(idx, out var fv) ? fv : float.MaxValue;
            var tie = MathF.Abs(f - bestF) <= 0.001f;
            if (f < bestF - 0.001f || (tie && idx < open[best]))
            {
                bestF = f;
                best = i;
            }
        }

        var node = open[best];
        open.RemoveAt(best);
        openSet.Remove(node);
        return node;
    }

    private static float Heuristic(IRoutingGraph graph, int from, int to) =>
        graph.FlatDistance(graph.GetPosition(from), graph.GetPosition(to));

    private static void Reconstruct(Dictionary<int, int> cameFrom, int current, List<int> path)
    {
        path.Add(current);
        while (cameFrom.TryGetValue(current, out var prev))
        {
            current = prev;
            path.Add(current);
        }

        path.Reverse();
    }

    private static float EstimateRouteCost(
        IRoutingGraph graph, Vec3 origin, Vec3 destination, int startIdx, int endIdx, List<int> path) =>
        graph.FlatDistance(origin, graph.GetPosition(startIdx)) +
        graph.EstimatePathTravelCost(path) +
        graph.EstimateArrivalLegCost(endIdx, destination);

    private static bool ShouldPreferRoute(
        float cost,
        List<int> candidate,
        float bestCost,
        List<int>? best)
    {
        if (best == null)
            return true;

        if (cost < bestCost - CostTieEpsilon)
            return true;
        if (MathF.Abs(cost - bestCost) > CostTieEpsilon)
            return false;

        if (candidate.Count != best.Count)
            return candidate.Count < best.Count;

        for (var i = 0; i < candidate.Count; i++)
        {
            if (candidate[i] == best[i])
                continue;
            return candidate[i] < best[i];
        }

        return false;
    }

    private static int CollectStrictAlignedStarts(IRoutingGraph graph, Vec3 origin, Vec3 forward, int[] buffer)
    {
        var count = CollectNearestAligned(graph, origin, forward, StrictStartAlignedSearchRadius, buffer);
        if (count <= 0)
            return 0;

        count = FilterFlowAligned(graph, buffer, count, forward);
        if (count <= 1)
            return count;

        Array.Sort(buffer, 0, count, Comparer<int>.Create((a, b) =>
        {
            var alignedA = IsFlowAlignedWithHeading(graph, a, forward) ? 0 : 1;
            var alignedB = IsFlowAlignedWithHeading(graph, b, forward) ? 0 : 1;
            return alignedA != alignedB ? alignedA.CompareTo(alignedB) : a.CompareTo(b);
        }));

        return count;
    }

    private static bool IsFlowAlignedWithHeading(IRoutingGraph graph, int listIndex, Vec3 flatForward)
    {
        if (flatForward.SqrMagnitude < 0.01f)
            return true;
        if (!TryGetForwardBearing(graph, listIndex, out var travelBearing))
            return true;

        var heading = MathF.Atan2(flatForward.X, flatForward.Z) * (180f / MathF.PI);
        return Vec3.DeltaAngle(travelBearing, heading) < OppositeBearingDegrees;
    }

    private static bool TryGetForwardBearing(IRoutingGraph graph, int listIndex, out float bearing)
    {
        bearing = 0f;
        var neighbors = graph.GetForwardNeighbors(listIndex);
        if (neighbors.Length == 0)
            return false;

        var best = neighbors[0];
        var bestLen = -1f;
        var pos = graph.GetPosition(listIndex);
        for (var i = 0; i < neighbors.Length; i++)
        {
            var n = neighbors[i];
            var len = graph.FlatDistance(pos, graph.GetPosition(n));
            if (len > bestLen)
            {
                bestLen = len;
                best = n;
            }
        }

        bearing = Vec3.BearingDeg(pos, graph.GetPosition(best));
        return true;
    }

    private static float ScoreAligned(IRoutingGraph graph, int listIndex, Vec3 worldPos, Vec3 flatForward)
    {
        var pos = graph.GetPosition(listIndex);
        var dx = pos.X - worldPos.X;
        var dz = pos.Z - worldPos.Z;
        var distSq = dx * dx + dz * dz;
        var align = 0f;
        if (distSq > 0.25f)
        {
            var len = MathF.Sqrt(distSq);
            align = flatForward.X * (dx / len) + flatForward.Z * (dz / len);
        }

        align = MathF.Max(align, OutgoingTravelAlign(graph, listIndex, flatForward));

        var lateral = dx * flatForward.Z + dz * -flatForward.X;
        var lateralPenalty = lateral < -4f
            ? 2500f
            : -Clamp(lateral, -1.5f, 12f) * 40f;

        return distSq - align * 160f + lateralPenalty;
    }

    private static float OutgoingTravelAlign(IRoutingGraph graph, int listIndex, Vec3 heading)
    {
        var pos = graph.GetPosition(listIndex);
        var headingDeg = MathF.Atan2(heading.X, heading.Z) * (180f / MathF.PI);
        var best = -2f;
        var neighbors = graph.GetForwardNeighbors(listIndex);
        for (var i = 0; i < neighbors.Length; i++)
        {
            var b = Vec3.BearingDeg(pos, graph.GetPosition(neighbors[i]));
            var dot = MathF.Cos((b - headingDeg) * (MathF.PI / 180f));
            if (dot > best)
                best = dot;
        }

        return best;
    }

    private static int EnsureShortCorridorEnd(
        IRoutingGraph graph,
        Vec3 origin,
        Vec3 destination,
        int[] buffer,
        int count)
    {
        var probe = SelectShortCorridorProbe(origin, destination);
        if (!graph.TryFindNearest(probe, CorridorEndSearchRadius, out var corridorEnd))
            return count;

        for (var i = 0; i < count; i++)
        {
            if (buffer[i] == corridorEnd)
                return count;
        }

        if (count < MaxEndCandidates)
            buffer[count++] = corridorEnd;
        else
            buffer[MaxEndCandidates - 1] = corridorEnd;

        return Math.Min(count, MaxEndCandidates);
    }

    private static Vec3 SelectShortCorridorProbe(Vec3 origin, Vec3 destination)
    {
        var dx = MathF.Abs(destination.X - origin.X);
        var dz = MathF.Abs(destination.Z - origin.Z);
        return dx >= dz
            ? new Vec3(destination.X, origin.Y, origin.Z)
            : new Vec3(origin.X, origin.Y, destination.Z);
    }

    private static int TryAppendEndCandidate(IRoutingGraph graph, int[] buffer, int count, Vec3 probe)
    {
        if (count >= buffer.Length)
            return count;
        if (!graph.TryFindNearest(probe, CorridorEndSearchRadius, out var idx))
            return count;

        for (var i = 0; i < count; i++)
        {
            if (buffer[i] == idx)
                return count;
        }

        buffer[count++] = idx;
        return count;
    }

    private static int TrimEndCandidates(IRoutingGraph graph, int[] buffer, int count, Vec3 destination, int maxCount)
    {
        if (count <= maxCount)
            return count;

        var limit = Math.Min(count, maxCount);
        for (var i = 0; i < limit; i++)
        {
            var best = i;
            var bestDist = graph.FlatDistance(graph.GetPosition(buffer[i]), destination);
            for (var j = i + 1; j < count; j++)
            {
                var dist = graph.FlatDistance(graph.GetPosition(buffer[j]), destination);
                if (dist >= bestDist)
                    continue;
                bestDist = dist;
                best = j;
            }

            if (best == i)
                continue;

            var swap = buffer[i];
            buffer[i] = buffer[best];
            buffer[best] = swap;
        }

        return limit;
    }

    private static float Clamp(float value, float min, float max) =>
        value < min ? min : value > max ? max : value;

    public static float GetFlatRouteDistance(IRoutingGraph graph, RouteResult route) =>
        route.AccessStartMeters + SumFlatEdges(graph, route.Path) + route.AccessEndMeters;

    private static float SumFlatEdges(IRoutingGraph graph, IReadOnlyList<int> path)
    {
        if (path.Count < 2)
            return 0f;

        var sum = 0f;
        for (var i = 1; i < path.Count; i++)
            sum += graph.FlatDistance(graph.GetPosition(path[i - 1]), graph.GetPosition(path[i]));
        return sum;
    }
}
