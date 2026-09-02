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
    [ThreadStatic] private static SearchWorkspace? _searchWorkspace;

    public static RouteResult? FindBestRoute(IRoutingGraph graph, RouteQuery query) =>
        TryFindBestRoute(graph, query, out var result) ? result : null;

    public static RouteCompareResult CompareRouteModes(IRoutingGraph graph, RouteQuery query)
    {
        if (!TryFindBestRoute(graph, query, out var withPenalties))
            return new RouteCompareResult { Error = "Aucun chemin trouvé avec les pénalités de virage." };

        RouteResult? withoutPenalties = null;
        if (TryAStar(graph, withPenalties.StartWaypoint, withPenalties.EndWaypoint,
                allowUturnAtStart: true, useGraphTravelCost: false, out var flatPath, out var explored))
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
            if (!query.HasPose || !query.AllowUturnAtStart)
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
                    ForcedEndWaypoint = -1,
                    AllowUturnAtStart = query.AllowUturnAtStart,
                    PreferBuildingSideArrival = query.PreferBuildingSideArrival,
                    HasArrivalRoadHint = query.HasArrivalRoadHint,
                    ArrivalRoadHint = query.ArrivalRoadHint,
                    CancellationToken = query.CancellationToken
                },
                out result);
        }

        if (!query.HasPose || !query.AllowUturnAtStart)
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
                    ForcedEndWaypoint = -1,
                    AllowUturnAtStart = query.AllowUturnAtStart,
                    PreferBuildingSideArrival = query.PreferBuildingSideArrival,
                    HasArrivalRoadHint = query.HasArrivalRoadHint,
                    ArrivalRoadHint = query.ArrivalRoadHint,
                    CancellationToken = query.CancellationToken
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
        var building = destination;
        var hasPose = query.HasPose;
        var forward = query.Forward;

        var startBuf = new int[12];
        var endBuf = new int[12];
        var radius = graph.FlatDistance(origin, destination) < 55f ? 55f : DefaultSearchRadius;

        int startCount;
        if (query.ForcedStartWaypoint != -1)
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

                if (query.AllowUturnAtStart)
                {
                    startCount = graph.ExpandLaneCandidates(startBuf, startCount, startBuf.Length, forward);
                    startCount = FilterFlowAligned(graph, startBuf, startCount, forward);
                }
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
        if (query.ForcedEndWaypoint != -1)
        {
            endBuf[0] = query.ForcedEndWaypoint;
            endCount = 1;
        }
        else
        {
            var endSearch = destination;

            endCount = graph.CollectNearest(endSearch, radius, endBuf);
            if (endCount == 0)
            {
                if (!graph.TryFindNearest(endSearch, 200f, out var fallbackEnd))
                    return false;
                endBuf[0] = fallbackEnd;
                endCount = 1;
            }

            endCount = graph.ExpandLaneCandidates(endBuf, endCount, endBuf.Length, default);
            endCount = FilterStreetLevelEndCandidates(graph, endBuf, endCount, endSearch);
            endCount = TrimEndCandidates(graph, endBuf, endCount, endSearch, MaxEndCandidates);
            if (query.PreferBuildingSideArrival)
                endCount = FilterBuildingSideEndCandidates(graph, endBuf, endCount, building);
            if (graph.FlatDistance(origin, destination) < ShortCorridorMaxMeters)
                endCount = EnsureShortCorridorEnd(graph, origin, destination, endBuf, endCount);
        }

        if (endCount <= 0 || query.CancellationToken.IsCancellationRequested)
            return false;

        if (!TryFindCandidateRoute(
                graph,
                query,
                origin,
                startBuf,
                startCount,
                endBuf,
                endCount,
                out var bestPath,
                out var bestStart,
                out var bestEnd,
                out var explored))
            return false;

        if (bestPath == null || bestPath.Count == 0)
            return false;

        if (query.PreferBuildingSideArrival && bestEnd >= 0)
            ApplyBuildingSideEnd(graph, query, bestPath, ref bestEnd);

        var turns = TurnAnalyzer.AnalyzePath(graph, bestPath);
        result = new RouteResult
        {
            Path = bestPath,
            StartWaypoint = bestStart,
            EndWaypoint = bestEnd,
            GraphCostMeters = graph.EstimatePathTravelCost(bestPath),
            AccessStartMeters = graph.FlatDistance(origin, graph.GetPosition(bestStart)),
            AccessEndMeters = query.PreferBuildingSideArrival
                ? graph.DistanceToDestination(graph.GetPosition(bestEnd), building)
                : graph.EstimateArrivalLegCost(bestEnd, destination),
            NodesExplored = explored,
            Turns = turns,
            TurnSummary = TurnAnalyzer.Summarize(turns)
        };
        return true;
    }

    /// <summary>
    /// One reusable multi-source/multi-target A* replaces the former start×end nested searches.
    /// Start access cost is seeded into the queue; every reachable destination candidate is
    /// evaluated with the same final route scoring rules as before.
    /// </summary>
    private static bool TryFindCandidateRoute(
        IRoutingGraph graph,
        RouteQuery query,
        Vec3 origin,
        int[] starts,
        int startCount,
        int[] ends,
        int endCount,
        out List<int>? bestPath,
        out int bestStart,
        out int bestEnd,
        out int explored)
    {
        bestPath = null;
        bestStart = -1;
        bestEnd = -1;
        explored = 0;
        var bestCost = float.MaxValue;
        var search = GetSearchWorkspace();
        search.BeginSearch();
        var stateStride = startCount;

        for (var i = 0; i < startCount; i++)
        {
            var start = starts[i];
            var accessCost = graph.FlatDistance(origin, graph.GetPosition(start));
            search.TryImprove(
                EncodeCandidateState(start, i, stateStride),
                accessCost,
                cameFrom: -1,
                rootStart: start,
                accessCost + Heuristic(graph, start, ends, endCount));
        }

        while (search.TryPop(out var currentState, out var gCurrent))
        {
            if (query.CancellationToken.IsCancellationRequested)
                return false;
            if (++explored > MaxAStarNodes * stateStride)
                return false;

            var current = DecodeCandidateNode(currentState, stateStride);
            search.Close(currentState);
            if (TryGetEndCandidateIndex(ends, endCount, current, out _))
            {
                var candidate = new List<int>();
                ReconstructCandidate(search, currentState, stateStride, candidate);
                var start = search.GetRootStart(currentState);
                var cost = EstimateRouteCost(graph, origin, query, start, current, candidate);
                if (ShouldPreferRoute(cost, candidate, bestCost, bestPath))
                {
                    bestCost = cost;
                    bestPath = candidate;
                    bestStart = start;
                    bestEnd = current;
                }
            }

            var incomingState = search.GetCameFrom(currentState);
            var incoming = incomingState >= 0
                ? DecodeCandidateNode(incomingState, stateStride)
                : -1;
            RelaxCandidateNeighbors(
                graph, query, currentState, current, incoming, gCurrent, ends, endCount, stateStride,
                graph.GetForwardNeighbors(current), search);
            RelaxCandidateNeighbors(
                graph, query, currentState, current, incoming, gCurrent, ends, endCount, stateStride,
                graph.GetLaneChangeNeighbors(current), search);
        }

        return bestPath != null;
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
        TryAStar(graph, start, goal, allowUturnAtStart: true, out path, out explored);

    private static bool TryAStar(
        IRoutingGraph graph, int start, int goal, bool allowUturnAtStart, out List<int> path, out int explored) =>
        TryAStar(graph, start, goal, allowUturnAtStart, useGraphTravelCost: true, out path, out explored);

    private static bool TryAStar(
        IRoutingGraph graph,
        int start,
        int goal,
        bool allowUturnAtStart,
        RouteQuery query,
        out List<int> path,
        out int explored) =>
        TryAStar(graph, start, goal, allowUturnAtStart, useGraphTravelCost: true, query, out path, out explored);

    private static bool TryAStar(
        IRoutingGraph graph,
        int start,
        int goal,
        bool allowUturnAtStart,
        bool useGraphTravelCost,
        out List<int> path,
        out int explored) =>
        TryAStar(graph, start, goal, allowUturnAtStart, useGraphTravelCost, default, out path, out explored);

    private static bool TryAStar(
        IRoutingGraph graph,
        int start,
        int goal,
        bool allowUturnAtStart,
        bool useGraphTravelCost,
        RouteQuery query,
        out List<int> path,
        out int explored)
    {
        path = new List<int>();
        explored = 0;
        var search = GetSearchWorkspace();
        search.BeginSearch();
        search.TryImprove(start, 0f, cameFrom: -1, rootStart: start, Heuristic(graph, start, goal));

        while (search.TryPop(out var current, out var gCurrent))
        {
            if (query.CancellationToken.IsCancellationRequested)
                return false;
            if (++explored > MaxAStarNodes)
                return false;

            if (current == goal)
            {
                Reconstruct(search, current, path);
                return path.Count >= 1;
            }

            search.Close(current);
            var incoming = search.GetCameFrom(current);
            RelaxSingleGoalNeighbors(
                graph, current, incoming, gCurrent, goal, useGraphTravelCost, allowUturnAtStart, query,
                graph.GetForwardNeighbors(current), search);
            RelaxSingleGoalNeighbors(
                graph, current, incoming, gCurrent, goal, useGraphTravelCost, allowUturnAtStart, query,
                graph.GetLaneChangeNeighbors(current), search);
        }

        return false;
    }

    private static void RelaxSingleGoalNeighbors(
        IRoutingGraph graph,
        int current,
        int incoming,
        float gCurrent,
        int goal,
        bool useGraphTravelCost,
        bool allowUturnAtStart,
        RouteQuery query,
        ReadOnlySpan<int> neighbors,
        SearchWorkspace search)
    {
        for (var i = 0; i < neighbors.Length; i++)
        {
            var next = neighbors[i];
            if (search.IsClosed(next))
                continue;
            if (!graph.IsForwardEdgeAllowed(incoming, current, next))
                continue;
            if (!allowUturnAtStart && incoming < 0 &&
                StartManeuverPolicy.IsBlockedManeuverAtStart(graph, current, next))
                continue;

            var step = useGraphTravelCost
                ? graph.GetForwardTravelCost(current, next, incoming)
                : graph.FlatDistance(graph.GetPosition(current), graph.GetPosition(next));
            step += BuildingSideTravelPenalty(graph, query, next);
            var tentative = gCurrent + step;
            search.TryImprove(
                next,
                tentative,
                current,
                search.GetRootStart(current),
                tentative + Heuristic(graph, next, goal));
        }
    }

    private static void RelaxCandidateNeighbors(
        IRoutingGraph graph,
        RouteQuery query,
        int currentState,
        int current,
        int incoming,
        float gCurrent,
        int[] goals,
        int goalCount,
        int stateStride,
        ReadOnlySpan<int> neighbors,
        SearchWorkspace search)
    {
        var startSlot = currentState % stateStride;
        for (var i = 0; i < neighbors.Length; i++)
        {
            var next = neighbors[i];
            var nextState = EncodeCandidateState(next, startSlot, stateStride);
            if (search.IsClosed(nextState))
                continue;
            if (!graph.IsForwardEdgeAllowed(incoming, current, next))
                continue;
            if (!query.AllowUturnAtStart && incoming < 0 &&
                StartManeuverPolicy.IsBlockedManeuverAtStart(graph, current, next))
                continue;

            var step = graph.GetForwardTravelCost(current, next, incoming) +
                       BuildingSideTravelPenalty(graph, query, next);
            var tentative = gCurrent + step;
            search.TryImprove(
                nextState,
                tentative,
                currentState,
                search.GetRootStart(currentState),
                tentative + Heuristic(graph, next, goals, goalCount));
        }
    }

    private static float Heuristic(IRoutingGraph graph, int from, int to) =>
        graph.FlatDistance(graph.GetPosition(from), graph.GetPosition(to));

    private static float Heuristic(IRoutingGraph graph, int from, int[] goals, int goalCount)
    {
        var best = float.MaxValue;
        for (var i = 0; i < goalCount; i++)
        {
            var distance = Heuristic(graph, from, goals[i]);
            if (distance < best)
                best = distance;
        }

        return best;
    }

    private static bool TryGetEndCandidateIndex(int[] ends, int count, int node, out int candidateIndex)
    {
        for (var i = 0; i < count; i++)
        {
            if (ends[i] != node)
                continue;
            candidateIndex = i;
            return true;
        }

        candidateIndex = -1;
        return false;
    }

    private static int EncodeCandidateState(int node, int startSlot, int stateStride) =>
        checked(node * stateStride + startSlot);

    private static int DecodeCandidateNode(int state, int stateStride) => state / stateStride;

    private static void ReconstructCandidate(
        SearchWorkspace search,
        int currentState,
        int stateStride,
        List<int> path)
    {
        path.Add(DecodeCandidateNode(currentState, stateStride));
        while (search.GetCameFrom(currentState) is var previousState && previousState >= 0)
        {
            currentState = previousState;
            path.Add(DecodeCandidateNode(currentState, stateStride));
        }

        path.Reverse();
    }

    private static void Reconstruct(SearchWorkspace search, int current, List<int> path)
    {
        path.Add(current);
        while (search.GetCameFrom(current) is var previous && previous >= 0)
        {
            current = previous;
            path.Add(current);
        }

        path.Reverse();
    }

    private static float EstimateRouteCost(
        IRoutingGraph graph,
        Vec3 origin,
        RouteQuery query,
        int startIdx,
        int endIdx,
        List<int> path)
    {
        var building = query.Destination;
        var arrivalEnd = query.PreferBuildingSideArrival
            ? FindBestBuildingSideEndNearTarget(graph, endIdx, building)
            : endIdx;

        return graph.FlatDistance(origin, graph.GetPosition(startIdx)) +
               graph.EstimatePathTravelCost(path) +
               graph.DistanceToDestination(graph.GetPosition(arrivalEnd), building);
    }

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
            var bestDist = graph.DistanceToDestination(graph.GetPosition(buffer[i]), destination);
            for (var j = i + 1; j < count; j++)
            {
                var dist = graph.DistanceToDestination(graph.GetPosition(buffer[j]), destination);
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

    private const float BuildingSideLaneMarginMeters = 4.5f;
    private const float BuildingSideMinLateralMeters = 0.5f;
    private const float BuildingSideParallelLaneSeparationMeters = 3f;
    /// <summary>Perpendicular distance from destination street — avoids penalizing detour legs on other roads.</summary>
    private const float BuildingSideArrivalCrossStreetMeters = 14f;
    /// <summary>Max distance along the destination street from the building where wrong-lane penalty applies.</summary>
    private const float BuildingSideArrivalAlongStreetMeters = 110f;
    private const float BuildingSideWrongLanePenaltyMeters = 45f;
    private const float BuildingSideCrossOffsetPenaltyPerMeter = 4f;
    private const float BuildingSideCrossOffsetFreeMeters = 7f;

    /// <summary>Signed lateral offset of building to the right of lane travel (US curb side).</summary>
    private static float ScoreBuildingSideLateral(IRoutingGraph graph, int wpIdx, Vec3 building)
    {
        if (!LaneFlow.TryGetLaneForwardBearing(graph, wpIdx, out var bearingDeg))
            return float.NegativeInfinity;

        var wp = graph.GetPosition(wpIdx);
        var dx = building.X - wp.X;
        var dz = building.Z - wp.Z;
        var rad = bearingDeg * (MathF.PI / 180f);
        var fwdX = MathF.Sin(rad);
        var fwdZ = MathF.Cos(rad);
        var rightX = fwdZ;
        var rightZ = -fwdX;
        return rightX * dx + rightZ * dz;
    }

    private static int TrimToNearestBuildingLanes(
        IRoutingGraph graph,
        int[] buffer,
        int count,
        Vec3 building)
    {
        if (count <= 1)
            return count;

        var minDist = float.MaxValue;
        for (var i = 0; i < count; i++)
        {
            var dist = graph.DistanceToDestination(graph.GetPosition(buffer[i]), building);
            if (dist < minDist)
                minDist = dist;
        }

        var kept = 0;
        for (var i = 0; i < count; i++)
        {
            var dist = graph.DistanceToDestination(graph.GetPosition(buffer[i]), building);
            if (dist > minDist + BuildingSideParallelLaneSeparationMeters)
                continue;
            buffer[kept++] = buffer[i];
        }

        return kept > 0 ? kept : count;
    }

    /// <summary>Extra A* step cost near destination when a parallel lane is closer to the building.</summary>
    private static float BuildingSideTravelPenalty(IRoutingGraph graph, RouteQuery query, int wpIdx)
    {
        if (!query.PreferBuildingSideArrival)
            return 0f;

        var building = query.Destination;
        var wp = graph.GetPosition(wpIdx);
        var cross = CrossStreetOffset(graph, wpIdx, wp, building);
        if (cross > BuildingSideArrivalCrossStreetMeters)
            return 0f;

        var flat = graph.FlatDistance(wp, building);
        var along = MathF.Sqrt(MathF.Max(0f, flat * flat - cross * cross));
        if (along > BuildingSideArrivalAlongStreetMeters)
            return 0f;

        var lateral = ScoreBuildingSideLateral(graph, wpIdx, building);
        if (lateral <= BuildingSideMinLateralMeters)
            return BuildingSideWrongLanePenaltyMeters;

        if (cross <= BuildingSideCrossOffsetFreeMeters)
            return 0f;

        return (cross - BuildingSideCrossOffsetFreeMeters) * BuildingSideCrossOffsetPenaltyPerMeter;
    }

    /// <summary>Perpendicular distance from waypoint to building along the cross-street axis.</summary>
    private static float CrossStreetOffset(IRoutingGraph graph, int wpIdx, Vec3 wp, Vec3 building)
    {
        if (!LaneFlow.TryGetLaneForwardBearing(graph, wpIdx, out var bearingDeg))
            return MathF.Min(MathF.Abs(wp.X - building.X), MathF.Abs(wp.Z - building.Z));

        var rad = bearingDeg * (MathF.PI / 180f);
        var fwdX = MathF.Sin(rad);
        var fwdZ = MathF.Cos(rad);
        var dx = building.X - wp.X;
        var dz = building.Z - wp.Z;
        return MathF.Abs(fwdX * dz - fwdZ * dx);
    }

    private static int SelectBuildingSideCandidate(
        IRoutingGraph graph,
        int[] buffer,
        int count,
        Vec3 building)
    {
        if (count <= 0)
            return -1;

        if (count == 1)
            return buffer[0];

        var bestIdx = -1;
        var bestDist = float.MaxValue;

        for (var i = 0; i < count; i++)
        {
            var idx = buffer[i];
            var lateral = ScoreBuildingSideLateral(graph, idx, building);
            if (lateral <= BuildingSideMinLateralMeters)
                continue;

            var dist = graph.DistanceToDestination(graph.GetPosition(idx), building);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestIdx >= 0)
            return bestIdx;

        bestIdx = buffer[0];
        bestDist = graph.DistanceToDestination(graph.GetPosition(bestIdx), building);
        for (var i = 1; i < count; i++)
        {
            var dist = graph.DistanceToDestination(graph.GetPosition(buffer[i]), building);
            if (dist >= bestDist)
                continue;

            bestDist = dist;
            bestIdx = buffer[i];
        }

        return bestIdx;
    }

    /// <summary>Drop bridge-deck ends when the building target is at street level (stacked roads).</summary>
    private static int FilterStreetLevelEndCandidates(
        IRoutingGraph graph,
        int[] buffer,
        int count,
        Vec3 destination)
    {
        if (!RouteGraph.IsStreetLevelDestination(destination.Y))
            return count;

        var kept = 0;
        for (var i = 0; i < count; i++)
        {
            if (RouteGraph.IsElevatedWaypoint(graph.GetPosition(buffer[i]).Y))
                continue;
            buffer[kept++] = buffer[i];
        }

        return kept;
    }

    /// <summary>Drop opposite-lane end candidates; prefer curb-side lane when geometry allows.</summary>
    private static int FilterBuildingSideEndCandidates(
        IRoutingGraph graph,
        int[] buffer,
        int count,
        Vec3 destination)
    {
        if (count <= 1)
            return count;

        var curbCount = 0;
        for (var i = 0; i < count; i++)
        {
            if (ScoreBuildingSideLateral(graph, buffer[i], destination) > BuildingSideMinLateralMeters)
                buffer[curbCount++] = buffer[i];
        }

        if (curbCount > 0)
            return TrimToNearestBuildingLanes(graph, buffer, curbCount, destination);

        var minDist = float.MaxValue;
        for (var i = 0; i < count; i++)
        {
            var dist = graph.DistanceToDestination(graph.GetPosition(buffer[i]), destination);
            if (dist < minDist)
                minDist = dist;
        }

        var kept = 0;
        for (var i = 0; i < count; i++)
        {
            var dist = graph.DistanceToDestination(graph.GetPosition(buffer[i]), destination);
            if (dist > minDist + BuildingSideLaneMarginMeters)
                continue;
            buffer[kept++] = buffer[i];
        }

        return kept > 0 ? kept : count;
    }

    private static int FindBestBuildingSideEndNearTarget(
        IRoutingGraph graph,
        int fallbackEnd,
        Vec3 arrivalTarget)
    {
        const float searchRadius = 90f;
        var buffer = new int[24];
        var count = graph.CollectNearest(arrivalTarget, searchRadius, buffer);
        if (count == 0)
            return ResolveBuildingSideEndWaypoint(graph, fallbackEnd, arrivalTarget);

        count = graph.ExpandLaneCandidates(buffer, count, buffer.Length, default);
        count = FilterStreetLevelEndCandidates(graph, buffer, count, arrivalTarget);
        count = FilterBuildingSideEndCandidates(graph, buffer, count, arrivalTarget);
        if (count <= 0)
            return ResolveBuildingSideEndWaypoint(graph, fallbackEnd, arrivalTarget);

        return SelectBuildingSideCandidate(graph, buffer, count, arrivalTarget);
    }

    private static int ResolveBuildingSideEndWaypoint(
        IRoutingGraph graph,
        int endIdx,
        Vec3 arrivalTarget)
    {
        var buffer = new int[12];
        buffer[0] = endIdx;
        var count = graph.ExpandLaneCandidates(buffer, 1, buffer.Length, default);
        if (count <= 1)
            return endIdx;

        return SelectBuildingSideCandidate(graph, buffer, count, arrivalTarget);
    }

    private static void ApplyBuildingSideEnd(
        IRoutingGraph graph,
        RouteQuery query,
        List<int> path,
        ref int endIdx)
    {
        var sideEnd = FindBestBuildingSideEndNearTarget(graph, endIdx, query.Destination);
        if (sideEnd == endIdx || path.Count == 0)
            return;

        var anchor = path.Count >= 2 ? path[path.Count - 2] : path[0];
        if (!TryAStar(graph, anchor, sideEnd, allowUturnAtStart: true, query, out var tail, out _) ||
            tail.Count < 1)
            return;

        if (path[path.Count - 1] == endIdx)
            path.RemoveAt(path.Count - 1);

        var startAt = 0;
        if (tail[0] == anchor && tail.Count > 1)
            startAt = 1;
        for (var i = startAt; i < tail.Count; i++)
            path.Add(tail[i]);
        endIdx = sideEnd;
    }

    private static SearchWorkspace GetSearchWorkspace() =>
        _searchWorkspace ??= new SearchWorkspace();

    /// <summary>
    /// Per-thread reusable A* state. Generation stamps avoid clearing large arrays and the
    /// binary min-heap permits duplicate priorities so decrease-key never scans the open set.
    /// </summary>
    private sealed class SearchWorkspace
    {
        private const float ScoreEpsilon = 0.0001f;
        private int[] _seenStamp = Array.Empty<int>();
        private int[] _closedStamp = Array.Empty<int>();
        private int[] _cameFrom = Array.Empty<int>();
        private int[] _rootStart = Array.Empty<int>();
        private float[] _gScore = Array.Empty<float>();
        private HeapEntry[] _heap = Array.Empty<HeapEntry>();
        private int _generation;
        private int _heapCount;

        internal void BeginSearch()
        {
            if (_generation == int.MaxValue)
            {
                Array.Clear(_seenStamp, 0, _seenStamp.Length);
                Array.Clear(_closedStamp, 0, _closedStamp.Length);
                _generation = 0;
            }

            _generation++;
            _heapCount = 0;
        }

        internal bool TryImprove(int node, float score, int cameFrom, int rootStart, float priority)
        {
            EnsureNodeCapacity(node);
            if (_closedStamp[node] == _generation)
                return false;
            if (_seenStamp[node] == _generation && score >= _gScore[node] - ScoreEpsilon)
                return false;

            _seenStamp[node] = _generation;
            _gScore[node] = score;
            _cameFrom[node] = cameFrom;
            _rootStart[node] = rootStart;
            Push(new HeapEntry(node, priority, score));
            return true;
        }

        internal bool TryPop(out int node, out float score)
        {
            while (_heapCount > 0)
            {
                var entry = Pop();
                if (entry.Node < 0 || entry.Node >= _seenStamp.Length ||
                    _seenStamp[entry.Node] != _generation ||
                    _closedStamp[entry.Node] == _generation ||
                    MathF.Abs(entry.Score - _gScore[entry.Node]) > ScoreEpsilon)
                    continue;

                node = entry.Node;
                score = entry.Score;
                return true;
            }

            node = -1;
            score = float.MaxValue;
            return false;
        }

        internal void Close(int node)
        {
            EnsureNodeCapacity(node);
            _closedStamp[node] = _generation;
        }

        internal bool IsClosed(int node)
        {
            EnsureNodeCapacity(node);
            return _closedStamp[node] == _generation;
        }

        internal int GetCameFrom(int node) =>
            node >= 0 && node < _seenStamp.Length && _seenStamp[node] == _generation
                ? _cameFrom[node]
                : -1;

        internal int GetRootStart(int node) =>
            node >= 0 && node < _seenStamp.Length && _seenStamp[node] == _generation
                ? _rootStart[node]
                : node;

        private void EnsureNodeCapacity(int node)
        {
            if (node < 0)
                throw new ArgumentOutOfRangeException(nameof(node));
            if (node < _seenStamp.Length)
                return;

            var capacity = _seenStamp.Length == 0 ? 256 : _seenStamp.Length;
            while (capacity <= node)
                capacity *= 2;

            Array.Resize(ref _seenStamp, capacity);
            Array.Resize(ref _closedStamp, capacity);
            Array.Resize(ref _cameFrom, capacity);
            Array.Resize(ref _rootStart, capacity);
            Array.Resize(ref _gScore, capacity);
        }

        private void Push(HeapEntry entry)
        {
            EnsureHeapCapacity(_heapCount + 1);
            var index = _heapCount++;
            while (index > 0)
            {
                var parent = (index - 1) / 2;
                if (!ComesBefore(entry, _heap[parent]))
                    break;
                _heap[index] = _heap[parent];
                index = parent;
            }

            _heap[index] = entry;
        }

        private HeapEntry Pop()
        {
            var root = _heap[0];
            var last = _heap[--_heapCount];
            if (_heapCount == 0)
                return root;

            var index = 0;
            while (true)
            {
                var left = index * 2 + 1;
                if (left >= _heapCount)
                    break;
                var right = left + 1;
                var child = right < _heapCount && ComesBefore(_heap[right], _heap[left]) ? right : left;
                if (!ComesBefore(_heap[child], last))
                    break;
                _heap[index] = _heap[child];
                index = child;
            }

            _heap[index] = last;
            return root;
        }

        private void EnsureHeapCapacity(int required)
        {
            if (required <= _heap.Length)
                return;
            var capacity = _heap.Length == 0 ? 256 : _heap.Length * 2;
            while (capacity < required)
                capacity *= 2;
            Array.Resize(ref _heap, capacity);
        }

        private static bool ComesBefore(HeapEntry left, HeapEntry right)
        {
            if (left.Priority < right.Priority - ScoreEpsilon)
                return true;
            if (left.Priority > right.Priority + ScoreEpsilon)
                return false;
            if (left.Node != right.Node)
                return left.Node < right.Node;
            return left.Score < right.Score;
        }

        private readonly struct HeapEntry
        {
            internal HeapEntry(int node, float priority, float score)
            {
                Node = node;
                Priority = priority;
                Score = score;
            }

            internal int Node { get; }
            internal float Priority { get; }
            internal float Score { get; }
        }
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
