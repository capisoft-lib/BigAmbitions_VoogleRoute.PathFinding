using System;
using System.Collections.Generic;
using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Routing;

namespace VoogleRoute.Pathfinding.Graph;

public sealed class RouteGraph : IRoutingGraph
{
    /// <summary>When destination Y exceeds this, nearest-waypoint search prefers matching elevation.</summary>
    internal const float ElevationReferenceMinY = 0.1f;

    /// <summary>Destinations at or below this are treated as street-level (building entrances ~0.01 m).</summary>
    internal const float StreetLevelMaxY = 5f;

    /// <summary>Waypoints at or above this are bridge decks / elevated cross roads.</summary>
    internal const float BridgeDeckMinY = 8f;

    /// <summary>Penalty weight for vertical mismatch vs planar distance (m²).</summary>
    internal const float ElevationMismatchWeight = 100f;

    private readonly Vec3[] _positions;
    private readonly int[][] _forwardNeighbors;
    private readonly int[][] _reverseNeighbors;
    private readonly int[][] _laneChangeNeighbors;
    private readonly int[][] _otherLanes;
    private readonly HashSet<long> _authorizedUturnEdges;
    private readonly bool[] _intersectionNode;
    private readonly RoutingIndex _routingIndex;
    private readonly Dictionary<long, Vec3> _turnControls;
    private readonly Dictionary<long, float> _syntheticTurnAngles;
    private readonly int[] _validIndices;

    public int Size => _positions.Length;
    public ReadOnlySpan<int> ValidIndices => _validIndices;

    internal RouteGraph(
        Vec3[] positions,
        int[][] forwardNeighbors,
        int[][] reverseNeighbors,
        int[][] laneChangeNeighbors,
        HashSet<long> authorizedUturnEdges,
        bool[] intersectionNode,
        RoutingIndex routingIndex,
        int[][] otherLanes,
        IReadOnlyDictionary<long, Vec3> turnControls,
        IReadOnlyDictionary<long, float> syntheticTurnAngles,
        int[] validIndices,
        float minX,
        float maxX,
        float minZ,
        float maxZ)
    {
        _positions = positions;
        _forwardNeighbors = forwardNeighbors;
        _reverseNeighbors = reverseNeighbors;
        _laneChangeNeighbors = laneChangeNeighbors;
        _authorizedUturnEdges = authorizedUturnEdges;
        _intersectionNode = intersectionNode;
        _routingIndex = routingIndex;
        _otherLanes = otherLanes;
        _turnControls = new Dictionary<long, Vec3>(turnControls);
        _syntheticTurnAngles = new Dictionary<long, float>(syntheticTurnAngles);
        _validIndices = validIndices;
        MinX = minX;
        MaxX = maxX;
        MinZ = minZ;
        MaxZ = maxZ;
    }

    public bool TryGetSyntheticTurnControl(int from, int to, out Vec3 control) =>
        _turnControls.TryGetValue(EdgeKey(from, to), out control);

    public bool TryGetSyntheticTurnAbsAngle(int from, int to, out float absDegrees) =>
        _syntheticTurnAngles.TryGetValue(EdgeKey(from, to), out absDegrees);

    public float MinX { get; }
    public float MaxX { get; }
    public float MinZ { get; }
    public float MaxZ { get; }

    public Vec3 GetPosition(int index) => _positions[index];

    public ReadOnlySpan<int> GetForwardNeighbors(int index) =>
        index >= 0 && index < _forwardNeighbors.Length
            ? _forwardNeighbors[index]
            : ReadOnlySpan<int>.Empty;

    public ReadOnlySpan<int> GetLaneChangeNeighbors(int index) =>
        index >= 0 && index < _laneChangeNeighbors.Length
            ? _laneChangeNeighbors[index]
            : ReadOnlySpan<int>.Empty;

    public bool IsLaneChangeEdge(int from, int to) => _routingIndex.IsLaneChange(from, to);

    public bool IsSyntheticTurnEdge(int from, int to) =>
        _turnControls.ContainsKey(EdgeKey(from, to));

    public bool HasForwardEdge(int from, int to)
    {
        if (from < 0 || from >= _forwardNeighbors.Length)
            return false;

        var edges = _forwardNeighbors[from];
        for (var i = 0; i < edges.Length; i++)
        {
            if (edges[i] == to)
                return true;
        }

        return false;
    }

    public bool IsAuthorizedUturnEdge(int from, int to) =>
        _authorizedUturnEdges.Contains(EdgeKey(from, to));

    public bool IsForwardEdgeAllowed(int incoming, int at, int next) =>
        LaneFlow.IsForwardEdgeAllowed(this, incoming, at, next);

    public float GetForwardTravelCost(int from, int to, int incomingFrom) =>
        _routingIndex.GetForwardTravelCost(from, to, incomingFrom);

    public float EstimatePathTravelCost(IReadOnlyList<int> path) =>
        _routingIndex.EstimatePathCost(path);

    public bool IsIntersectionNode(int index) =>
        index >= 0 && index < _intersectionNode.Length && _intersectionNode[index];

    public float FlatDistance(Vec3 a, Vec3 b) => Vec3.FlatLength(a, b);

    public float DistanceToDestination(Vec3 position, Vec3 destination)
    {
        var score = NearestCandidateScore(position, destination);
        return score <= 0f ? 0f : MathF.Sqrt(score);
    }

    public float EstimateArrivalLegCost(int endIdx, Vec3 destination) =>
        DistanceToDestination(_positions[endIdx], destination);

    internal static bool HasElevationReference(float y) => MathF.Abs(y) > ElevationReferenceMinY;

    internal static bool IsStreetLevelDestination(float y) => y <= StreetLevelMaxY;

    internal static bool IsElevatedWaypoint(float y) => y >= BridgeDeckMinY;

    internal static bool ShouldWeightElevation(Vec3 worldPos) =>
        HasElevationReference(worldPos.Y) || IsStreetLevelDestination(worldPos.Y);

    internal static float NearestCandidateScore(Vec3 pos, Vec3 worldPos)
    {
        var dx = pos.X - worldPos.X;
        var dz = pos.Z - worldPos.Z;
        var planarSq = dx * dx + dz * dz;
        if (!ShouldWeightElevation(worldPos))
            return planarSq;

        var dy = pos.Y - worldPos.Y;
        return planarSq + ElevationMismatchWeight * dy * dy;
    }

    public int ExpandLaneCandidates(int[] buffer, int count, int capacity, Vec3 flatForward)
    {
        if (count <= 0 || capacity <= count)
            return count;

        var seen = new HashSet<int>();
        for (var i = 0; i < count; i++)
            seen.Add(buffer[i]);

        var write = count;
        for (var i = 0; i < count && write < capacity; i++)
        {
            var idx = buffer[i];
            if (idx < 0 || idx >= _otherLanes.Length)
                continue;

            var lanes = _otherLanes[idx];
            for (var j = 0; j < lanes.Length && write < capacity; j++)
            {
                var lane = lanes[j];
                if (!seen.Add(lane))
                    continue;
                buffer[write++] = lane;
            }
        }

        return write;
    }

    public int CollectNearest(Vec3 worldPos, float maxDistance, int[] buffer)
    {
        var maxSq = maxDistance * maxDistance;
        var hits = new List<(int idx, float score)>();
        foreach (var i in _validIndices)
        {
            var pos = _positions[i];
            var score = NearestCandidateScore(pos, worldPos);
            var dx = pos.X - worldPos.X;
            var dz = pos.Z - worldPos.Z;
            if (dx * dx + dz * dz > maxSq)
                continue;

            hits.Add((i, score));
        }

        hits.Sort((a, b) => a.score.CompareTo(b.score));
        var count = Math.Min(hits.Count, buffer.Length);
        for (var i = 0; i < count; i++)
            buffer[i] = hits[i].idx;
        return count;
    }

    public int CollectNearestAligned(Vec3 worldPos, Vec3 flatForward, float maxDistance, int[] buffer) =>
        WaypointPathfinder.CollectNearestAligned(this, worldPos, flatForward, maxDistance, buffer);

    public int FilterFlowAligned(int[] buffer, int count, Vec3 flatForward) =>
        WaypointPathfinder.FilterFlowAligned(this, buffer, count, flatForward);

    public bool TryFindNearest(Vec3 worldPos, float maxDistance, out int index)
    {
        index = -1;
        var bestScore = float.MaxValue;
        var maxSq = maxDistance * maxDistance;
        foreach (var i in _validIndices)
        {
            var pos = _positions[i];
            var dx = pos.X - worldPos.X;
            var dz = pos.Z - worldPos.Z;
            var planarSq = dx * dx + dz * dz;
            if (planarSq > maxSq)
                continue;

            var score = NearestCandidateScore(pos, worldPos);
            if (score >= bestScore)
                continue;

            bestScore = score;
            index = i;
        }

        return index >= 0;
    }

    public static long EdgeKey(int from, int to) => ((long)from << 32) ^ (uint)to;

    public MapViewport CreateViewport(int width = 1800, int height = 1500, float margin = 70f)
    {
        var spanX = MaxX - MinX;
        var spanZ = MaxZ - MinZ;
        var scale = MathF.Min(
            (width - margin * 2f) / spanX,
            (height - margin * 2f) / spanZ);
        var usedW = spanX * scale;
        var usedH = spanZ * scale;
        return new MapViewport(MinX, MaxZ, scale, (width - usedW) / 2f, (height - usedH) / 2f);
    }
}

public readonly struct MapViewport
{
    public MapViewport(float minX, float maxZ, float scale, float offX, float offY)
    {
        MinX = minX;
        MaxZ = maxZ;
        Scale = scale;
        OffX = offX;
        OffY = offY;
    }

    public float MinX { get; }
    public float MaxZ { get; }
    public float Scale { get; }
    public float OffX { get; }
    public float OffY { get; }

    public (float X, float Y) WorldToSvg(float x, float z) =>
        (OffX + (x - MinX) * Scale, OffY + (MaxZ - z) * Scale);

    public (float X, float Z) SvgToWorld(float sx, float sy) =>
        (MinX + (sx - OffX) / Scale, MaxZ - (sy - OffY) / Scale);
}
