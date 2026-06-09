using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Routing;

namespace VoogleRoute.Pathfinding.Graph;

/// <summary>
/// Graphe routier prétraité : coûts de segment (longueur / arc CSV, facteur vitesse) et pénalités de virage.
/// Port fidèle de TrafficRoutingIndex.cs du mod VoogleRoute.
/// </summary>
internal sealed class RoutingIndex
{
    private const float DefaultSpeedKmh = 45f;
    private const float ReferenceSpeedKmh = 50f;

    private readonly Vec3[] _positions;
    private readonly int[][] _forwardNeighbors;
    private readonly int[][] _laneChangeNeighbors;
    private readonly float[][] _forwardCosts;
    private readonly float[][] _laneChangeCosts;
    private readonly IReadOnlyDictionary<long, float>? _edgeLengths;
    private readonly bool[] _intersectionNode;

    private RoutingIndex(
        Vec3[] positions,
        int[][] forwardNeighbors,
        int[][] laneChangeNeighbors,
        float[][] forwardCosts,
        float[][] laneChangeCosts,
        IReadOnlyDictionary<long, float>? edgeLengths,
        bool[] intersectionNode)
    {
        _positions = positions;
        _forwardNeighbors = forwardNeighbors;
        _laneChangeNeighbors = laneChangeNeighbors;
        _forwardCosts = forwardCosts;
        _laneChangeCosts = laneChangeCosts;
        _edgeLengths = edgeLengths;
        _intersectionNode = intersectionNode;
    }

    internal static RoutingIndex Build(
        Vec3[] positions,
        int[][] forwardEdges,
        IReadOnlyDictionary<long, Vec3>? turnControls = null,
        int[][]? laneChangeEdges = null,
        IReadOnlyDictionary<long, float>? edgeLengths = null,
        bool[]? intersectionNode = null)
    {
        var forwardCosts = BuildEdgeCosts(positions, forwardEdges, turnControls, edgeLengths);
        var laneChanges = laneChangeEdges ?? CreateEmptyNeighbors(forwardEdges.Length);
        var laneChangeCosts = BuildLaneChangeCosts(positions, laneChanges);
        var intersections = intersectionNode ?? new bool[forwardEdges.Length];
        return new RoutingIndex(
            positions,
            forwardEdges,
            laneChanges,
            forwardCosts,
            laneChangeCosts,
            edgeLengths,
            intersections);
    }

    internal float GetForwardTravelCost(int from, int to, int incomingFrom)
    {
        var cost = LookupCost(_forwardNeighbors, _forwardCosts, from, to);
        if (cost < 0f)
            cost = LookupCost(_laneChangeNeighbors, _laneChangeCosts, from, to);

        if (cost < 0f)
            cost = Vec3.FlatLength(_positions[from], _positions[to]);

        if (incomingFrom >= 0 && !IsLaneChange(from, to))
            cost += GetTurnPenalty(incomingFrom, from, to);

        return cost;
    }

    internal float GetLaneChangeTravelCost(int from, int to)
    {
        var cost = LookupCost(_laneChangeNeighbors, _laneChangeCosts, from, to);
        return cost >= 0f ? cost : CsvLaneChangeBuilder.ComputeLaneChangeCost(_positions, from, to);
    }

    internal bool IsLaneChange(int from, int to) =>
        ContainsEdge(_laneChangeNeighbors, from, to);

    internal float EstimatePathCost(IReadOnlyList<int> path)
    {
        if (path.Count < 2)
            return 0f;

        var cost = 0f;
        var incoming = -1;
        for (var i = 0; i < path.Count - 1; i++)
        {
            cost += GetForwardTravelCost(path[i], path[i + 1], incoming);
            incoming = path[i];
        }

        return cost;
    }

    private static int[][] CreateEmptyNeighbors(int size)
    {
        var neighbors = new int[size][];
        for (var i = 0; i < size; i++)
            neighbors[i] = [];
        return neighbors;
    }

    private static float[][] BuildEdgeCosts(
        Vec3[] positions,
        int[][] neighbors,
        IReadOnlyDictionary<long, Vec3>? turnControls,
        IReadOnlyDictionary<long, float>? edgeLengths)
    {
        var size = neighbors.Length;
        var costs = new float[size][];
        for (var from = 0; from < size; from++)
        {
            var next = neighbors[from];
            if (next.Length == 0)
            {
                costs[from] = Array.Empty<float>();
                continue;
            }

            var row = new float[next.Length];
            for (var i = 0; i < next.Length; i++)
                row[i] = ComputeSegmentCost(positions, from, next[i], turnControls, edgeLengths);

            costs[from] = row;
        }

        return costs;
    }

    private static float[][] BuildLaneChangeCosts(Vec3[] positions, int[][] neighbors)
    {
        var size = neighbors.Length;
        var costs = new float[size][];
        for (var from = 0; from < size; from++)
        {
            var next = neighbors[from];
            if (next.Length == 0)
            {
                costs[from] = Array.Empty<float>();
                continue;
            }

            var row = new float[next.Length];
            for (var i = 0; i < next.Length; i++)
                row[i] = CsvLaneChangeBuilder.ComputeLaneChangeCost(positions, from, next[i]);

            costs[from] = row;
        }

        return costs;
    }

    private static float ComputeSegmentCost(
        Vec3[] positions,
        int from,
        int to,
        IReadOnlyDictionary<long, Vec3>? turnControls,
        IReadOnlyDictionary<long, float>? edgeLengths)
    {
        var key = RouteGraph.EdgeKey(from, to);
        float length;
        if (edgeLengths != null && edgeLengths.TryGetValue(key, out var csvLength) && csvLength > 0f)
        {
            length = csvLength;
        }
        else if (turnControls != null && turnControls.TryGetValue(key, out var control))
        {
            length = ManeuverGeometry.SyntheticTurnTravelMeters(positions[from], positions[to], control);
        }
        else
        {
            length = Vec3.FlatLength(positions[from], positions[to]);
        }

        return length * (ReferenceSpeedKmh / DefaultSpeedKmh);
    }

    private float LookupCost(int[][] neighbors, float[][] costs, int from, int to)
    {
        if (from < 0 || from >= neighbors.Length)
            return -1f;

        var next = neighbors[from];
        var row = costs[from];
        for (var i = 0; i < next.Length; i++)
        {
            if (next[i] == to)
                return row[i];
        }

        return -1f;
    }

    private static bool ContainsEdge(int[][] neighbors, int from, int to)
    {
        if (from < 0 || from >= neighbors.Length)
            return false;

        var next = neighbors[from];
        for (var i = 0; i < next.Length; i++)
        {
            if (next[i] == to)
                return true;
        }

        return false;
    }

    private float GetTurnPenalty(int incoming, int at, int to)
    {
        if (at < 0 || at >= _intersectionNode.Length || !_intersectionNode[at])
            return 0f;

        var abs = TurnGeometry.AbsLaneTurnDegrees(_positions, _forwardNeighbors, incoming, at, to);
        return TurnPenalties.PenaltyMeters(abs);
    }
}
