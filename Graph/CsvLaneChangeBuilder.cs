using System.Collections.Generic;
using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Routing;

namespace VoogleRoute.Pathfinding.Graph;

internal static class CsvLaneChangeBuilder
{
    private const float ParallelLaneMaxLateralMeters = 18f;
    private const int ApproachHops = 5;
    private const float LaneChangeBaseMeters = 22f;

    internal static bool[] BuildApproachZone(int[][] reverseEdges, bool[] junctionZone)
    {
        var size = junctionZone.Length;
        var approachZone = new bool[size];
        var queue = new Queue<(int node, int depth)>();
        var visited = new bool[size];

        for (var i = 0; i < size; i++)
        {
            if (!junctionZone[i])
                continue;

            queue.Enqueue((i, 0));
            visited[i] = true;
        }

        while (queue.Count > 0)
        {
            var (node, depth) = queue.Dequeue();
            if (depth > 0)
                approachZone[node] = true;

            if (depth >= ApproachHops)
                continue;

            var reverse = reverseEdges[node];
            for (var i = 0; i < reverse.Length; i++)
            {
                var prev = reverse[i];
                if (prev < 0 || prev >= size || visited[prev])
                    continue;

                visited[prev] = true;
                queue.Enqueue((prev, depth + 1));
            }
        }

        return approachZone;
    }

    internal static int[][] BuildLaneChangeEdges(
        Vec3[] positions,
        int[][] forwardEdges,
        int[][] otherLanes,
        bool[] junctionZone,
        bool[] approachZone)
    {
        var size = positions.Length;
        var laneChanges = new int[size][];
        var maxLateralSq = ParallelLaneMaxLateralMeters * ParallelLaneMaxLateralMeters;

        for (var idx = 0; idx < size; idx++)
        {
            var list = new List<int>(4);
            var inZone = idx < junctionZone.Length && junctionZone[idx];
            var inApproach = idx < approachZone.Length && approachZone[idx];
            var allowParallel = inZone || inApproach;

            var lanes = idx < otherLanes.Length ? otherLanes[idx] : null;
            if (lanes != null)
            {
                for (var i = 0; i < lanes.Length; i++)
                {
                    var lane = lanes[i];
                    if (!IsValidLaneChangeTarget(positions, forwardEdges, idx, lane, allowParallel, maxLateralSq))
                        continue;

                    list.Add(lane);
                }
            }

            laneChanges[idx] = list.Count == 0 ? [] : list.ToArray();
        }

        return laneChanges;
    }

    internal static float ComputeLaneChangeCost(Vec3[] positions, int from, int to)
    {
        var lateral = Vec3.FlatLength(positions[from], positions[to]);
        return MathF.Max(lateral, 4f) + LaneChangeBaseMeters;
    }

    private static bool IsValidLaneChangeTarget(
        Vec3[] positions,
        int[][] forwardEdges,
        int fromIdx,
        int toIdx,
        bool allowParallel,
        float maxLateralSq)
    {
        if (toIdx < 0 || toIdx >= positions.Length || fromIdx == toIdx)
            return false;

        var edges = forwardEdges[toIdx];
        if (edges == null || edges.Length == 0)
            return false;

        if (!LaneFlow.SharesTravelDirection(positions, forwardEdges, fromIdx, toIdx))
            return false;

        if (allowParallel)
            return true;

        var dx = positions[fromIdx].X - positions[toIdx].X;
        var dz = positions[fromIdx].Z - positions[toIdx].Z;
        return dx * dx + dz * dz <= maxLateralSq;
    }

}
