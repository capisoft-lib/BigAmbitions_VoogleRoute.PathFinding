using System;
using System.Collections.Generic;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Graph;

/// <summary>
/// Réplique les virages synthétiques (souvent générés depuis la voie « leftmost » uniquement)
/// sur toutes les voies parallèles (otherLanes ou même road + proximité).
/// </summary>
public static class ParallelLaneTurnExpander
{
    public const float SameRoadClusterMeters = 28f;

    public readonly struct TurnEdge
    {
        public TurnEdge(int from, int to, Vec3 control)
        {
            From = from;
            To = to;
            Control = control;
        }

        public int From { get; }
        public int To { get; }
        public Vec3 Control { get; }
    }

    public static void ExpandTurnsToParallelLanes(
        List<int>[] builders,
        HashSet<int>[] seen,
        IReadOnlyList<TurnEdge> turns,
        Dictionary<long, Vec3> turnControls,
        int[][]? otherLanes,
        Vec3[] positions,
        IReadOnlyDictionary<int, string>? roadByIndex)
    {
        if (turns == null || turns.Count == 0)
            return;

        for (var t = 0; t < turns.Count; t++)
        {
            var turn = turns[t];
            if (turn.From < 0 || turn.To < 0 || turn.From >= positions.Length || turn.To >= positions.Length)
                continue;

            foreach (var source in CollectParallelSources(turn.From, otherLanes, positions, roadByIndex))
            {
                if (source < 0 || source >= builders.Length)
                    continue;
                if (builders[source] == null || seen[source] == null)
                    continue;

                if (!seen[source].Add(turn.To))
                    continue;

                builders[source].Add(turn.To);
                turnControls[RouteGraph.EdgeKey(source, turn.To)] = turn.Control;
            }
        }
    }

    public static IReadOnlyList<int> CollectParallelSources(
        int seed,
        int[][]? otherLanes,
        Vec3[] positions,
        IReadOnlyDictionary<int, string>? roadByIndex)
    {
        if (otherLanes != null && seed >= 0 && seed < otherLanes.Length)
            return CollectOtherLaneCluster(seed, otherLanes);

        if (roadByIndex != null && roadByIndex.TryGetValue(seed, out var road))
            return CollectSameRoadCluster(seed, road, positions, roadByIndex);

        return new[] { seed };
    }

    public static int[] CollectOtherLaneCluster(int seed, int[][] otherLanes)
    {
        var cluster = new List<int> { seed };
        var visited = new HashSet<int> { seed };
        var queue = new Queue<int>();
        queue.Enqueue(seed);

        while (queue.Count > 0)
        {
            var node = queue.Dequeue();
            if (node < 0 || node >= otherLanes.Length)
                continue;

            var lanes = otherLanes[node];
            if (lanes == null)
                continue;

            for (var i = 0; i < lanes.Length; i++)
            {
                var next = lanes[i];
                if (next < 0 || next >= otherLanes.Length || !visited.Add(next))
                    continue;

                cluster.Add(next);
                queue.Enqueue(next);
            }
        }

        return cluster.ToArray();
    }

    private static int[] CollectSameRoadCluster(
        int seed,
        string road,
        Vec3[] positions,
        IReadOnlyDictionary<int, string> roadByIndex)
    {
        var cluster = new List<int> { seed };
        var seedPos = positions[seed];
        var maxSq = SameRoadClusterMeters * SameRoadClusterMeters;

        foreach (var pair in roadByIndex)
        {
            if (pair.Key == seed || pair.Value != road)
                continue;

            var pos = positions[pair.Key];
            var dx = pos.X - seedPos.X;
            var dz = pos.Z - seedPos.Z;
            if (dx * dx + dz * dz <= maxSq)
                cluster.Add(pair.Key);
        }

        return cluster.ToArray();
    }
}
