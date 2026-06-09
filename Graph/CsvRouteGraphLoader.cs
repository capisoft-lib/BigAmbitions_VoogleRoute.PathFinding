using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Routing;

namespace VoogleRoute.Pathfinding.Graph;

public static class CsvRouteGraphLoader
{
    public static RouteGraph LoadFromEnhancedCsv(string csvPath)
    {
        var positions = new Dictionary<int, Vec3>();
        var roadByIndex = new Dictionary<int, string>();
        var forward = new Dictionary<int, HashSet<int>>();
        var uturns = new HashSet<long>();
        var syntheticTurns = new List<ParallelLaneTurnExpander.TurnEdge>();
        var turnControls = new Dictionary<long, Vec3>();

        using var reader = new StreamReader(csvPath);
        var header = reader.ReadLine();
        if (string.IsNullOrEmpty(header))
            throw new InvalidDataException("CSV vide.");

        string? line;
        while ((line = reader.ReadLine()) != null)
        {
            if (string.IsNullOrWhiteSpace(line))
                continue;

            var cols = line.Split(',');
            if (cols.Length < 23)
                continue;

            var edgeType = cols[1];
            if (edgeType == "base")
            {
                if (!TryParseInt(cols[3], out var from) || !TryParseInt(cols[10], out var to))
                    continue;
                if (!TryParseFloat(cols[7], out var fx) || !TryParseFloat(cols[9], out var fz) ||
                    !TryParseFloat(cols[14], out var tx) || !TryParseFloat(cols[16], out var tz))
                    continue;

                positions[from] = new Vec3(fx, 0, fz);
                positions[to] = new Vec3(tx, 0, tz);
                roadByIndex[from] = cols[5];
                roadByIndex[to] = cols[11];
                AddEdge(forward, from, to);
            }
            else if (edgeType == "synthetic_turn")
            {
                var maneuver = cols[2];
                if (maneuver is not ("left" or "uturn"))
                    continue;

                if (!TryParseInt(cols[3], out var from) || !TryParseInt(cols[10], out var to))
                    continue;
                if (!TryParseFloat(cols[7], out var fx) || !TryParseFloat(cols[9], out var fz) ||
                    !TryParseFloat(cols[14], out var tx) || !TryParseFloat(cols[16], out var tz))
                    continue;

                positions[from] = new Vec3(fx, 0, fz);
                positions[to] = new Vec3(tx, 0, tz);
                roadByIndex[from] = cols[5];
                roadByIndex[to] = cols[11];

                Vec3 control = default;
                if (TryParseFloat(cols[17], out var cx) &&
                    TryParseFloat(cols[18], out var cy) &&
                    TryParseFloat(cols[19], out var cz))
                    control = new Vec3(cx, cy, cz);

                syntheticTurns.Add(new ParallelLaneTurnExpander.TurnEdge(from, to, control));
                AddEdge(forward, from, to);
                turnControls[RouteGraph.EdgeKey(from, to)] = control;

                if (maneuver == "uturn")
                    uturns.Add(RouteGraph.EdgeKey(from, to));
            }
        }

        if (positions.Count == 0)
            throw new InvalidDataException("Aucun waypoint dans le CSV.");

        var maxIndex = positions.Keys.Max();
        var size = maxIndex + 1;
        var posArray = new Vec3[size];
        foreach (var (idx, pos) in positions)
            posArray[idx] = pos;

        var otherLanes = BuildOtherLanesFromRoadProximity(posArray, forward, roadByIndex, size);
        var builders = new List<int>[size];
        var seen = new HashSet<int>[size];
        for (var i = 0; i < size; i++)
        {
            builders[i] = forward.TryGetValue(i, out var set)
                ? set.OrderBy(x => x).ToList()
                : new List<int>();
            seen[i] = new HashSet<int>(builders[i]);
        }

        // Synthetic turns stay on their designed lane only — lane changes reach the correct lane first.

        foreach (var turn in syntheticTurns)
        {
            foreach (var source in ParallelLaneTurnExpander.CollectParallelSources(
                         turn.From, otherLanes, posArray, roadByIndex))
            {
                if (!uturns.Contains(RouteGraph.EdgeKey(turn.From, turn.To)))
                    continue;
                uturns.Add(RouteGraph.EdgeKey(source, turn.To));
            }
        }

        var forwardArray = new int[size][];
        var reverseArray = new int[size][];
        for (var i = 0; i < size; i++)
            forwardArray[i] = builders[i].ToArray();

        var reverseBuilders = new List<int>[size];
        for (var i = 0; i < size; i++)
            reverseBuilders[i] = new List<int>();

        for (var from = 0; from < size; from++)
        {
            foreach (var to in forwardArray[from])
                reverseBuilders[to].Add(from);
        }

        for (var i = 0; i < size; i++)
            reverseArray[i] = reverseBuilders[i].OrderBy(x => x).ToArray();

        var junctionZone = BuildJunctionZone(forwardArray, reverseArray);
        var approachZone = CsvLaneChangeBuilder.BuildApproachZone(reverseArray, junctionZone);
        var laneChangeArray = CsvLaneChangeBuilder.BuildLaneChangeEdges(
            posArray, forwardArray, otherLanes, junctionZone, approachZone);
        var routingIndex = RoutingIndex.Build(
            posArray, forwardArray, reverseArray, junctionZone, turnControls, laneChangeArray);

        var xs = positions.Values.Select(p => p.X).ToArray();
        var zs = positions.Values.Select(p => p.Z).ToArray();
        var validIndices = positions.Keys.OrderBy(i => i).ToArray();

        return new RouteGraph(
            posArray,
            forwardArray,
            reverseArray,
            laneChangeArray,
            uturns,
            junctionZone,
            routingIndex,
            otherLanes,
            turnControls,
            validIndices,
            xs.Min(),
            xs.Max(),
            zs.Min(),
            zs.Max());
    }

    private static int[][] BuildOtherLanesFromRoadProximity(
        Vec3[] positions,
        Dictionary<int, HashSet<int>> forward,
        Dictionary<int, string> roadByIndex,
        int size)
    {
        var otherLanes = new int[size][];
        for (var i = 0; i < size; i++)
            otherLanes[i] = Array.Empty<int>();

        var forwardArray = new int[size][];
        for (var i = 0; i < size; i++)
            forwardArray[i] = forward.TryGetValue(i, out var set)
                ? set.OrderBy(x => x).ToArray()
                : Array.Empty<int>();

        var maxSq = ParallelLaneTurnExpander.SameRoadClusterMeters * ParallelLaneTurnExpander.SameRoadClusterMeters;

        foreach (var (idx, road) in roadByIndex)
        {
            var cluster = new List<int> { idx };
            var pos = positions[idx];
            foreach (var (otherIdx, otherRoad) in roadByIndex)
            {
                if (otherIdx == idx || otherRoad != road)
                    continue;

                if (!LaneFlow.SharesTravelDirection(positions, forwardArray, idx, otherIdx))
                    continue;

                var otherPos = positions[otherIdx];
                var dx = otherPos.X - pos.X;
                var dz = otherPos.Z - pos.Z;
                if (dx * dx + dz * dz <= maxSq)
                    cluster.Add(otherIdx);
            }

            if (cluster.Count > 1)
            {
                var withoutSelf = cluster.Where(i => i != idx).ToArray();
                otherLanes[idx] = withoutSelf;
            }
            else
            {
                otherLanes[idx] = Array.Empty<int>();
            }
        }

        return otherLanes;
    }

    private static bool[] BuildJunctionZone(int[][] forward, int[][] reverse)
    {
        var size = forward.Length;
        var zone = new bool[size];
        for (var i = 0; i < size; i++)
        {
            if (forward[i].Length >= 2 || reverse[i].Length >= 2)
                zone[i] = true;
        }

        return zone;
    }

    private static void AddEdge(Dictionary<int, HashSet<int>> forward, int from, int to)
    {
        if (!forward.TryGetValue(from, out var set))
        {
            set = new HashSet<int>();
            forward[from] = set;
        }

        set.Add(to);
    }

    private static bool TryParseInt(string value, out int result) =>
        int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out result);

    private static bool TryParseFloat(string value, out float result) =>
        float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out result);
}
