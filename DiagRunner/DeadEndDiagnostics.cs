using System.Text.Json;
using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;

internal static class DeadEndDiagnostics
{
    internal static int Run(RouteGraph graph, string? output)
    {
        var fixtures = new[]
        {
            (237, 15987, 10345, 14484, 11060), (222, 3206, 5888, 14398, 6237),
            (230, 10331, 9644, 6935, 3803), (210, 13386, 17491, 3923, 4695),
            (233, 12509, 8144, 2495, 17628), (246, 7194, 8963, 8311, 11789),
        };
        var results = new List<object>();
        foreach (var (road, approach, start, end, target) in fixtures)
        {
            var query = new RouteQuery
            {
                Origin = graph.GetPosition(approach), Destination = graph.GetPosition(target),
                ForcedStartWaypoint = approach, ForcedEndWaypoint = target,
                AllowUturnAtStart = false, PreferBuildingSideArrival = false,
            };
            var found = VehicleRoutePolyline.TryBuild(graph, query, out var built);
            Console.WriteLine($"Road {road}: found={found} cost={(found ? built.GraphCostMeters : -1)} " +
                $"approachAllowed={graph.IsForwardEdgeAllowed(-1, approach, start)} " +
                $"turnAllowed={graph.IsForwardEdgeAllowed(approach, start, end)} " +
                $"exitAllowed={graph.IsForwardEdgeAllowed(start, end, target)}");
            results.Add(new
            {
                road, approach, start, end, target, found,
                cost = found ? built.GraphCostMeters : -1,
                length = found ? built.PolylineLengthMeters : -1,
                path = found ? built.Route.Path : null,
                points = found ? built.Points.Select(p => new[] { p.X, p.Y, p.Z }).ToArray() : null,
            });
        }
        if (!string.IsNullOrEmpty(output))
        {
            File.WriteAllText(output, JsonSerializer.Serialize(results, new JsonSerializerOptions { WriteIndented = true }));
            var buffer = new int[graph.Size];
            var index = new Dictionary<int, int[]>();
            using var stream = new MemoryStream();
            using (var writer = new BinaryWriter(stream, System.Text.Encoding.UTF8, leaveOpen: true))
            {
                for (var i = 0; i < graph.Size; i++)
                {
                    buffer[0] = i;
                    var count = graph.ExpandLaneCandidates(buffer, 1, buffer.Length, default);
                    index[i] = buffer.Skip(1).Take(count - 1).ToArray();
                    writer.Write(i);
                    writer.Write(count - 1);
                    foreach (var next in index[i]) writer.Write(next);
                }
            }
            Console.WriteLine($"Lane index: rows={index.Count(p => p.Value.Length > 0)}, " +
                $"pairs={index.Values.Sum(p => p.Length)}, " +
                $"SHA256={Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(stream.ToArray()))}");
            File.WriteAllText(output + ".lane-index.json", JsonSerializer.Serialize(index));
        }
        return 0;
    }
}
