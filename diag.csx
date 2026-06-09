using System;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;

var csv = @"c:\Users\AI\AI\Dev\Cursor\BigAmbitions\src\VoogleRoute\VoogleRoute\Data\big_ambitions_enhanced_routes.csv";
var g = CsvRouteGraphLoader.LoadFromEnhancedCsv(csv);
var q = RouteQuery.FromWorldCoords(-178.70422f, -381.93646f, 87.92999f, 137.24268f, -119.015686f);
Console.WriteLine($"Graph size: {g.Size}");
var ok = WaypointPathfinder.TryFindBestRoute(g, q, out var r);
Console.WriteLine($"TryFindBestRoute: {ok}");
if (!ok) {
  // diagnose start candidates
  var origin = q.Origin; var forward = q.Forward; var dest = q.Destination;
  var buf = new int[12];
  var radius = 120f;
  var sc = WaypointPathfinder.CollectNearestAligned(g, origin, forward, radius, buf);
  Console.WriteLine($"CollectNearestAligned: {sc} -> [{string.Join(",", buf.AsSpan(0, sc).ToArray())}]");
  var fc = WaypointPathfinder.FilterFlowAligned(g, buf, sc, forward);
  Console.WriteLine($"FilterFlowAligned: {fc}");
  var ec = g.CollectNearest(dest, radius, buf);
  Console.WriteLine($"End candidates: {ec} -> [{string.Join(",", buf.AsSpan(0, ec).ToArray())}]");
} else {
  Console.WriteLine($"Path len={r.Path.Count} start={r.StartWaypoint} end={r.EndWaypoint}");
}
