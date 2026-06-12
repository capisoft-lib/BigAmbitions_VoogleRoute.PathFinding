using VoogleRoute.Pathfinding.Geometry;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;

var csv = args.Length > 0 && !args[0].StartsWith("--")
    ? args[0]
    : Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
        @"AppData\LocalLow\Hovgaard Games\Big Ambitions\ModsLocal\VoogleRoute\Data\big_ambitions_enhanced_routes.csv");

var stage = GetArg(args, "--stage");
var pairArgs = args.Where(a => a.StartsWith("--pair=")).Select(a => a["--pair=".Length..]).ToList();
var repro = GetArg(args, "--repro");

if (!File.Exists(csv))
{
    Console.Error.WriteLine("CSV not found: " + csv);
    return 1;
}

var graph = CsvRouteGraphLoader.LoadFromEnhancedCsv(csv);
Console.WriteLine($"Graph size={graph.Size} csv={Path.GetFileName(csv)}");
Console.WriteLine($"MaxAStarNodes=32768 (WaypointPathfinder)");

if (repro == "log")
{
    RunLogRepro(graph);
    return 0;
}

var pairs = pairArgs.Count > 0
    ? ParsePairArgs(pairArgs, graph)
    : stage switch
    {
        "bridge" => BridgeStagePairs(graph),
        "industrial" => IndustrialStagePairs(graph),
        "north" => NorthStagePairs(graph),
        "all" => BridgeStagePairs(graph)
            .Concat(IndustrialStagePairs(graph))
            .Concat(NorthStagePairs(graph))
            .ToArray(),
        _ => DefaultPairs(),
    };

if (pairs.Length == 0)
{
    Console.Error.WriteLine("No probe pairs.");
    return 1;
}

Console.WriteLine($"Stage={stage ?? "default"} probes={pairs.Length}");
Console.WriteLine();

var anyFail = false;
foreach (var probe in pairs)
{
  RunProbe(graph, probe, ref anyFail);
}

return anyFail ? 1 : 0;

static void RunProbe(RouteGraph graph, ProbePair probe, ref bool anyFail)
{
    var q = new RouteQuery
    {
        Origin = graph.GetPosition(probe.Start),
        Destination = graph.GetPosition(probe.End),
        ForcedStartWaypoint = probe.Start,
        ForcedEndWaypoint = probe.End,
    };

    var direct = graph.FlatDistance(graph.GetPosition(probe.Start), graph.GetPosition(probe.End));

    if (!WaypointPathfinder.TryFindBestRoute(graph, q, out var r))
    {
        Console.WriteLine($"FAIL | {probe.Label}");
        Console.WriteLine($"     start={probe.Start} end={probe.End} direct={direct:F0}m");
        Console.WriteLine($"     {probe.Note}");
        anyFail = true;
        Console.WriteLine();
        return;
    }

    var ratio = direct > 1f ? r.GraphCostMeters / direct : 0f;
    var turns = r.TurnSummary.Left + r.TurnSummary.Right + r.TurnSummary.UTurn + r.TurnSummary.Sharp;
    Console.WriteLine($"OK   | {probe.Label}");
    Console.WriteLine(
        $"     path={r.Path.Count} explored={r.NodesExplored} cost={r.GraphCostMeters:F0}m " +
        $"direct={direct:F0}m ratio={ratio:F2} turns={turns} penalty={r.TurnSummary.TotalPenaltyMeters:F0}m");
    Console.WriteLine($"     wp {r.StartWaypoint} -> {r.EndWaypoint}");
    Console.WriteLine($"     {probe.Note}");
    Console.WriteLine();
}

static ProbePair[] DefaultPairs() =>
[
    new("SW->NE (max)", 7733, 1133, "corner extremes"),
    new("NE->industrial", 1133, 13382, "NE corner to industrial anchor"),
    new("SW->downtown", 7733, 516, "SW corner to downtown"),
    new("SW->industrial", 7733, 13382, "SW corner to industrial"),
    new("downtown->industrial", 516, 3149, "city center to industrial"),
    new("SE->NW", 3891, 4929, "SE to NW"),
    new("bridge_city->industrial", 6847, 3149, "1706 city connector to industrial"),
];

static ProbePair[] BridgeStagePairs(RouteGraph graph) =>
[
    new("bridge|1706 city->industrial", 529, 7935, "R1706 extremites (ville z~-178 -> industriel z~-1222)"),
    new("bridge|1703 city->industrial", 7679, 12992, "R1703 extremites"),
    new("bridge|1705 city->industrial", 3093, 703, "R1705 extremites"),
    new("bridge|1706->1708 L0 city", 6847, 6711, "traversee 1706 -> portail ville couloir L0"),
    new("bridge|1708 L0 corridor", 6711, 2098, "segment fusionne Road 1708 lane 0 (ville->ouest)"),
    new("bridge|1708 L3 corridor", 2152, 6028, "segment fusionne Road 1708 lane 3 (ouest->ville)"),
    new("bridge|deck south->north L0", 7446, 1382, "R1700 tablier sud->nord L0 Out"),
    new("bridge|deck south->north L1", 7088, 8913, "R1700 tablier sud->nord L1 Out"),
    new("bridge|city 1706 -> deck south", 529, 7446, "traversee complete ville->tablier industriel"),
];

static ProbePair[] IndustrialStagePairs(RouteGraph graph) =>
[
    new("industrial|deck south->zone", 7446, 3149, "tablier sud -> wp zone industrielle (3572)"),
    new("industrial|deck south->168 L3", 7446, 3572, "tablier sud -> jonction R168 L3"),
    new("industrial|city bridge -> zone", 6847, 3149, "connecteur ville 1706 -> zone industrielle"),
    new("industrial|city bridge -> deck south", 6847, 7446, "ville -> tablier sud via pont"),
    new("industrial|downtown -> deck south", 516, 7446, "centre-ville -> tablier sud"),
    new("industrial|downtown -> zone", 516, 3149, "centre-ville -> zone industrielle"),
];

static ProbePair[] NorthStagePairs(RouteGraph graph)
{
    var dest = 3149;
    var list = new List<ProbePair>
    {
        new("north|deck north->industrial", 1382, dest, "R1700 nord -> zone (58m direct)"),
        new("north|deck south->industrial", 7446, dest, "R1700 sud -> zone (167m direct)"),
        new("north|bridge city->industrial", 6847, dest, "connecteur 1706 -> zone (1380m)"),
        new("north|bridge city end->industrial", 529, dest, "extremite ville 1706 -> zone (1722m)"),
        new("north|downtown->industrial", 516, dest, "centre-ville -> zone (2275m)"),
        new("north|NE corner->industrial", 1133, dest, "extreme NE -> zone (2691m)"),
        new("north|NE->deck south", 1133, 7446, "extreme NE -> tablier sud (2738m)"),
        new("north|LIMIT SW pocket->industrial", 7733, dest, "composante isolee SW (81 noeuds, 394m direct)"),
        new("north|LIMIT NW dead-end", 4929, dest, "dead-end forward R166 (213m direct)"),
        new("north|LIMIT SE dead-end", 3891, dest, "dead-end forward R11 (2248m direct)"),
    };

    return list.ToArray();
}

static ProbePair[] ParsePairArgs(IEnumerable<string> pairArgs, RouteGraph graph)
{
    var list = new List<ProbePair>();
    foreach (var arg in pairArgs)
    {
        var parts = arg.Split(':', 3);
        if (parts.Length < 2 || !int.TryParse(parts[0], out var start) || !int.TryParse(parts[1], out var end))
        {
            Console.Error.WriteLine($"Invalid --pair: {arg} (expected start:end[:label])");
            continue;
        }

        var label = parts.Length > 2 ? parts[2] : $"{start}->{end}";
        list.Add(new(label, start, end, $"custom pair {start}->{end}"));
    }

    return list.ToArray();
}

static string? GetArg(string[] args, string name)
{
    var idx = Array.IndexOf(args, name);
    if (idx < 0 || idx + 1 >= args.Length)
        return null;
    return args[idx + 1];
}

static void RunLogRepro(RouteGraph graph)
{
    var cases = new (string label, Vec3 origin, Vec3 dest)[]
    {
        ("4th_south_wordsmith", new Vec3(134f, 0.44f, 55f), new Vec3(145f, 0.41f, -8f)),
        ("4th_14_log", new Vec3(174.44f, 0.46f, -25.72f), new Vec3(255.54f, 0.09f, -6.44f)),
        ("eighth_8_FAIL", new Vec3(131.28f, 0.44f, 121.01f), new Vec3(-1740.94f, 0.41f, -1163.29f)),
        ("25th_9_OK", new Vec3(131.28f, 0.44f, 121.01f), new Vec3(-1759.67f, 0.37f, -1393.37f)),
        ("22nd_11_FAIL", new Vec3(131.28f, 0.44f, 121.01f), new Vec3(-1489.64f, 0.41f, -1257.01f)),
        ("1st_2_OK", new Vec3(131.28f, 0.44f, 121.01f), new Vec3(404.27f, 0.09f, 444.64f)),
        ("1st_player_move_FAIL", new Vec3(133.88f, 0.46f, 82.45f), new Vec3(404.27f, 0.09f, 444.64f)),
    };

    foreach (var (label, origin, dest) in cases)
    {
        Console.WriteLine($"=== {label} -> ({dest.X:F1},{dest.Z:F1})");
        graph.TryFindNearest(origin, 200f, out var startNear);
        graph.TryFindNearest(dest, 200f, out var endNear);
        var sp = graph.GetPosition(startNear);
        var ep = graph.GetPosition(endNear);
        Console.WriteLine($"  nearest start={startNear} @ ({sp.X:F1},{sp.Z:F1}) end={endNear} @ ({ep.X:F1},{ep.Z:F1})");

        var south = new Vec3(0f, 0f, -1f);
        var q = new RouteQuery
        {
            Origin = origin,
            Destination = dest,
            HasPose = true,
            Forward = south,
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = -1,
        };
        ProbeQuery(graph, "  pose_south", q);

        if (label == "4th_14_log")
            ComparePoseVsRelaxed(graph, origin, dest, south);

        if (label is "25th_9_OK" or "1st_player_move_FAIL")
        {
            for (var h = 0; h < 360; h += (label == "25th_9_OK" ? 15 : 30))
            {
                var rad = h * (MathF.PI / 180f);
                var fwd = new Vec3(MathF.Sin(rad), 0, MathF.Cos(rad));
                var hq = new RouteQuery
                {
                    Origin = origin,
                    Destination = dest,
                    HasPose = true,
                    Forward = fwd,
                    ForcedStartWaypoint = -1,
                    ForcedEndWaypoint = -1,
                };
                if (!WaypointPathfinder.TryFindBestRoute(graph, hq, out var hr))
                {
                    Console.WriteLine($"  h={h,3}: FAIL (no route)");
                    continue;
                }

                var poly = RoutePolylineBuilder.BuildPoints(graph, hr.Path, origin, dest).Count;
                Console.WriteLine($"  h={h,3}: OK explored={hr.NodesExplored} poly={poly}");
            }
        }

        Console.WriteLine();
    }
}

static void ComparePoseVsRelaxed(RouteGraph graph, Vec3 origin, Vec3 dest, Vec3 forward)
{
    var poseQ = new RouteQuery
    {
        Origin = origin,
        Destination = dest,
        HasPose = true,
        Forward = forward,
        ForcedStartWaypoint = -1,
        ForcedEndWaypoint = -1,
    };
    var relaxQ = poseQ with { HasPose = false, Forward = default };

    if (!WaypointPathfinder.TryFindBestRoute(graph, poseQ, out var best))
    {
        Console.WriteLine("  best: FAIL");
        return;
    }

    WaypointPathfinder.TryFindBestRoute(graph, relaxQ, out var relaxedOnly);

    Console.WriteLine($"  direct={graph.FlatDistance(origin, dest):F1}m");
    Console.WriteLine($"  best(pose+compare): path={best.Path.Count} flat={WaypointPathfinder.GetFlatRouteDistance(graph, best):F1}m wp {best.StartWaypoint}->{best.EndWaypoint}");
    Console.WriteLine($"  relaxed_only: path={relaxedOnly.Path.Count} flat={WaypointPathfinder.GetFlatRouteDistance(graph, relaxedOnly):F1}m wp {relaxedOnly.StartWaypoint}->{relaxedOnly.EndWaypoint}");

    for (var h = 0; h < 360; h += 30)
    {
        var rad = h * (MathF.PI / 180f);
        var fwd = new Vec3(MathF.Sin(rad), 0, MathF.Cos(rad));
        var hq = new RouteQuery
        {
            Origin = origin,
            Destination = dest,
            HasPose = true,
            Forward = fwd,
            ForcedStartWaypoint = -1,
            ForcedEndWaypoint = -1,
        };
        if (!WaypointPathfinder.TryFindBestRoute(graph, hq, out var hr))
            continue;
        Console.WriteLine($"  h={h,3}: path={hr.Path.Count} flat={WaypointPathfinder.GetFlatRouteDistance(graph, hr):F0}m wp {hr.StartWaypoint}->{hr.EndWaypoint}");
    }
}

static void ProbeQuery(RouteGraph graph, string label, RouteQuery query)
{
    if (!WaypointPathfinder.TryFindBestRoute(graph, query, out var r))
    {
        Console.WriteLine($"{label}: FAIL");
        return;
    }

    var poly = RoutePolylineBuilder.BuildPoints(
        graph, r.Path, prependOrigin: query.Origin, appendDestination: query.Destination);
    Console.WriteLine(
        $"{label}: OK path={r.Path.Count} explored={r.NodesExplored} poly={poly.Count} " +
        $"wp {r.StartWaypoint}->{r.EndWaypoint}");
}

readonly record struct ProbePair(string Label, int Start, int End, string Note);
