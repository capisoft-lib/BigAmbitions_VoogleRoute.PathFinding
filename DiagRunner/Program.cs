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
var scenario = GetArg(args, "--scenario");

if (!File.Exists(csv))
{
    Console.Error.WriteLine("CSV not found: " + csv);
    return 1;
}

var graph = CsvRouteGraphLoader.LoadFromEnhancedCsv(csv);
Console.WriteLine($"Graph size={graph.Size} csv={Path.GetFileName(csv)}");
Console.WriteLine($"MaxAStarNodes=32768 (WaypointPathfinder)");

if (scenario == "deadends")
{
    return DeadEndDiagnostics.Run(graph, GetArg(args, "--output"));
}

if (scenario == "third45")
{
    var failed = RunThirdStreet45Scenario(graph);
    return failed ? 1 : 0;
}

if (scenario == "third45_ingame")
{
    var failed = RunThirdStreet45InGameScenario(graph);
    return failed ? 1 : 0;
}

if (scenario == "third45_hint")
{
    RunThirdStreet45HintRegression(graph);
    return 0;
}

if (scenario == "eleventh21")
{
    RunEleventhStreet21Scenario(graph);
    return 0;
}

if (scenario == "eleventh26")
{
    RunEleventhStreet26Scenario(graph);
    return 0;
}

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

static bool RunThirdStreet45Scenario(RouteGraph graph)
{
    // From voogle-route_2026-06-14_15-52-34.log (45 3rd St, car on 3rd facing south).
    var origin = new Vec3(220.98f, 0.01f, -235.04f);
    var dest = new Vec3(214.21f, 0.09f, -136.95f);
    var forward = new Vec3(0f, 0f, -1f); // heading 180°

    Console.WriteLine("# third45 | origin=(220.98,-235.04) heading=180 dest=(214.21,-136.95)");
    Console.WriteLine("# building=west side of 3rd St — correct lane is west (~x=213-215), wrong lane is east (~x=225)");

    var cases = new (string id, bool preferSide, bool allowUturn, float maxCostMeters, int expectedEndWp)[]
    {
        ("release_v0.11.7", false, true, 140f, 13393),
        ("side_off_uturn_off", false, false, 700f, 13393),
        ("side_on_uturn_on", true, true, 750f, 7242),
        ("side_on_uturn_off", true, false, 900f, 7242),
    };

    var anyFail = false;
    foreach (var (id, preferSide, allowUturn, maxCost, expectedEndWp) in cases)
    {
        if (!TryRunThird45Case(graph, origin, dest, forward, id, preferSide, allowUturn, hasHint: false, default,
                maxCost, expectedEndWp))
            anyFail = true;
    }

    return anyFail;
}

static bool RunThirdStreet45InGameScenario(RouteGraph graph)
{
    // From voogle-route_2026-06-14_16-48-31.log — exact in-game pose after config sync fix.
    var origin = new Vec3(221.36f, 0.01f, -279.60f);
    var dest = new Vec3(214.21f, 0.09f, -136.95f);
    var forward = new Vec3(0f, 0f, -1f);

    Console.WriteLine("# third45_ingame | origin=(221.36,-279.60) heading=180 dest=(214.21,-136.95)");
    Console.WriteLine("# parity with ModsLocal session 16-48-31 | preferSide=True allowUturn=False");

    return !TryRunThird45Case(graph, origin, dest, forward, "ingame_side_on_uturn_off", true, false,
        hasHint: false, default, maxCostMeters: 950f, expectedEndWp: 7242);
}

static bool TryRunThird45Case(
    RouteGraph graph,
    Vec3 origin,
    Vec3 dest,
    Vec3 forward,
    string id,
    bool preferSide,
    bool allowUturn,
    bool hasHint,
    Vec3 arrivalHint,
    float maxCostMeters,
    int expectedEndWp)
{
    var query = new RouteQuery
    {
        Origin = origin,
        Destination = dest,
        Forward = forward,
        HasPose = true,
        ForcedStartWaypoint = -1,
        ForcedEndWaypoint = -1,
        AllowUturnAtStart = allowUturn,
        PreferBuildingSideArrival = preferSide,
        HasArrivalRoadHint = hasHint,
        ArrivalRoadHint = arrivalHint,
    };

    if (!VehicleRoutePolyline.TryBuild(graph, query, out var built))
    {
        Console.WriteLine($"ROUTE {id} FAIL preferSide={preferSide} allowUturn={allowUturn}");
        return false;
    }

    var endPos = graph.GetPosition(built.Route.EndWaypoint);
    var poly = built.Points;
    Console.WriteLine(
        $"ROUTE {id} OK preferSide={preferSide} allowUturn={allowUturn} append={built.AppendMode} " +
        $"pathWp={built.Route.Path.Count} poly={poly.Count} cost={built.GraphCostMeters:F1}m " +
        $"polyLen={built.PolylineLengthMeters:F1}m startWp={built.Route.StartWaypoint} endWp={built.Route.EndWaypoint} " +
        $"endLane=({endPos.X:F2},{endPos.Z:F2}) last=({poly[^1].X:F2},{poly[^1].Z:F2})");

    var ok = true;
    if (built.Route.EndWaypoint != expectedEndWp)
    {
        Console.WriteLine($"ASSERT {id} endWp: got {built.Route.EndWaypoint} want {expectedEndWp}");
        ok = false;
    }

    if (built.GraphCostMeters > maxCostMeters)
    {
        Console.WriteLine($"ASSERT {id} cost: got {built.GraphCostMeters:F1}m max {maxCostMeters:F0}m");
        ok = false;
    }

    if (preferSide && !allowUturn)
    {
        var onThird = poly.Where(p => p.Z > -280f && p.Z < -120f).ToList();
        if (onThird.Count > 0)
        {
            var maxX = onThird.Max(p => p.X);
            if (maxX > 222.5f)
            {
                Console.WriteLine($"ASSERT {id} lane: maxX on 3rd segment={maxX:F2} want <222.5 (not east lane ~225)");
                ok = false;
            }
        }
    }

    foreach (var p in poly)
        Console.WriteLine($"  {p.X:F3} {p.Z:F3}");
    Console.WriteLine(ok ? "ASSERT OK" : "ASSERT FAIL");
    Console.WriteLine("ENDROUTE");
    Console.WriteLine();
    return ok;
}

static void RunEleventhStreet21Scenario(RouteGraph graph)
{
    // 21 11th St under bridge — in-game defaults (preferSide off, uturn off).
    var origin = new Vec3(-520f, 0.01f, -280f);
    var destGround = new Vec3(-475.5f, 0.01f, -294.5f);
    var destTest = new Vec3(-475.5f, 1.0f, -294.5f);
    var forward = new Vec3(0f, 0f, 1f);

    foreach (var (label, dest, preferSide) in new (string, Vec3, bool)[]
    {
        ("ingame_Y0.01_sideOff", destGround, false),
        ("ingame_Y0.01_sideOn", destGround, true),
        ("test_Y1.0_sideOn", destTest, true),
        ("test_Y1.0_sideOff", destTest, false),
    })
    {
        var q = new RouteQuery
        {
            Origin = origin,
            Destination = dest,
            Forward = forward,
            HasPose = true,
            AllowUturnAtStart = false,
            PreferBuildingSideArrival = preferSide,
        };

        graph.TryFindNearest(dest, 200f, out var near);
        var np = graph.GetPosition(near);
        Console.WriteLine($"# {label} destY={dest.Y:F2} nearest={near} Y={np.Y:F2}");

        if (!VehicleRoutePolyline.TryBuild(graph, q, out var built))
        {
            Console.WriteLine("  ROUTE FAIL");
            continue;
        }

        var ep = graph.GetPosition(built.Route.EndWaypoint);
        var bridgePts = built.Points.Count(p => p.Y > 8f);
        Console.WriteLine(
            $"  endWp={built.Route.EndWaypoint} endY={ep.Y:F2} cost={built.GraphCostMeters:F0}m " +
            $"poly={built.Points.Count} bridgePts={bridgePts} append={built.AppendMode}");
        Console.WriteLine(
            $"  last=({built.Points[^1].X:F1},{built.Points[^1].Y:F1},{built.Points[^1].Z:F1})");
        foreach (var p in built.Points)
            Console.WriteLine($"    {p.X:F1} {p.Y:F1} {p.Z:F1}");
        Console.WriteLine();
    }
}

static void RunEleventhStreet26Scenario(RouteGraph graph)
{
    var destGround = new Vec3(-475.5f, 0.01f, -273.5f);

    foreach (var (label, origin, forward) in new (string, Vec3, Vec3)[]
    {
        ("11th_near", new Vec3(-520f, 0.01f, -280f), new Vec3(0f, 0f, 1f)),
        ("bridge_deck", new Vec3(-475f, 16f, -296f), new Vec3(0f, 0f, 1f)),
        ("downtown", graph.GetPosition(516), new Vec3(0f, 0f, -1f)),
    })
    {
        foreach (var preferSide in new[] { false, true })
        {
            var q = new RouteQuery
            {
                Origin = origin,
                Destination = destGround,
                Forward = forward,
                HasPose = true,
                AllowUturnAtStart = false,
                PreferBuildingSideArrival = preferSide,
                ForcedStartWaypoint = -1,
                ForcedEndWaypoint = -1,
            };
            if (!VehicleRoutePolyline.TryBuild(graph, q, out var built))
            {
                Console.WriteLine($"{label} preferSide={preferSide} FAIL");
                continue;
            }

            var ep = graph.GetPosition(built.Route.EndWaypoint);
            var bridgePts = built.Points.Count(p => p.Y > 8f);
            Console.WriteLine(
                $"{label} preferSide={preferSide} endWp={built.Route.EndWaypoint} endY={ep.Y:F2} " +
                $"bridgePts={bridgePts} append={built.AppendMode}");
        }
    }
}

static void RunThirdStreet45HintRegression(RouteGraph graph)
{
    // From voogle-route_2026-06-14_16-25-09.log — in-game navmesh snap lands on east lane (~225).
    var origin = new Vec3(221.20f, 0.01f, -256.45f);
    var dest = new Vec3(214.21f, 0.09f, -136.95f);
    var forward = new Vec3(0f, 0f, -1f);
    var wrongHint = new Vec3(225.40f, 0.01f, -138.82f);

    Console.WriteLine("# third45_hint | wrong east-lane arrival hint must not break building-side routing");

    var query = new RouteQuery
    {
        Origin = origin,
        Destination = dest,
        Forward = forward,
        HasPose = true,
        AllowUturnAtStart = false,
        PreferBuildingSideArrival = true,
        HasArrivalRoadHint = true,
        ArrivalRoadHint = wrongHint,
    };

    if (!VehicleRoutePolyline.TryBuild(graph, query, out var built))
    {
        Console.WriteLine("REGRESSION FAIL preferSide=True with wrong navmesh hint");
        return;
    }

    var endPos = graph.GetPosition(built.Route.EndWaypoint);
    Console.WriteLine(
        $"REGRESSION OK append={built.AppendMode} poly={built.Points.Count} endWp={built.Route.EndWaypoint} " +
        $"endLane=({endPos.X:F2},{endPos.Z:F2}) last=({built.Points[^1].X:F2},{built.Points[^1].Z:F2})");

    var onThird = built.Points.Where(p => p.Z > -280f && p.Z < -120f).ToList();
    if (onThird.Count > 0)
    {
        var avgX = onThird.Average(p => p.X);
        var maxX = onThird.Max(p => p.X);
        Console.WriteLine($"REGRESSION lane check | points_on_3rd={onThird.Count} avgX={avgX:F2} maxX={maxX:F2} (want <222, not ~225)");
    }
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
    if (!VehicleRoutePolyline.TryBuild(graph, query, out var built))
    {
        Console.WriteLine($"{label}: FAIL");
        return;
    }

    Console.WriteLine(
        $"{label}: OK path={built.Route.Path.Count} explored={built.Route.NodesExplored} poly={built.Points.Count} " +
        $"append={built.AppendMode} wp {built.Route.StartWaypoint}->{built.Route.EndWaypoint}");
}

readonly record struct ProbePair(string Label, int Start, int End, string Note);
