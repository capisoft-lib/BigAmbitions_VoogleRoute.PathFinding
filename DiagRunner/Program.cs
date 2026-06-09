using VoogleRoute.Pathfinding.Graph;

var repoRoot = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", ".."));
var csv = Path.Combine(repoRoot, "data", "big_ambitions_enhanced_routes.csv");
if (!File.Exists(csv))
{
    Console.Error.WriteLine("CSV not found: " + csv);
    return 1;
}

var g = CsvRouteGraphLoader.LoadFromEnhancedCsv(csv);

var inc = 3518;
Console.WriteLine($"From wp9007 incoming={inc}:");
foreach (var n in g.GetForwardNeighbors(9007))
{
    var np = g.GetPosition(n);
    Console.WriteLine($"  -> wp{n} ({np.X:F1},{np.Z:F1}) allowed={g.IsForwardEdgeAllowed(inc,9007,n)} cost={g.GetForwardTravelCost(9007,n,inc):F0}");
}

if (AStar(g, 9007, 2274, true, out var p, out _))
{
    Console.WriteLine($"\n9007->2274: {Cost(g, p, true):F0}m uses 5334={Has(p, 9007, 5334)}");
    for (var i = 0; i < Math.Min(8, p.Count); i++) Console.WriteLine($"  wp{p[i]}");
}

return 0;

static bool Has(List<int> p, int a, int b) { for (var i = 1; i < p.Count; i++) if (p[i - 1] == a && p[i] == b) return true; return false; }
static bool AStar(IRoutingGraph g, int start, int goal, bool pen, out List<int> path, out int ex) { path = new(); ex = 0; var open = new List<int> { start }; var os = new HashSet<int> { start }; var cf = new Dictionary<int, int>(); var gs = new Dictionary<int, float> { { start, 0 } }; var fs = new Dictionary<int, float> { { start, H(g, start, goal) } }; var cl = new HashSet<int>(); while (open.Count > 0) { if (++ex > 16384) return false; var c = Pop(open, os, fs); if (c == goal) { Recon(cf, c, path); return true; } cl.Add(c); var inc = cf.TryGetValue(c, out var pv) ? pv : -1; var gc = gs[c]; Relax(g, c, inc, gc, goal, pen, g.GetForwardNeighbors(c), cl, open, os, cf, gs, fs); Relax(g, c, inc, gc, goal, pen, g.GetLaneChangeNeighbors(c), cl, open, os, cf, gs, fs); } return false; }
static void Relax(IRoutingGraph g, int c, int i, float gc, int goal, bool pen, ReadOnlySpan<int> ns, HashSet<int> cl, List<int> op, HashSet<int> os, Dictionary<int, int> cf, Dictionary<int, float> gs, Dictionary<int, float> fs) { for (var j = 0; j < ns.Length; j++) { var n = ns[j]; if (cl.Contains(n) || !g.IsForwardEdgeAllowed(i, c, n)) continue; var st = pen ? g.GetForwardTravelCost(c, n, i) : g.FlatDistance(g.GetPosition(c), g.GetPosition(n)); var t = gc + st; if (gs.TryGetValue(n, out var e) && t >= e) continue; cf[n] = c; gs[n] = t; fs[n] = t + H(g, n, goal); if (os.Add(n)) op.Add(n); } }
static float H(IRoutingGraph g, int a, int b) => g.FlatDistance(g.GetPosition(a), g.GetPosition(b));
static int Pop(List<int> o, HashSet<int> s, Dictionary<int, float> f) { var b = 0; float bf = float.MaxValue; for (var i = 0; i < o.Count; i++) { var v = f.TryGetValue(o[i], out var x) ? x : float.MaxValue; if (v < bf) { bf = v; b = i; } } var n = o[b]; o.RemoveAt(b); s.Remove(n); return n; }
static void Recon(Dictionary<int, int> cf, int c, List<int> p) { p.Add(c); while (cf.TryGetValue(c, out var pr)) { c = pr; p.Add(c); } p.Reverse(); }
static float Cost(IRoutingGraph g, List<int> p, bool pen) { if (p.Count < 2) return 0; float s = 0; var i = -1; for (var j = 1; j < p.Count; j++) { s += pen ? g.GetForwardTravelCost(p[j - 1], p[j], i) : g.FlatDistance(g.GetPosition(p[j - 1]), g.GetPosition(p[j])); i = p[j - 1]; } return s; }
