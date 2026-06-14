using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Routing;

namespace VoogleRoute.Pathfinding.Tests;

/// <summary>Loads the shipped enhanced route graph once per test run.</summary>
public sealed class RouteGraphFixture : IDisposable
{
    public RouteGraph Graph { get; }

    public RouteGraphFixture()
    {
        var csv = ResolveGraphCsv();
        Graph = CsvRouteGraphLoader.LoadFromEnhancedCsv(csv);
    }

    public void Dispose()
    {
    }

    internal static string ResolveGraphCsv()
    {
        var candidates = new[]
        {
            Path.Combine(AppContext.BaseDirectory, "data", "big_ambitions_enhanced_routes.csv"),
            Path.Combine(
                FindPathFindingRoot(),
                "data",
                "big_ambitions_enhanced_routes.csv"),
        };

        foreach (var path in candidates)
        {
            if (File.Exists(path))
                return path;
        }

        throw new FileNotFoundException(
            "Enhanced route graph CSV not found. Expected under test output data/ or PathFinding/data/.");
    }

    private static string FindPathFindingRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir != null)
        {
            var candidate = Path.Combine(dir.FullName, "data", "big_ambitions_enhanced_routes.csv");
            if (File.Exists(candidate))
                return dir.FullName;

            if (dir.Name == "PathFinding")
                return dir.FullName;

            dir = dir.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate PathFinding/data from " + AppContext.BaseDirectory);
    }
}
