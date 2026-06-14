using System.Globalization;
using System.IO;

namespace VoogleRoute.Pathfinding.Routing.Foot;

public static class CsvSubwayStationLoader
{
    public static IReadOnlyList<SubwayStation> LoadFromCsv(string csvPath)
    {
        if (!File.Exists(csvPath))
            return Array.Empty<SubwayStation>();

        var stations = new List<SubwayStation>();
        using var reader = new StreamReader(csvPath);
        var header = reader.ReadLine();
        if (string.IsNullOrWhiteSpace(header))
            return Array.Empty<SubwayStation>();

        string? line;
        while ((line = reader.ReadLine()) != null)
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith("#", StringComparison.Ordinal))
                continue;

            var parts = line.Split(',');
            if (parts.Length < 8)
                continue;

            stations.Add(new SubwayStation
            {
                Index = stations.Count,
                StationName = parts[0].Trim(),
                Neighborhood = parts[1].Trim(),
                WorldPosition = ReadVec3(parts, 2),
                NavPosition = ReadVec3(parts, 5)
            });
        }

        return stations;
    }

    private static Geometry.Vec3 ReadVec3(string[] parts, int start) =>
        new(
            float.Parse(parts[start], CultureInfo.InvariantCulture),
            float.Parse(parts[start + 1], CultureInfo.InvariantCulture),
            float.Parse(parts[start + 2], CultureInfo.InvariantCulture));
}
