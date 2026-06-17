using System;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing.Foot;

/// <summary>
/// Outdoor foot routing: vanilla-complete direct walk when possible; subway only as fallback.
/// </summary>
public static class FootSubwayRoutePlanner
{
    public static bool TryBuildRoute(
        Vec3 origin,
        Vec3 target,
        Vec3 sampleOrigin,
        IFootPathProvider footPaths,
        IReadOnlyList<SubwayStation> stations,
        SubwayNetwork subwayNetwork,
        FootRouteOptions options,
        out FootRouteResult result)
    {
        result = FootRouteResult.None;

        if (TryBuildDirect(origin, target, sampleOrigin, footPaths, options, out var direct))
        {
            result = direct;
            return true;
        }

        if (!options.AllowSubwayPlanning || !options.UseSubwayEnabled || stations.Count == 0)
            return false;

        if (!TryBuildViaSubway(
                origin, target, sampleOrigin, footPaths, stations, subwayNetwork, options, out var subway))
            return false;

        result = subway;
        return true;
    }

    private static float MeasureFootMeters(FootRouteResult path)
    {
        if (path.Segments.Count > 0)
        {
            var total = 0f;
            for (var i = 0; i < path.Segments.Count; i++)
            {
                if (path.Segments[i].Kind != FootRouteSegmentKind.Foot)
                    continue;

                var points = path.Segments[i].Points;
                if (points.Count >= 2)
                    total += RoutePolylineMetrics.FlatLength(points);
            }

            if (total > 0f)
                return total;
        }

        return path.Points.Count >= 2 ? RoutePolylineMetrics.FlatLength(path.Points) : 0f;
    }

    private static bool TryBuildDirect(
        Vec3 origin,
        Vec3 target,
        Vec3 sampleOrigin,
        IFootPathProvider footPaths,
        FootRouteOptions options,
        out FootRouteResult result)
    {
        result = FootRouteResult.None;

        if (!footPaths.TryBuildFootLeg(
                origin, target, sampleOrigin, FootLegPurpose.DirectToDestination, out var leg) ||
            !leg.Success)
            return false;

        if (leg.IsPartial)
            return false;

        if (leg.Points.Count < 2)
            return false;

        result = new FootRouteResult
        {
            Success = true,
            IsPartial = leg.IsPartial,
            Points = leg.Points,
            Segments = new[]
            {
                new FootRouteSegment { Kind = FootRouteSegmentKind.Foot, Points = leg.Points }
            },
            Subway = FootSubwayHint.None
        };
        return true;
    }

    private static bool TryBuildViaSubway(
        Vec3 origin,
        Vec3 target,
        Vec3 sampleOrigin,
        IFootPathProvider footPaths,
        IReadOnlyList<SubwayStation> stations,
        SubwayNetwork subwayNetwork,
        FootRouteOptions options,
        out FootRouteResult result)
    {
        result = FootRouteResult.None;

        var boardCandidates = CollectNearestCandidates(origin, stations, options);
        var exitCandidates = CollectNearestCandidates(target, stations, options);
        if (boardCandidates.Count == 0 || exitCandidates.Count == 0)
            return false;

        var bestWalkMeters = float.PositiveInfinity;
        SubwayStation? bestBoard = null;
        SubwayStation? bestExit = null;
        IReadOnlyList<Vec3>? bestWalkToBoard = null;
        IReadOnlyList<Vec3>? bestWalkFromExit = null;
        IReadOnlyList<Vec3>? bestSubwayDisplay = null;
        var bestPartial = false;

        for (var bi = 0; bi < boardCandidates.Count; bi++)
        {
            var board = boardCandidates[bi];
            if (!footPaths.TryBuildFootLeg(
                    origin, board.NavPosition, sampleOrigin, FootLegPurpose.Connector, out var toBoard) ||
                !toBoard.Success ||
                (toBoard.IsPartial && !options.ShowPartialPaths) ||
                toBoard.Points.Count < 2)
                continue;

            var walkToBoardLen = toBoard.WalkMeters;

            for (var ei = 0; ei < exitCandidates.Count; ei++)
            {
                var exit = exitCandidates[ei];
                if (board.StationName == exit.StationName)
                    continue;

                if (!footPaths.TryBuildFootLeg(
                        exit.NavPosition, target, exit.NavPosition, FootLegPurpose.Connector, out var fromExit) ||
                    !fromExit.Success ||
                    (fromExit.IsPartial && !options.ShowPartialPaths) ||
                    fromExit.Points.Count < 2)
                    continue;

                var subwayDisplay = subwayNetwork.BuildDisplayPath(board, exit);
                if (subwayDisplay.Count < 2)
                    continue;

                var walkOnlyMeters = walkToBoardLen + fromExit.WalkMeters;
                if (walkOnlyMeters >= bestWalkMeters)
                    continue;

                bestWalkMeters = walkOnlyMeters;
                bestBoard = board;
                bestExit = exit;
                bestWalkToBoard = toBoard.Points;
                bestWalkFromExit = fromExit.Points;
                bestSubwayDisplay = subwayDisplay;
                bestPartial = toBoard.IsPartial || fromExit.IsPartial;
            }
        }

        if (bestBoard == null || bestExit == null || bestWalkToBoard == null || bestWalkFromExit == null)
            return false;

        var segments = new[]
        {
            new FootRouteSegment { Kind = FootRouteSegmentKind.Foot, Points = bestWalkToBoard },
            new FootRouteSegment { Kind = FootRouteSegmentKind.Subway, Points = bestSubwayDisplay! },
            new FootRouteSegment { Kind = FootRouteSegmentKind.Foot, Points = bestWalkFromExit }
        };

        result = new FootRouteResult
        {
            Success = true,
            IsPartial = bestPartial,
            Points = ConcatenateSegments(segments),
            Segments = segments,
            Subway = new FootSubwayHint
            {
                Active = true,
                BoardStationName = bestBoard.StationName,
                ExitStationName = bestExit.StationName,
                BoardNavPosition = bestBoard.NavPosition,
                ExitNavPosition = bestExit.NavPosition,
                BoardWorldPosition = bestBoard.WorldPosition,
                ExitWorldPosition = bestExit.WorldPosition
            }
        };
        return true;
    }

    private static List<SubwayStation> CollectNearestCandidates(
        Vec3 worldPos,
        IReadOnlyList<SubwayStation> stations,
        FootRouteOptions options)
    {
        var ranked = new List<(SubwayStation Station, float Distance)>(stations.Count);
        for (var i = 0; i < stations.Count; i++)
        {
            var station = stations[i];
            var distance = station.HorizontalDistanceTo(worldPos);
            if (distance > options.MaxStationPickMeters)
                continue;

            ranked.Add((station, distance));
        }

        ranked.Sort((a, b) => a.Distance.CompareTo(b.Distance));

        var result = new List<SubwayStation>(options.MaxStationCandidates);
        for (var i = 0; i < ranked.Count && result.Count < options.MaxStationCandidates; i++)
            result.Add(ranked[i].Station);

        return result;
    }

    private static IReadOnlyList<Vec3> ConcatenateSegments(IReadOnlyList<FootRouteSegment> segments)
    {
        var total = 0;
        for (var i = 0; i < segments.Count; i++)
            total += segments[i].Points.Count;

        if (total < 2)
            return Array.Empty<Vec3>();

        var merged = new List<Vec3>(total);
        for (var i = 0; i < segments.Count; i++)
        {
            var points = segments[i].Points;
            for (var p = 0; p < points.Count; p++)
            {
                if (merged.Count > 0 && Vec3.FlatDistSq(points[p], merged[^1]) < 0.04f)
                    continue;

                merged.Add(points[p]);
            }
        }

        return merged.Count >= 2 ? merged : Array.Empty<Vec3>();
    }
}
