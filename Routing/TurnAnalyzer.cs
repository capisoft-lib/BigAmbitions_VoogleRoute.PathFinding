using System;
using VoogleRoute.Pathfinding.Graph;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing;

public static class TurnAnalyzer
{
    public static IReadOnlyList<PathTurn> AnalyzePath(IRoutingGraph graph, IReadOnlyList<int> path)
    {
        if (path.Count < 2)
            return Array.Empty<PathTurn>();

        var turns = new List<PathTurn>(System.Math.Max(0, path.Count - 2));
        var incoming = -1;
        for (var i = 0; i < path.Count - 1; i++)
        {
            var from = path[i];
            var to = path[i + 1];
            if (incoming >= 0)
            {
                var afterTo = i + 2 < path.Count ? path[i + 2] : -1;
                var signed = SignedTurnDegrees(graph, incoming, from, to, afterTo);
                var penalty = GetTurnPenalty(graph, incoming, from, to, afterTo);
                turns.Add(new PathTurn
                {
                    FromIndex = incoming,
                    AtIndex = from,
                    ToIndex = to,
                    SignedDegrees = signed,
                    PenaltyMeters = penalty,
                    EdgeMeters = Vec3.FlatLength(graph.GetPosition(from), graph.GetPosition(to)),
                    Kind = Classify(signed),
                    EdgeAllowed = graph.IsForwardEdgeAllowed(incoming, from, to)
                });
            }

            incoming = from;
        }

        return turns;
    }

    /// <summary>Nombre de virages payants (|angle| ≥ 22°) sur un chemin — pour le critère de sélection.</summary>
    public static int CountPaidTurns(IRoutingGraph graph, IReadOnlyList<int> path)
    {
        if (path.Count < 3)
            return 0;

        var count = 0;
        var incoming = -1;
        for (var i = 0; i < path.Count - 1; i++)
        {
            var from = path[i];
            var to = path[i + 1];
            if (incoming >= 0 && graph.IsIntersectionNode(from))
            {
                var afterTo = i + 2 < path.Count ? path[i + 2] : -1;
                if (AbsTurnDegrees(graph, incoming, from, to, afterTo) >= TurnPenalties.StraightMaxDegrees)
                    count++;
            }

            incoming = from;
        }

        return count;
    }

    public static TurnSummary Summarize(IReadOnlyList<PathTurn> turns) =>

        new(
            turns.Count(t => t.Kind == TurnKind.Left),
            turns.Count(t => t.Kind == TurnKind.Right),
            turns.Count(t => t.Kind == TurnKind.Straight),
            turns.Count(t => t.Kind == TurnKind.UTurn),
            turns.Count(t => t.Kind == TurnKind.Sharp),
            turns.Count(t => !t.EdgeAllowed),
            turns.Sum(t => t.PenaltyMeters));

    private static TurnKind Classify(float signedDegrees)
    {
        var abs = MathF.Abs(signedDegrees);
        if (abs < TurnPenalties.StraightMaxDegrees)
            return TurnKind.Straight;
        if (abs >= TurnPenalties.UTurnBlockDegrees)
            return TurnKind.UTurn;
        return signedDegrees > 0f ? TurnKind.Left : TurnKind.Right;
    }

    private static float GetTurnPenalty(
        IRoutingGraph graph, int incoming, int at, int to, int afterTo = -1)
    {
        if (!graph.IsIntersectionNode(at))
            return 0f;

        return TurnPenalties.PenaltyMeters(AbsTurnDegrees(graph, incoming, at, to, afterTo));
    }

    private static float SignedTurnDegrees(
        IRoutingGraph graph, int incoming, int at, int to, int afterTo = -1)
    {
        if (graph.IsSyntheticTurnEdge(at, to) &&
            graph.TryGetSyntheticTurnAbsAngle(at, to, out var csvAbs))
            return csvAbs;

        return TurnGeometry.SignedLaneTurnDegrees(graph, incoming, at, to, afterTo);
    }

    private static float AbsTurnDegrees(
        IRoutingGraph graph, int incoming, int at, int to, int afterTo = -1) =>
        MathF.Abs(SignedTurnDegrees(graph, incoming, at, to, afterTo));
}

public readonly record struct TurnSummary(
    int Left,
    int Right,
    int Straight,
    int UTurn,
    int Sharp,
    int Blocked,
    float TotalPenaltyMeters);
