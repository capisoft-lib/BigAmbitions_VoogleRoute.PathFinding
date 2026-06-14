using VoogleRoute.Pathfinding.Graph;

namespace VoogleRoute.Pathfinding.Routing;

/// <summary>Start-of-route maneuver rules (e.g. block immediate U-turn across the center line).</summary>
public static class StartManeuverPolicy
{
    /// <summary>First routing step cannot cross the center line (U-turn connector or lane change).</summary>
    public static bool IsBlockedManeuverAtStart(IRoutingGraph graph, int at, int next)
    {
        if (graph.IsAuthorizedUturnEdge(at, next))
            return true;

        if (graph.IsLaneChangeEdge(at, next))
            return true;

        if (!graph.IsSyntheticTurnEdge(at, next))
            return false;

        return graph.TryGetSyntheticTurnAbsAngle(at, next, out var abs) &&
               abs >= TurnPenalties.UTurnBlockDegrees;
    }
}
