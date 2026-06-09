using System;
using System.Collections.Generic;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Graph;

/// <summary>
/// Graphe routier + recherche de candidats — implémenté par le CSV (Blazor) et Gley (mod).
/// </summary>
public interface IRoutingGraph
{
    Vec3 GetPosition(int index);

    ReadOnlySpan<int> GetForwardNeighbors(int index);

    ReadOnlySpan<int> GetLaneChangeNeighbors(int index);

    bool IsLaneChangeEdge(int from, int to);

    bool IsSyntheticTurnEdge(int from, int to);

    bool TryGetSyntheticTurnControl(int from, int to, out Vec3 control);

    bool TryGetSyntheticTurnAbsAngle(int from, int to, out float absDegrees);

    bool IsForwardEdgeAllowed(int incoming, int at, int next);

    float GetForwardTravelCost(int from, int to, int incomingFrom);

    float EstimatePathTravelCost(IReadOnlyList<int> path);

    bool IsAuthorizedUturnEdge(int from, int to);

    bool IsIntersectionNode(int index);

    int CollectNearest(Vec3 worldPos, float maxDistance, int[] buffer);

    int CollectNearestAligned(Vec3 worldPos, Vec3 forward, float maxDistance, int[] buffer);

    int FilterFlowAligned(int[] buffer, int count, Vec3 forward);

    int ExpandLaneCandidates(int[] buffer, int count, int capacity, Vec3 forward);

    bool TryFindNearest(Vec3 worldPos, float maxDistance, out int index);

    float EstimateArrivalLegCost(int endIdx, Vec3 destination);

    float FlatDistance(Vec3 a, Vec3 b);
}
