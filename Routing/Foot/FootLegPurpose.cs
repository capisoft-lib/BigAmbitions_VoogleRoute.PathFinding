namespace VoogleRoute.Pathfinding.Routing.Foot;

/// <summary>
/// Direct-to-destination legs must be vanilla-complete (PathComplete).
/// Connector legs reach subway stations and may use relaxed NavMesh sampling.
/// </summary>
public enum FootLegPurpose
{
    DirectToDestination,
    Connector
}
