namespace VoogleRoute.Pathfinding.Geometry;

public enum HamptonsBoundaryPathStatus
{
    Invalid = 0,
    Partial = 1,
    Complete = 2,
}

/// <summary>
/// Hamptons plots switch the player from the indoor agent to the city agent at
/// the property boundary. A partial indoor path is therefore usable only when
/// its last corner actually reaches that boundary seam.
/// </summary>
public static class HamptonsBoundaryPathPolicy
{
    public const float MaxPartialEndpointToTarget = 2.75f;
    public const float MaxPartialEndpointToBoundary = 1.5f;

    public static bool IsUsable(
        HamptonsBoundaryPathStatus status,
        int cornerCount,
        float endpointToTarget,
        float endpointToBoundary)
    {
        if (cornerCount < 2 || status == HamptonsBoundaryPathStatus.Invalid)
            return false;

        if (status == HamptonsBoundaryPathStatus.Complete)
            return true;

        return endpointToTarget <= MaxPartialEndpointToTarget &&
               endpointToBoundary <= MaxPartialEndpointToBoundary;
    }
}
