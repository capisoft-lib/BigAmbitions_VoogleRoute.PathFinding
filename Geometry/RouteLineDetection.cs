namespace VoogleRoute.Pathfinding.Geometry;

/// <summary>
/// Cross-track margins before the mod recalculates the GPS route line.
/// Vehicle: locked-route follow tolerance. Foot: origin-move recalc radius.
/// </summary>
public static class RouteLineDetection
{
    public const float VehicleCrossTrackMeters = 14f;
    public const float FootCrossTrackMeters = 15f;

    public static float GetCrossTrackMeters(bool isVehicle) =>
        isVehicle ? VehicleCrossTrackMeters : FootCrossTrackMeters;
}
