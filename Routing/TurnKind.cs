namespace VoogleRoute.Pathfinding.Routing;

public enum TurnKind
{
    Straight,
    Slight,
    Left,
    Right,
    UTurn,
    Sharp,
    Blocked
}

public sealed class PathTurn
{
    public required int FromIndex { get; init; }
    public required int AtIndex { get; init; }
    public required int ToIndex { get; init; }
    public float SignedDegrees { get; init; }
    public float PenaltyMeters { get; init; }
    public float EdgeMeters { get; init; }
    public TurnKind Kind { get; init; }
    public bool EdgeAllowed { get; init; } = true;
}
