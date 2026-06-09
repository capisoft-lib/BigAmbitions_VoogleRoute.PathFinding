namespace VoogleRoute.Pathfinding.Routing;



/// <summary>Pénalités de virage (mètres) ajoutées au coût A* aux intersections.</summary>

public static class TurnPenalties

{

    /// <summary>|angle| &lt; 22° = tout droit (pas de pénalité).</summary>

    public const float StraightMaxDegrees = 22f;



    /// <summary>Seuil demi-tour : au-delà, l'arête doit être un connecteur CSV autorisé.</summary>

    public const float UTurnBlockDegrees = 150f;



    /// <summary>Pénalité unique pour tout virage (|angle| ≥ 22°). À distance égale, moins de virages = optimal.</summary>

    public const float TurnMeters = 60f;

    /// <summary>Budget (m) par virage évité lors du choix entre candidats départ/arrivée.</summary>
    public const float SelectionMetersPerTurn = TurnMeters;

    public static float PenaltyMeters(float absDegrees) =>

        absDegrees < StraightMaxDegrees ? 0f : TurnMeters;

}

