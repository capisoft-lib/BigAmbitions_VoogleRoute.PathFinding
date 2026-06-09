using System;
using VoogleRoute.Pathfinding.Geometry;

namespace VoogleRoute.Pathfinding.Routing;

/// <summary>
/// Longueur de manœuvre aux carrefours — jamais la corde diagonale minimale des synthetic_turn.
/// </summary>
public static class ManeuverGeometry
{
    public const int BezierSegments = 12;

    /// <summary>
    /// Distance de parcours pour un synthetic_turn : max(corde, L via contrôle, arc Bézier).
    /// Le contrôle CSV est l'intersection des axes de voie au carrefour.
    /// </summary>
    public static float SyntheticTurnTravelMeters(Vec3 from, Vec3 to, Vec3 control)
    {
        var chord = Vec3.FlatLength(from, to);
        var viaControl = Vec3.FlatLength(from, control) + Vec3.FlatLength(control, to);
        var bezier = QuadraticBezierFlatLength(from, control, to, BezierSegments);
        return MathF.Max(chord, MathF.Max(viaControl, bezier));
    }

    public static float QuadraticBezierFlatLength(Vec3 p0, Vec3 p1, Vec3 p2, int segments)
    {
        if (segments < 2)
            segments = 2;

        var prev = p0;
        var len = 0f;
        for (var s = 1; s <= segments; s++)
        {
            var t = s / (float)segments;
            var u = 1f - t;
            var pt = new Vec3(
                u * u * p0.X + 2f * u * t * p1.X + t * t * p2.X,
                0f,
                u * u * p0.Z + 2f * u * t * p1.Z + t * t * p2.Z);
            len += Vec3.FlatLength(prev, pt);
            prev = pt;
        }

        return len;
    }
}
