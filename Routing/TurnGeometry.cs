using System;

using VoogleRoute.Pathfinding.Geometry;

using VoogleRoute.Pathfinding.Graph;



namespace VoogleRoute.Pathfinding.Routing;



/// <summary>

/// Angle de virage entre voies (axes de circulation), pas la corde synthetic_turn at→to.

/// </summary>

public static class TurnGeometry

{

    public static float SignedLaneTurnDegrees(

        IRoutingGraph graph,

        int incoming,

        int at,

        int to,

        int afterTo = -1)

    {

        var inPos = graph.GetPosition(incoming);

        var atPos = graph.GetPosition(at);

        var toPos = graph.GetPosition(to);



        var inDir = Vec3.FlatDir(inPos, atPos);

        var outDir = afterTo >= 0

            ? Vec3.FlatDir(toPos, graph.GetPosition(afterTo))

            : ResolveOutgoingLaneDir(graph, at, to);



        return SignedAngleDegrees(inDir, outDir);

    }



    public static float AbsLaneTurnDegrees(

        IRoutingGraph graph,

        int incoming,

        int at,

        int to,

        int afterTo = -1) =>

        MathF.Abs(SignedLaneTurnDegrees(graph, incoming, at, to, afterTo));



    public static float SignedLaneTurnDegrees(

        ReadOnlySpan<Vec3> positions,

        int[][] forward,

        int incoming,

        int at,

        int to,

        int afterTo = -1)

    {

        var inDir = Vec3.FlatDir(positions[incoming], positions[at]);

        var outDir = afterTo >= 0

            ? Vec3.FlatDir(positions[to], positions[afterTo])

            : ResolveOutgoingLaneDir(positions, forward, at, to);

        return SignedAngleDegrees(inDir, outDir);

    }



    public static float AbsLaneTurnDegrees(

        ReadOnlySpan<Vec3> positions,

        int[][] forward,

        int incoming,

        int at,

        int to,

        int afterTo = -1) =>

        MathF.Abs(SignedLaneTurnDegrees(positions, forward, incoming, at, to, afterTo));



    private static Vec3 ResolveOutgoingLaneDir(IRoutingGraph graph, int at, int to)

    {

        var posAt = graph.GetPosition(at);

        var posTo = graph.GetPosition(to);

        var edgeHint = Vec3.FlatDir(posAt, posTo);

        var neighbors = graph.GetForwardNeighbors(to);

        if (neighbors.Length == 0)

            return edgeHint.SqrMagnitude > 0.01f ? edgeHint : new Vec3(0, 0, 0);



        var bestAlign = float.NegativeInfinity;

        var bestDir = new Vec3(0, 0, 0);

        var bestLen = -1f;

        var fallbackDir = new Vec3(0, 0, 0);

        var hasHint = edgeHint.SqrMagnitude > 0.01f;



        for (var i = 0; i < neighbors.Length; i++)

        {

            var posN = graph.GetPosition(neighbors[i]);

            var dir = Vec3.FlatDir(posTo, posN);

            if (dir.SqrMagnitude < 0.01f)

                continue;



            var len = Vec3.FlatLength(posTo, posN);

            if (len > bestLen)

            {

                bestLen = len;

                fallbackDir = dir;

            }



            if (!hasHint)

                continue;



            var align = dir.X * edgeHint.X + dir.Z * edgeHint.Z;

            if (align <= bestAlign)

                continue;



            bestAlign = align;

            bestDir = dir;

        }



        if (hasHint && bestAlign > -0.15f)

            return bestDir;



        return fallbackDir.SqrMagnitude > 0.01f ? fallbackDir : edgeHint;

    }



    private static Vec3 ResolveOutgoingLaneDir(ReadOnlySpan<Vec3> positions, int[][] forward, int at, int to)

    {

        var posAt = positions[at];

        var posTo = positions[to];

        var edgeHint = Vec3.FlatDir(posAt, posTo);

        var neighbors = forward[to];

        if (neighbors == null || neighbors.Length == 0)

            return edgeHint.SqrMagnitude > 0.01f ? edgeHint : new Vec3(0, 0, 0);



        var bestAlign = float.NegativeInfinity;

        var bestDir = new Vec3(0, 0, 0);

        var bestLen = -1f;

        var fallbackDir = new Vec3(0, 0, 0);

        var hasHint = edgeHint.SqrMagnitude > 0.01f;



        for (var i = 0; i < neighbors.Length; i++)

        {

            var n = neighbors[i];

            var dir = Vec3.FlatDir(posTo, positions[n]);

            if (dir.SqrMagnitude < 0.01f)

                continue;



            var len = Vec3.FlatLength(posTo, positions[n]);

            if (len > bestLen)

            {

                bestLen = len;

                fallbackDir = dir;

            }



            if (!hasHint)

                continue;



            var align = dir.X * edgeHint.X + dir.Z * edgeHint.Z;

            if (align <= bestAlign)

                continue;



            bestAlign = align;

            bestDir = dir;

        }



        if (hasHint && bestAlign > -0.15f)

            return bestDir;



        return fallbackDir.SqrMagnitude > 0.01f ? fallbackDir : edgeHint;

    }



    private static float SignedAngleDegrees(Vec3 inDir, Vec3 outDir)

    {

        if (inDir.SqrMagnitude < 0.01f || outDir.SqrMagnitude < 0.01f)

            return 0f;



        var cross = inDir.X * outDir.Z - inDir.Z * outDir.X;

        var dot = inDir.X * outDir.X + inDir.Z * outDir.Z;

        return MathF.Atan2(cross, dot) * (180f / MathF.PI);

    }

}

