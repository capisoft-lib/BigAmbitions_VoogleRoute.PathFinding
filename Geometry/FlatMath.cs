namespace VoogleRoute.Pathfinding.Geometry;

public readonly struct Vec3(float x, float y, float z)
{
    public float X { get; } = x;
    public float Y { get; } = y;
    public float Z { get; } = z;

    public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Vec3 operator *(Vec3 a, float s) => new(a.X * s, a.Y * s, a.Z * s);

    public float SqrMagnitude
    {
        get
        {
            var dx = X;
            var dy = Y;
            var dz = Z;
            return dx * dx + dy * dy + dz * dz;
        }
    }

    public static float FlatLength(Vec3 a, Vec3 b)
    {
        var dx = a.X - b.X;
        var dz = a.Z - b.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
    }

    public static float FlatDistSq(Vec3 a, Vec3 b)
    {
        var dx = a.X - b.X;
        var dz = a.Z - b.Z;
        return dx * dx + dz * dz;
    }

    public static Vec3 Lerp(Vec3 a, Vec3 b, float t) =>
        new(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);

    public static Vec3 FlatDir(Vec3 from, Vec3 to)
    {
        var d = to - from;
        var lenSq = d.X * d.X + d.Z * d.Z;
        if (lenSq < 0.01f)
            return new Vec3(0, 0, 0);

        var inv = 1f / MathF.Sqrt(lenSq);
        return new Vec3(d.X * inv, 0, d.Z * inv);
    }

    /// <summary>Positive = left turn (Unity Vector3.SignedAngle with Vector3.up).</summary>
    public static float SignedTurnDegrees(Vec3 incoming, Vec3 at, Vec3 to)
    {
        var inDir = FlatDir(incoming, at);
        var outDir = FlatDir(at, to);
        if (inDir.SqrMagnitude < 0.01f || outDir.SqrMagnitude < 0.01f)
            return 0f;

        var cross = inDir.X * outDir.Z - inDir.Z * outDir.X;
        var dot = inDir.X * outDir.X + inDir.Z * outDir.Z;
        return MathF.Atan2(cross, dot) * (180f / MathF.PI);
    }

    public static float DeltaAngle(float a, float b)
    {
        var delta = (b - a + 540f) % 360f;
        if (delta < 0f)
            delta += 360f;
        delta -= 180f;
        return MathF.Abs(delta);
    }

    public static float BearingDeg(Vec3 from, Vec3 to)
    {
        var dx = to.X - from.X;
        var dz = to.Z - from.Z;
        return MathF.Atan2(dx, dz) * (180f / MathF.PI);
    }
}
