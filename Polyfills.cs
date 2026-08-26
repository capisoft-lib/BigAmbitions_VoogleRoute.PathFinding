// Enables init/required/records on netstandard2.1 (Unity + shared lib).
using System;

namespace System.Runtime.CompilerServices
{
    internal static class IsExternalInit { }

    [AttributeUsage(AttributeTargets.Class | AttributeTargets.Struct | AttributeTargets.Property | AttributeTargets.Field, Inherited = false)]
    internal sealed class RequiredMemberAttribute : Attribute { }

    [AttributeUsage(AttributeTargets.All, AllowMultiple = true, Inherited = false)]
    internal sealed class CompilerFeatureRequiredAttribute : Attribute
    {
        public CompilerFeatureRequiredAttribute(string featureName) => FeatureName = featureName;
        public string FeatureName { get; }
    }
}

#if BA_PLAYER_RUNTIME
namespace VoogleRoute.Pathfinding
{
    /// <summary>MathF surface missing from the Mono profile embedded by Big Ambitions.</summary>
    internal static class MathF
    {
        internal const float PI = 3.14159265358979323846f;

        internal static float Sqrt(float value) => (float)System.Math.Sqrt(value);
        internal static float Sin(float value) => (float)System.Math.Sin(value);
        internal static float Cos(float value) => (float)System.Math.Cos(value);
        internal static float Acos(float value) => (float)System.Math.Acos(value);
        internal static float Atan2(float y, float x) => (float)System.Math.Atan2(y, x);
        internal static float Abs(float value) => value < 0f ? -value : value;
        internal static float Min(float x, float y) => x < y ? x : y;
        internal static float Max(float x, float y) => x > y ? x : y;
    }
}

namespace System
{
    /// <summary>Compiler support for the ^index syntax on the Big Ambitions Mono profile.</summary>
    public readonly struct Index
    {
        private readonly int _value;
        private readonly bool _fromEnd;

        public Index(int value, bool fromEnd = false)
        {
            if (value < 0)
                throw new ArgumentOutOfRangeException(nameof(value));
            _value = value;
            _fromEnd = fromEnd;
        }

        public int GetOffset(int length) => _fromEnd ? length - _value : _value;
    }
}

namespace System.IO
{
    /// <summary>Compatibility exception absent from the Big Ambitions Mono profile.</summary>
    public class InvalidDataException : IOException
    {
        public InvalidDataException() { }
        public InvalidDataException(string message) : base(message) { }
        public InvalidDataException(string message, Exception innerException) : base(message, innerException) { }
    }
}
#endif
