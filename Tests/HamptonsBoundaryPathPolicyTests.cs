using VoogleRoute.Pathfinding.Geometry;
using Xunit;

namespace VoogleRoute.Pathfinding.Tests;

public class HamptonsBoundaryPathPolicyTests
{
    [Fact]
    public void CompletePathWithCornersIsUsable()
    {
        Assert.True(HamptonsBoundaryPathPolicy.IsUsable(
            HamptonsBoundaryPathStatus.Complete,
            cornerCount: 2,
            endpointToTarget: 100f,
            endpointToBoundary: 100f));
    }

    [Fact]
    public void PartialPathReachingBoundarySeamIsUsable()
    {
        Assert.True(HamptonsBoundaryPathPolicy.IsUsable(
            HamptonsBoundaryPathStatus.Partial,
            cornerCount: 3,
            endpointToTarget: 0.9f,
            endpointToBoundary: 0.2f));
    }

    [Theory]
    [InlineData(1, 0.1f, 0.1f)]
    [InlineData(2, 2.76f, 0.1f)]
    [InlineData(2, 0.1f, 1.51f)]
    public void PartialPathThatDoesNotReachBoundarySeamIsRejected(
        int cornerCount,
        float endpointToTarget,
        float endpointToBoundary)
    {
        Assert.False(HamptonsBoundaryPathPolicy.IsUsable(
            HamptonsBoundaryPathStatus.Partial,
            cornerCount,
            endpointToTarget,
            endpointToBoundary));
    }

    [Fact]
    public void InvalidPathIsRejected()
    {
        Assert.False(HamptonsBoundaryPathPolicy.IsUsable(
            HamptonsBoundaryPathStatus.Invalid,
            cornerCount: 4,
            endpointToTarget: 0f,
            endpointToBoundary: 0f));
    }
}
