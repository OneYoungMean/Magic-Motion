using Unity.Mathematics;

namespace MagicMotion
{
    /// <summary>
    /// Transform to ConstraintData
    /// </summary>

    public struct JointTransformData
    {
        RigidTransform transform;
        RigidTransform parentTransform;
    }
}