namespace MagicMotion
{
    /// <summary>
    /// Joint relation data ,containing constraint 's muslce joint and relative joint data
    /// </summary>
    public struct ParallelRelationData
    {
        public int relatedJointIndex;
        public int relatedMuscleIndex;

        public int parentJointIndex;
        public int jointIndex;
    }
}