namespace MagicMotion
{
    internal struct MuscleData
    {
        /// <summary>
        /// the muscle index,range 0-95
        /// </summary>
        public int muscleIndex;
        /// <summary>
        /// muscle to joint index ,range0-55
        /// </summary>
        public int jointIndex;
        /// <summary>
        /// muscle dof range 0-2
        /// </summary>
        public int dof;
    }
}