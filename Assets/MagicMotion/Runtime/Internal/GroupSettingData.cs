using Unity.Mathematics;

namespace MagicMotion
{
    internal struct GroupSettingData
    {
        /// <summary>
        /// constraint length
        /// </summary>
        public readonly int constraintLength;
        /// <summary>
        ///  parallel's data count
        /// </summary>
        public readonly int parallelLength;
        /// <summary>
        ///  variable's count ,is the same as muscle's count.
        /// </summary>
        public readonly int muscleLength;
        /// <summary>
        /// joint count ,
        /// </summary>
        public readonly int jointLength;
        /// <summary>
        /// inner loop iteration count
        /// </summary>
        public int insideLoopCount;
        /// <summary>
        ///outside loop iteration index  
        /// </summary>
        public int outsideLoopIndex;
        /// <summary>
        /// outside loop iteration count  
        /// </summary>
        public int outsideLoopCount;
        /// <summary>
        /// rootTransform
        /// </summary>
        public RigidTransform rootTransform;

        public int LoopSum => insideLoopCount * outsideLoopCount;

        public double loopConvergence;
        internal int linerSearchGroupCount;

        public GroupSettingData(int constraintLength, int parallelLength, int muscleLength, int jointLength)
        {
            this.jointLength = jointLength; 
            this.constraintLength = constraintLength;   
            this.parallelLength = parallelLength;   
            this.muscleLength = muscleLength;

            insideLoopCount = 0;    
            outsideLoopIndex = 0;
            outsideLoopCount= 0;
            rootTransform = RigidTransform.identity;
            loopConvergence = 1;
            linerSearchGroupCount = 1;
        }

    }
}