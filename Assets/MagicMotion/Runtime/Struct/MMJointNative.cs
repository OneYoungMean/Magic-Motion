using Unity.Mathematics;

namespace MagicMotion
{
    public struct MMJointNative
    {
        /// <summary>
        ///  Is the joint vaild?
        /// </summary>
        public bool isVaild;

        /// <summary>
        ///  parent index in heap
        /// </summary>
        public int parentIndex;
        /*        /// <summary>
                /// son index start in heap
                /// </summary>
                public int childIndexStart;
                /// <summary>
                /// son index end in heap
                /// they are continuous in the heap
                /// </summary>
                public int childIndexEnd;*/
        /// <summary>
        /// if is Position Joint mode ,it means move Range ,else means euler angle;
        /// </summary>
        public float3 maxRange;
        /// <summary>
        /// if is Position Joint mode ,it means move Range ,else means euler angle;
        /// </summary>
        public float3 minRange;

        /// <summary>
        /// initial localPosition from parent,used be clac currentPosition
        /// </summary>
        public float3 localPosition;
        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion localRotation;

        /// <summary>
        /// the dof3 axis
        /// </summary>
        public float3x3 dof3Axis;
        /*        /// <summary>
                /// Dof3 to euler angle to localrotation's value ,I dont know why does it exist.
                /// Dof3Rotation =math.inverse(dof3RotateOffset) *quaternion.euler(Dof3)* dof3RotateOffset
                /// </summary>
                public quaternion dof3RotateOffset;*/


        /*
                public bool isNoChild => childIndexStart < childIndexEnd;*/
    }
}