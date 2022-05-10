using Unity.Mathematics;

namespace MagicMotion
{
    public enum JointKind
    {
        LeftHand,
        RightHand,
        Body
    }
    internal struct JointData
    {
        /// <summary>
        ///  Is the joint vaild?
        /// </summary>
        public bool isVaild;

        /// <summary>
        ///  parent index in heap
        /// </summary>
        public int parentIndex;
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
        /// initial localPosition 's Length
        /// </summary>
        public float length;
        /// <summary>
        /// the dof3 axis
        /// </summary>
        public float3x3 dof3Axis;
    }
}