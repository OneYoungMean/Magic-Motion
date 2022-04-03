using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion
{
    #region Joint
    public struct MagicMotionJoint
    {
        /// <summary>
        ///  parent index in heap
        /// </summary>
        public int parentIndex;
        /// <summary>
        /// son index start in heap
        /// </summary>
        public int childIndexStart;
        /// <summary>
        /// son index end in heap
        /// they are continuous in the heap
        /// </summary>
        public int childIndexEnd;
        /// <summary>
        /// if is Position Joint mode ,it means move Range ,else means euler angle;
        /// </summary>
        public float3 MaxRange;
        /// <summary>
        /// if is Position Joint mode ,it means move Range ,else means euler angle;
        /// </summary>
        public float3 MinRange;

        /// <summary>
        /// initial localPosition from parent,used be clac currentPosition
        /// </summary>
        public float3 localPosition;
        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion localRotation;

        public bool isNoChild => childIndexStart < childIndexEnd;
    }

    public enum JointMode
    {
        Position, Rotation
    }
    #endregion
}
