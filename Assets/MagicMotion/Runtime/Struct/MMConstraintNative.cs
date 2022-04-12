using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion
{
    #region JointConstraint
    /// <summary>
    /// the joint Constraint Type
    /// </summary>
    public enum MMConstraintType : byte
    {
        Position = 0,
        Rotation = 1,
        LookAt = 2,
    }
    /// <summary>
    /// The joint constraint.
    /// Can be read as position ,Rotation LookAt
    /// </summary>
    //[StructLayout(LayoutKind.Explicit)]
    public struct MMConstraintNative
    {
        /// <summary>
        /// The constraint type
        /// </summary>
        //[FieldOffset(0)] 
        public MMConstraintType constraintType;

        /// <summary>
        /// The target joint index;
        /// </summary>
        //[FieldOffset(1)] 
        public int targetJointIndex;

        /// <summary>
        /// The target's position or eulers weight, range [0-1],0 means dont believe that. 
        /// </summary>
        //[FieldOffset(1 + 4 )] 
        public float3 weight3;
        /// <summary>
        /// The target's position or eulers weight, range [0-1],0 means dont believe that. 
        /// </summary>
        //[FieldOffset(1 + 4+12)] 
        public float3 torlerace3;

        /// <summary>
        /// The target position .overlapping with rotation
        /// </summary>
        //[FieldOffset(1 + 4 + 12+12)]
        public float3 position;

        /// <summary>
        /// The target rotation .overlapping with position
        /// </summary>
        //[FieldOffset(1 + 4 + 12 + 12)]
        public quaternion rotation;
    }
    #endregion
}
