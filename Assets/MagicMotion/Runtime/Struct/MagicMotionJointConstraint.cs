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
    public enum MagicMotionConstraintType : short
    {
        None = 0,
        Position = 1,
        Rotation = 1 >> 1,
        LookAt = 1 >> 2,
    }
    /// <summary>
    /// The joint constraint.
    /// Can be read as position ,Rotation LookAt
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct MagicMotionJointConstraint
    {
        /// <summary>
        /// The target joint index;
        /// </summary>
        [FieldOffset(0)] public byte targetJointIndex;

        /// <summary>
        /// The constraint type
        /// </summary>
        [FieldOffset(1)] public MagicMotionConstraintType constraintType;

        /// <summary>
        /// The target position .overlapping with rotation
        /// </summary>
        [FieldOffset(1 + 2)] public float3 position;

        /// <summary>
        /// The target rotation .overlapping with position
        /// </summary>
        [FieldOffset(1 + 2)] public quaternion rotation;


        /// <summary>
        /// The target's position or eulers weight, range [0-1],0 means dont believe that. 
        /// </summary>
        [FieldOffset(1 + 2 + 16)] public float3 weight3;

        /// <summary>
        /// The target's rotation weight, range [0-1],0 means dont believe that. 
        /// </summary>
        [FieldOffset(1 + 2 + 16)] public float weight;
    }
    #endregion
}
