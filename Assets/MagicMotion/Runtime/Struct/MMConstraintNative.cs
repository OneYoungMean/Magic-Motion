using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion
{
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
    public struct MMJointConstraintNative
    {
        public MMPositionConstraint positionConstraint;
        public MMMuscleConstraint muscleConstraint;
        public MMLookAtConstraint lookAtConstraint;
        public MMColliderConstraint colliderConstraint;
        public MMPositionChangeConstraint positionChangeConstraint;
        public MMMuscleChangeConstraint muscleChangeConstraint;
    }
    public struct MMJoinFitness
    {
        public float positionFitness;
        public float muscleFitness;
        public float lookAtFitness;
        public float colliderFitness;
        public float positionChangeFitness;
        public float muscleChangeFitness;
    }

    public struct MMPositionConstraint
    {
        public float3 position;
        public float3 weight3;
        public float3 tolerance3;
        public bool isVaild
        {
            get { return math.all(weight3 == 0); }
        }
    }

    public struct MMMuscleConstraint
    {
        public float3 tolerance3;
        public float3 weight3;
        public bool isVaild
        {
            get { return math.all(tolerance3 == 1); }
        }
    }

    public struct MMLookAtConstraint
    {
        public float3 position;
        public float3 direction;
        public float weight;
        public float tolerance;
        public bool isVaild
        {
            get { return weight == 0; }
        }
    }

    public struct MMColliderConstraint
    {
        public float3 localPosition;
        public float3 localDirection;
        public float radius;
        public float length;
        public float weight;
        public bool isVaild
        {
            get { return weight == 0; }
        }
    }

    public struct MMPositionChangeConstraint
    {
        public float3 oldPosition;
        public float3 tolerance3;
        public float3 weight3;
        public bool isVaild
        {
            get { return math.all(weight3 == 0); }
        }
    }
    public struct MMMuscleChangeConstraint
    {
        public float3 oldDof3;
        public float3 torlerence3;
        public float3 weight3;
        public bool isVaild
        {
            get { return math.all(weight3 == 0); }
        }
    }

    public struct MMGlobalConstraint
    {
        public bool isBodyBalanceConstraint;//OYM£ºfuture
    }

}
//[StructLayout(LayoutKind.Explicit)]
//public struct mmconstraintnative
//{
//    /// <summary>
//    /// the constraint type
//    /// </summary>
//    //[fieldoffset(0)] 
//    public mmconstrainttype constrainttype;

//    /// <summary>
//    /// the target joint index;
//    /// </summary>
//    //[fieldoffset(1)] 
//    public int targetjointindex;

//    /// <summary>
//    /// the target's position or eulers weight, range [0-1],0 means dont believe that. 
//    /// </summary>
//    //[fieldoffset(1 + 4 )] 
//    public float3 weight3;
//    /// <summary>
//    /// the target's position or eulers weight, range [0-1],0 means dont believe that. 
//    /// </summary>
//    //[fieldoffset(1 + 4+12)] 
//    public float3 torlerace3;

//    /// <summary>
//    /// the target position .overlapping with rotation
//    /// </summary>
//    //[fieldoffset(1 + 4 + 12+12)]
//    public float3 position;

//    /// <summary>
//    /// the target rotation .overlapping with position
//    /// </summary>
//    //[fieldoffset(1 + 4 + 12 + 12)]
//    public quaternion rotation;
//}
//#endregion