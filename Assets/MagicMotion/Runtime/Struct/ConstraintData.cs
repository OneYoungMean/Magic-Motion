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
        Collider=3,
        PositionChange=4,
        DofChange=5,
        Direction = 6,
    }
    /// <summary>
    /// The joint constraint.
    /// Can be read as position ,Rotation LookAt
    /// </summary>
    internal struct ConstraintData
    {
        public float lengthSum;
        public PositionConstraint positionConstraint;
        public DofConstraint DofConstraint;
        public LookAtConstraint lookAtConstraint;
        public ColliderConstraint colliderConstraint;
        public PositionChangeConstraint positionChangeConstraint;
        public DofChangeConstraint  DofChangeConstraint ;
        public DirectionConstraint directionConstraint; 

        public int GetVaildCount()
        {
            return (positionConstraint.isVaild ? 1 : 0) +
                (DofConstraint.isVaild ? 1 : 0) +
                (lookAtConstraint.isVaild ? 1 : 0) +
                (colliderConstraint.isVaild ? 1 : 0) +
                (positionChangeConstraint.isVaild ? 1 : 0) +
                (DofChangeConstraint.isVaild ? 1 : 0);
        }
    }

    internal struct PositionConstraint
    {
        public float3 position;
        public float3 weight3;
        public float3 tolerance3;


        public bool isVaild
        {
            get { return !math.all(weight3 == 0); }
        }
    }

    internal struct DofConstraint
    {
        public float3 tolerance3;
        public float3 weight3;
        public bool isVaild
        {
            get { return !math.all(tolerance3 == 0); }
        }
    }

    internal struct LookAtConstraint
    {
        public float3 position;
        public float3 direction;
        public float weight;
        public float tolerance;
        public bool isVaild
        {
            get { return weight != 0; }
        }
    }

    internal struct ColliderConstraint
    {
        public float3 localPosition;
        public float3 localDirection;
        public float radius;
        public float length;
        public float weight;
        public bool isVaild
        {
            get { return weight != 0; }
        }
    }

    internal struct PositionChangeConstraint
    {
        public float3 oldPosition;
        public float3 tolerance3;
        public float3 weight3;
        public bool isVaild
        {
            get { return !math.all(weight3 == 0); }
        }
    }
    internal struct DofChangeConstraint 
    {
        public float3 oldDof3;
        public float3 tolerance3;
        public float3 weight3;
        public bool isVaild
        {
            get { return !math.all(weight3 == 0); }
        }
    }
    internal struct DirectionConstraint
    {
        public float3 direction;
        public float tolerance;
        public float weight;
        public bool isVaild
        {
            get { return weight == 0; }
        }
    }

    internal struct MMGlobalConstraint
    {
        public bool isBodyBalanceConstraint;//OYM£ºfuture
    }

}