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
        ///  Is the joint vaild?
        /// </summary>
        public bool isVaild;

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
        /// initial worldPosition from parent,used be clac currentPosition
        /// </summary>
        public float3 position;
        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion rotation;
        /// <summary>
        /// initial localPosition from parent,used be clac currentPosition
        /// </summary>
        public float3 localPosition;
        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion localRotation;
        /// <summary>
        /// the intial motion's dof
        /// </summary>
        public float3 Dof3;
        /// <summary>
        /// the joint human type
        /// </summary>
       public  HumanBodyBones humanBodyBone;

        public bool isNoChild => childIndexStart < childIndexEnd;

        public MagicMotionJoint(bool isvaild=true)
        {
            isVaild = isvaild;
            parentIndex = -1;
            childIndexStart = -1;
            childIndexEnd = -1;
            position = float3.zero;
            rotation = quaternion.identity;
            localPosition = float3.zero;
            localRotation = quaternion.identity;
            MaxRange = float3.zero;
            MinRange = float3.zero;
            Dof3 = float3.zero;
            humanBodyBone = HumanBodyBones.LastBone;
        }
    }

    public enum JointMode
    {
        Position, Rotation
    }
    #endregion
}
