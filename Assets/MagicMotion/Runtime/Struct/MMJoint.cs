using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion.Mono
{
    /// <summary>
    /// the Joint form the bone
    /// </summary>
    public class MMJoint:MonoBehaviour
    {
        /// <summary>
        ///  the joint's parent , the hip 's parent is empty
        /// </summary>
        public MMJoint parent;

        /// <summary>
        ///  the joint's muscle
        /// </summary>
        public MMMuscle[] muscles = new MMMuscle[3];

        /// <summary>
        ///  parent index in heap
        /// </summary>
        [HideInInspector]
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
        /// the dof3 axis
        /// </summary>
        public float3x3 dof3Axis;

/*        /// <summary>
        /// initial worldPosition from parent,used be clac currentPosition
        /// </summary>
        public float3 position;

        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion rotation;*/

        /// <summary>
        /// the intial motion's dof,mostly is zero
        /// </summary>
        public float3 Dof3;

        /// <summary>
        /// the joint human type
        /// </summary>
        public HumanBodyBones humanBodyBone;


        public MMJointNative GetNativeData()
        {
            return new MMJointNative()
            {
                parentIndex = parentIndex,
                localPosition = localPosition,
                localRotation = localRotation,
                maxRange = maxRange,
                minRange = minRange,
                dof3Axis = dof3Axis,
                isVaild = enabled,
            };
    }
    }

}
