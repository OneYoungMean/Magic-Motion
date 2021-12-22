using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace GeneticJob
{
    public struct GeneticJoint
    {
        /// <summary>
        /// if positionMode. Joint will be move ,if RotationMode Joint will be rotate.
        /// a transform can take 0-2 joint,just keep it parent- child chain relation truly .
        /// </summary>
        public JointMode JointMode;
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
        /// initial localPRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion localRotation;

        public bool hasChild => childIndexStart < childIndexEnd;
    }

    public enum JointMode
    {
        Position, Rotation
    }
}