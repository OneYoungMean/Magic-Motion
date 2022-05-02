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
        /// the joint Name
        /// </summary>
        public string jointName;
        /// <summary>
        /// data index in array
        /// </summary>
        [HideInInspector]
        public int jointIndex;
        /// <summary>
        ///  the joint's parent , the hip 's parent is empty
        /// </summary>
        public MMJoint parent;
        /// <summary>
        ///  the joint's muscle
        /// </summary>
        public MMMuscle[] muscles = new MMMuscle[3];
        /// <summary>
        ///  the joint's constraint
        /// </summary>
        public MMConstraint[]constraints = new MMConstraint[sizeof(byte)*8];

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
        public float3 initiallocalPosition;

        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        public quaternion initiallocalRotation;

        /// <summary>
        /// the dof3 axis
        /// </summary>
        public float3x3 dof3Axis;

        /// <summary>
        /// initial localPosition 's Length
        /// </summary>
        public float length;

        /// <summary>
        /// the direction length to hips joint
        /// </summary>
        public float cumulativeLength;

        /// <summary>
        /// the joint human type
        /// </summary>
        public HumanBodyBones humanBodyBone;


        internal JointData GetNativeJointData()
        {
            return new JointData()
            {
                parentIndex = parent==null?-1: parent.jointIndex,
                localPosition = initiallocalPosition,
                localRotation = initiallocalRotation,
                maxRange = maxRange,
                minRange = minRange,
                dof3Axis = dof3Axis,
                length = length,

                isVaild = enabled,
            };
    }

        internal void GetNativeConstraintData(out ConstraintData constraint,out List<TransformToConstraintData>transformToConstraints,out List<Transform> transforms )
        {
             constraint=default(ConstraintData);
            constraint.lengthSum = cumulativeLength;
            transformToConstraints =new List<TransformToConstraintData>();
            transforms=new List<Transform>();

            for (int i = 0; i < constraints.Length; i++)
            {
                var constraintData = constraints[i];
                if (constraintData==null)
                {
                    continue;
                }

                var type = (MMConstraintType)i;
                switch (type)
                {
                    case MMConstraintType.Position:
                        var positionConstraint = constraintData as MMPositionConstraint;
                        constraint.positionConstraint= positionConstraint.GetNativeData();
                        transformToConstraints.Add(new TransformToConstraintData()
                        {
                            jointIndex = jointIndex,
                            constraintType = type
                        });
                        transforms.Add(positionConstraint.targetTransform);
                        break;
                    case MMConstraintType.Rotation:
                        break;
                    case MMConstraintType.LookAt:
                        var lookAtConstraint = constraintData as MMLookConstraint;
                        constraint.lookAtConstraint = lookAtConstraint.GetNativeData();

                        transformToConstraints.Add(new TransformToConstraintData()
                        {
                            jointIndex = jointIndex,
                            constraintType = type
                        });
                        transforms.Add(lookAtConstraint.targetTransform);
                        break;
                    case MMConstraintType.Collider:
                        break;
                    case MMConstraintType.PositionChange:
                        constraint.positionChangeConstraint = (constraintData as MMPositionChangeConstraint).GetNativeData();
                        break;
                    case MMConstraintType.DofChange:
                        constraint.DofChangeConstraint = (constraintData as MMDofChangeConstraint).GetNativeData();
                        break;
                    default:
                        break;
                }
            }
        }
    }

}
