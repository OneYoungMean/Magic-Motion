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
        #region  Field&Property
        /// <summary>
        /// Controller
        /// </summary>
        public MMJointController controller;
        /// <summary>
        /// the joint Name
        /// </summary>
        public string jointName;
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
        /// the joint dof3 Value;
        /// </summary>
        public Vector3 dof3Value;

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
        #endregion

        #region UnityFunc

        public void OnValidate()
        {
            if (controller != null)
            controller.UpdateMotion();
        }

        #endregion

        #region LocalFunc
        public MMConstraint GetConstraint(MMConstraintType constraintType)
        {
            return constraints[(int)constraintType];
        }
        internal JointData GetNativeJointData(MMJoint[] group)
        {
            return new JointData()
            {
                parentIndex = parent == null ? -1 : Array.IndexOf(group,parent),
                localPosition = initiallocalPosition,
                localRotation = initiallocalRotation,
                maxRange = maxRange,
                minRange = minRange,
                dof3Axis = dof3Axis,
                length = length,
                isVaild = enabled,
            };
        }

        internal ConstraintData GetNativeConstraintData()
        {
            var constraintData=default(ConstraintData);
            constraintData.lengthSum =math.max(0.1f, cumulativeLength);

            for (int i = 0; i < constraints.Length; i++)
            {
                var constraintMono = constraints[i];
                if (constraintMono==null)
                {
                    continue;
                }

                var type = (MMConstraintType)i;
                switch (type)
                {
                    case MMConstraintType.Position:
                        constraintData.positionConstraint = constraintMono.GetConstriantContainer().positionConstraint;
                        break;
                    case MMConstraintType.Rotation:
                        constraintData.rotationConstraint = constraintMono.GetConstriantContainer().rotationConstraint;
                        break;
                    case MMConstraintType.LookAt:
                        constraintData.lookAtConstraint = constraintMono.GetConstriantContainer().lookAtConstraint;
                        break;
                    case MMConstraintType.Collider:
                        constraintData.colliderConstraint = constraintMono.GetConstriantContainer().colliderConstraint;
                        break;
                    case MMConstraintType.PositionChange:
                        constraintData.positionChangeConstraint = constraintMono.GetConstriantContainer().positionChangeConstraint;
                        break;
                    case MMConstraintType.DofChange:
                        constraintData.DofChangeConstraint = constraintMono.GetConstriantContainer().DofChangeConstraint;
                        break;
                    case MMConstraintType.Direction:
                        constraintData.directionConstraint = constraintMono.GetConstriantContainer().directionConstraint;
                        break;
                    default:
                        break;
                }

            }
            return constraintData;
        }

        public MMConstraint CreateConstraint(MMConstraintType constraintType)
        {
            var oldData = GetConstraint(constraintType);
            if (oldData != null)
            {
                return oldData;
            }
            else
            {
                return MMConstraint.CreateConstraint(constraintType, this);
            }

        }
        #endregion
    }

}
