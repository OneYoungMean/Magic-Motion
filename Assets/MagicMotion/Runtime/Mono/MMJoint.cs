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
    public class MMJoint : MonoBehaviour
    {
        #region  Field&Property

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
        public MMConstraint[] constraints = new MMConstraint[sizeof(byte) * 8];

        /// <summary>
        /// if is Position Joint mode ,it means move Range ,else means euler angle;
        /// </summary>
        private float3 maxRange;

        /// <summary>
        /// if is Position Joint mode ,it means move Range ,else means euler angle;
        /// </summary>
        private float3 minRange;

        /// <summary>
        /// the dof3 axis
        /// </summary>
        private float3x3 dof3Axis;

        /// <summary>
        /// initial localPosition from parent,used be clac currentPosition
        /// </summary>
        private float3 initiallocalPosition;

        /// <summary>
        /// initial localRotation from parent,used be clac currentPosition and localRotation
        /// </summary>
        private quaternion initiallocalRotation;

        [HideInInspector]
        /// <summary>
        /// the joint dof3 Value;
        /// </summary>
        internal float3 dof3Value;

        /// <summary>
        /// initial localPosition 's Length
        /// </summary>
        private float length;

        /// <summary>
        /// the direction length to hips joint
        /// </summary>
        private float cumulativeLength;

        /// <summary>
        /// the joint human type
        /// </summary>
        public HumanBodyBones humanBodyBone;
        #endregion

        #region UnityFunc

        public void OnValidate()
        {
            UpdateMotion();
        }

        #endregion

        #region LocalFunc

        public void UpdateMotion()
        {
            UpdateMuscleData();

            float3 Dof3toRadian = math.radians(
                    math.lerp(0, minRange, -math.clamp(dof3Value, -1, 0))
                + math.lerp(0, maxRange, math.clamp(dof3Value, 0, 1))
                );

            quaternion eulerAngle = quaternion.identity;
            if (dof3Value[0] != 0)
            {
                eulerAngle = math.mul(quaternion.AxisAngle(dof3Axis[0], Dof3toRadian[0]), eulerAngle);
            }
            if (dof3Value[1] != 0)
            {
                eulerAngle = math.mul(quaternion.AxisAngle(dof3Axis[1], Dof3toRadian[1]), eulerAngle);
            }
            if (dof3Value[2] != 0)
            {
                eulerAngle = math.mul(quaternion.AxisAngle(dof3Axis[2], Dof3toRadian[2]), eulerAngle);
            }

            if (parent == null)
            {
                transform.localRotation = math.mul(initiallocalRotation, eulerAngle);
            }
            else
            {
                quaternion parentRotation = parent.transform.rotation;
                float3 parentPosition = parent.transform.position;
                transform.position = parentPosition + math.mul(parentRotation, initiallocalPosition);
                transform.rotation = math.mul(parentRotation, math.mul(initiallocalRotation, eulerAngle));
            }
        }

        private void UpdateMuscleData()
        {
            for (int i = 0; i < 3; i++)
            {
                if (muscles[i] == null)
                {
                    dof3Value[i] = 0;
                    minRange[i] = 0;
                    maxRange[i] = 0;
                    dof3Axis[i] = 0;
                }
                else
                {
                    dof3Value[i] = muscles[i].value;
                    minRange[i] = muscles[i].angleRange[0];
                    maxRange[i] = muscles[i].angleRange[1];
                    dof3Axis[i] = muscles[i].axis;
                }
            }
        }

        public void RegisterMuscleAndConstraint()
        {
           var musclesTemp = gameObject.GetComponents<MMMuscle>();
            for (int i = 0; i < musclesTemp.Length; i++)
            {
                muscles[i]=musclesTemp[i];
                musclesTemp[i].dof = i;
            }
            var constraintsTemp = gameObject.GetComponents<MMConstraint>();
            for (int i = 0; i < constraintsTemp.Length; i++)
            {
                int index = (int)constraintsTemp[i].ConstraintType;
                constraints[index]=constraintsTemp[i];
            }
        }

        public void ClacInitialLocalTransform()
        {
            if (parent == null)
            {
                initiallocalRotation = transform.localRotation;
                initiallocalPosition = transform.localPosition;
                length = cumulativeLength = 0;
            }
            else
            {
                initiallocalRotation = math.mul(math.inverse(parent.transform.rotation), transform.rotation);
                initiallocalPosition = math.mul(math.inverse(parent.transform.rotation), (float3)(transform.position - parent.transform.position));

                length = math.length(initiallocalPosition);
                cumulativeLength = parent.cumulativeLength + length;
            }

        }

        public void Initialize()
        {
            RegisterMuscleAndConstraint();
            ClacInitialLocalTransform();
            UpdateMuscleData();
        }

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
            constraintData.lengthSum =math.max(0.01f, cumulativeLength);

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
