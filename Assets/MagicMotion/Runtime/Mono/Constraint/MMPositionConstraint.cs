using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion.Mono
{
    /// <summary>
    /// Position Constraint ,will keep target joint's position.
    /// </summary>
    public class MMPositionConstraint : MMConstraint
    {
        #region  Field&Property

        public const float MIN__TOLERENCE = 0.001f;
        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        public Vector3 weight3;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        public Vector3 tolerance3;
        /// <summary>
        /// targetTransform
        /// </summary>
        public override Transform TargetTransform => targetTransform;
        /// <summary>
        /// constraintType
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.Position;

        /// <summary>
        /// initialLocalPosition;
        /// </summary>
        private Vector3 initalLocalPosition;
        [SerializeField]
        /// <summary>
        /// Constraint target
        /// </summary>
        private Transform targetTransform;
        #endregion
        #region LocalFunc

        public void SetTarget(Transform targetTransform)
        {
            this.targetTransform = targetTransform;
            initalLocalPosition = targetTransform.localPosition;
        }
        public override void ReSet()
        {
            transform.localPosition = initalLocalPosition;
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                positionConstraint = GetNativeData()
            };
        }

        internal PositionConstraint GetNativeData()
        {
            return new PositionConstraint()
            {
                weight3 = weight3,
                tolerance3 = tolerance3,
                position = targetTransform.position
            };
        }

        public void OnDrawGizmos()
        {
            if (targetTransform==null)
            {
                return;
            }
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(targetTransform.position,0.01f);
        }
        #endregion
    }
}

/*        public override MMConstraintNative GetNativeData()
        {
            return new MMConstraintNative()
            {
                constraintType = MMConstraintType.Position,
                position = transform.position,
                weight3 = weight3,
                torlerace3 = tolerance3,
            };

        }*/