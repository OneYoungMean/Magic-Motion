using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMColliderConstraint : MMConstraint
    {
        #region  Field&Property

        public const float MIN__TOLERENCE = 1e-2f;

        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        [Range(0,1)]
        public float weight=1;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        [Range(MIN__TOLERENCE, 180f)]
        public float tolerance= MIN__TOLERENCE;
        /// <summary>
        /// Constraint target
        /// </summary>
        public Transform targetTransform;
        /// <summary>
        /// collider Direction
        /// </summary>
        public Vector3 colliderDirection;
        /// <summary>
        /// collider length ,sphere is zero
        /// </summary>
        public float length;
        /// <summary>
        /// collider radius 
        /// </summary>
        public float radius;
        /// <summary>
        /// Constraint Type
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.Collider;
        /// <summary>
        /// Target transform;
        /// </summary>
        public override Transform TargetTransform => targetTransform;

        /// <summary>
        /// initialLocalPosition;
        /// </summary>
        private Vector3 initalLocalPosition;
        #endregion

        #region LocalFunc

        public void AddTarget(Transform targetTransform)
        {
            this.targetTransform = targetTransform;
            initalLocalPosition = targetTransform.localPosition;

            colliderDirection = Quaternion.Inverse(targetTransform.rotation) * Vector3.up;
        }

        public override void ReSet()
        {
            transform.localPosition = initalLocalPosition;
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                colliderConstraint = GetNativeData(),
            };
        }

        internal ColliderConstraint GetNativeData()
        {
            return new ColliderConstraint()
            {
                weight = weight,
                localDirection = targetTransform.rotation* colliderDirection,
                localPosition=targetTransform.position,

                length = length,
                radius = radius,
            };
        }

        public void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(targetTransform.position, 0.05f);
        }
        #endregion
    }

}
