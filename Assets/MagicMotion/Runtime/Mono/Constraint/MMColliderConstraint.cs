using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMLookConstraint : MMConstraint
    {
        #region  Field&Property

        public const float MIN__TOLERENCE = 1e-2f;

        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        [Range(0, 1)]
        public float weight = 1;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        [Range(MIN__TOLERENCE, 180f)]
        public float tolerance = MIN__TOLERENCE;
        /// <summary>
        /// Constraint target
        /// </summary>
        public Transform targetTransform;
        /// <summary>
        /// joint lookat direction
        /// </summary>
        public Vector3 jointDirection;
        /// <summary>
        /// Constraint Type
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.LookAt;
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
        }

        public override void ReSet()
        {
            transform.localPosition = initalLocalPosition;
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                lookAtConstraint = GetNativeData(),
            };
        }

        internal LookAtConstraint GetNativeData()
        {
            return new LookAtConstraint()
            {
                weight = weight,
                tolerance = tolerance,
                direction = jointDirection,
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
