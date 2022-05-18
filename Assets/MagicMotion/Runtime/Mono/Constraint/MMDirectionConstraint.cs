using System;
using UnityEngine;

namespace MagicMotion.Mono
{
    internal class MMDirectionConstraint : MMConstraint
    {
        #region  Field&Property
        public const float MIN__TOLERENCE = 1e-2f;
        /// <summary>
        /// direction from transform
        /// </summary>
        public Transform fromDirection;
        /// <summary>
        /// direction to transform
        /// </summary>
        public Transform toDirection;
        /// <summary>
        /// angle tolerance
        /// </summary>
        public float tolerance;
        /// <summary>
        /// constraint weight
        /// </summary>
        public float weight;
        /// <summary>
        /// constraint type
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.Direction;
        /// <summary>
        /// EMMMMMMMMMM....... all right.
        /// </summary>
        public override Transform TargetTransform => toDirection;

        private Vector3 initialLocalPosition_From;
        private Vector3 initialLocalPosition_To;
        #endregion

        #region LocalFunc
        public void SetTarget(Transform form, Transform to)
        {
            this.fromDirection = form;
            this.toDirection = to;
            initialLocalPosition_From = form.localPosition;
            initialLocalPosition_To = to.localPosition;
        }
        public override void ReSet()
        {
            fromDirection.localPosition = initialLocalPosition_From;
            toDirection.localPosition = initialLocalPosition_To;
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                directionConstraint = GetNativeData()
            };
        }
        internal DirectionConstraint GetNativeData()
        {
            return new DirectionConstraint()
            {
                weight = weight,
                tolerance = tolerance,
                direction = (toDirection.position - fromDirection.position).normalized,
            };
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(TargetTransform.position, 0.01f);
        }
        #endregion


    }
}