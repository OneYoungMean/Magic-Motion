using System.Collections;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMPositionChangeConstraint : MMConstraint
    {

        #region  Field&Property
        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        public Vector3 weight3=Vector3.one;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        public Vector3 tolerance3;
        /// <summary>
        /// targetTransform
        /// </summary>
        public override Transform TargetTransform => relationJoint.transform;
        /// <summary>
        /// constraint Type
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.PositionChange;

        private bool isReset;
        private Vector3 tempWeight3;
        #endregion

        #region LocalFunc

        public override void ReSet()
        {
            StartCoroutine(BeginReset());
        }
        private IEnumerator BeginReset()
        {
            tempWeight3 = weight3;
            weight3 = Vector3.zero;
            yield return null;
            weight3 = tempWeight3;
            yield break;
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                positionChangeConstraint = GetNativeData()
            };
        }

        internal PositionChangeConstraint GetNativeData()
        {
            return new PositionChangeConstraint()
            {
                tolerance3 = tolerance3,
                weight3 = weight3,
                oldPosition = TargetTransform.position
            };
        }
        #endregion
    }
}