using System;
using UnityEngine;
using System.Collections;


namespace MagicMotion.Mono
{
    public class MMDofChangeConstraint : MMConstraint
    {
        #region  Field&Property
        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        [Range(0, 1)]
        public Vector3 weight3;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        [Range(0, 1f)]
        public Vector3 tolerance3;
        /// <summary>
        /// constraint Type
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.PositionChange;
        /// <summary>
        /// old Dof3 value
        /// </summary>
        public Vector3 oldDof3 => relationJoint.dof3Value;

        private Vector3 tempWeight3;


        #endregion

        #region LocalFunc
        private IEnumerator BeginReset()
        {
            tempWeight3 = weight3;
            weight3 = Vector3.zero;
            yield return null;
            weight3 = tempWeight3;
            yield break;
        }


        public override void ReSet()
        {
            StartCoroutine(BeginReset());
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                DofChangeConstraint = GetNativeData(),
            };
        }

        internal DofChangeConstraint GetNativeData()
        {
            return new DofChangeConstraint()
            {
                tolerance3 = tolerance3,
                weight3 = weight3,
                oldDof3 = oldDof3
            };
        }
        #endregion
    }
}