
using UnityEngine;

namespace MagicMotion.Mono
{
    internal class MMRotationConstraint : MMConstraint
    {

        #region  Field&Property
        /// <summary>
        /// Rotation tolerance 
        /// </summary>
        public float tolerance;
        /// <summary>
        /// Rotation weight
        /// </summary>
        public float weight;
        /// <summary>
        /// rotationTarget
        /// </summary>
        public Transform targetTransform;
        /// <summary>
        /// rotation target 
        /// </summary>
        public override Transform TargetTransform => targetTransform;
        /// <summary>
        /// Constraint type
        /// </summary>
        public override MMConstraintType ConstraintType => MMConstraintType.Rotation;

        private Quaternion initialLocalRotation;
        #endregion

        #region LocalFunc
        public void AddTarget(Transform targetTransform)
        {
            this.targetTransform = targetTransform;
            initialLocalRotation = targetTransform.localRotation;
        }
        public override void ReSet()
        {
            this.TargetTransform.localRotation = initialLocalRotation;
        }

        internal override ConstriantContainer GetConstriantContainer()
        {
            return new ConstriantContainer()
            {
                rotationConstraint = GetNativeContraint()
            };
        }

        internal RotationConstraint GetNativeContraint()
        {
            return new RotationConstraint()
            {
                weight = weight,
                tolerance = tolerance,
                rotation = targetTransform.rotation,
            };

        }
        #endregion




    }

}