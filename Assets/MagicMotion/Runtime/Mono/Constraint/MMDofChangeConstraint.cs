using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMDofChangeConstraint : MMConstraint
    {
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

        public override MMConstraintType GetConstraintType()
        {
            return MMConstraintType.PositionChange;
        }

        public override void Reset()
        {
            
        }

        public DofChangeConstraint GetNativeData()
        {
            return new DofChangeConstraint()
            {
                torlerence3 = tolerance3,
                weight3 = weight3,
            };
        }
    }
}