using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMPositionChangeConstraint : MMConstraint
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

        public override void ReSet()
        {
            
        }

        public PositionChangeConstraint GetNativeData()
        {
            return new PositionChangeConstraint()
            {
                tolerance3 = tolerance3,
                weight3 = weight3,
                oldPosition = transform.position
            };
        }    
    }
}