using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMLookConstraint : MMConstraint
    {
        public const float MIN__TOLERENCE = 0.1f;

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
        public Transform targetJointTransform;
        /// <summary>
        /// initialLocalPosition;
        /// </summary>
        private Vector3 initalLocalPosition;
        /// <summary>
        /// initialLocalQuaternion;
        /// </summary>
        private Quaternion initalLocalRotation;

        public override void Initialize()
        {
            initalLocalPosition = transform.localPosition;
            initalLocalRotation = transform.localRotation;
        }

        public override MMConstraintNative GetNativeData()
        {
            return new MMConstraintNative()
            {
                constraintType =MMConstraintType.LookAt,
                weight3 = weight,
                torlerace3 = tolerance,
                position = transform.position
            };
        }

        public override void ResetPositionAndRotation()
        {
            transform.localPosition = initalLocalPosition;
            transform.localRotation = initalLocalRotation;
        }

        public override MMConstraintType GetConstraintType()
        {
           return MMConstraintType.LookAt;
        }
    }

}
