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
        public Transform targetTransform;
        /// <summary>
        /// joint lookat direction
        /// </summary>
        public Vector3 jointDirection;

        /// <summary>
        /// initialLocalPosition;
        /// </summary>
        private Vector3 initalLocalPosition;
        /// <summary>
        /// initialLocalQuaternion;
        /// </summary>
        private Quaternion initalLocalRotation;

        public void AddTarget(Transform targetTransform)
        {
            this.targetTransform = targetTransform;
            initalLocalPosition = targetTransform.localPosition;
            initalLocalRotation = targetTransform.localRotation;
        }

/*        public override MMConstraintNative GetNativeData()
        {
            return new MMConstraintNative()
            {
                constraintType =MMConstraintType.LookAt,
                weight3 = weight,
                torlerace3 = tolerance,
                position = transform.position
            };
        }*/

        public override void Reset()
        {
            transform.localPosition = initalLocalPosition;
            transform.localRotation = initalLocalRotation;
        }

        public override MMConstraintType GetConstraintType()
        {
           return MMConstraintType.LookAt;
        }

        public LookAtConstraint GetNativeData()
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
    }

}
