using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion.Mono
{
    /// <summary>
    /// Position Constraint ,will keep target joint's position.
    /// </summary>
    public class MMPositionConstraint : MMConstraint
    {
        // Start is called before the first frame update
        public const float MIN__TOLERENCE = 0.001f;
        /// <summary>
        /// Manager
        /// </summary>
        public MMJointController controller;
        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        public Vector3 weight3;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        public Vector3 tolerance3;
        /// <summary>
        /// Constraint target
        /// </summary>
        public Transform targetTransform;

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
                constraintType = MMConstraintType.Position,
                position = transform.position,
                weight3 = weight3,
                torlerace3 = tolerance3,
            };

        }*/

        public override void ReSet()
        {
            transform.localPosition = initalLocalPosition;
            transform.localRotation = initalLocalRotation;

        }

        public override MMConstraintType GetConstraintType()
        {
            return MMConstraintType.Position;
        }

        internal PositionConstraint GetNativeData()
        {
            return new PositionConstraint()
            {
                weight3 = weight3,
                tolerance3 = tolerance3,
                position = targetTransform.position
            };
        }

        public void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(targetTransform.position,0.02f);
        }
    }
}

