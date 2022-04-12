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
        /// the constraint error's weight;
        /// </summary>
        [Range(0, 1)]
        public float3 weight3=1;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        [Range(MIN__TOLERENCE, 1f)]
        public float3 tolerance3= MIN__TOLERENCE;
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
                constraintType = MMConstraintType.Position,
                position = transform.position,
                weight3 = weight3,
                torlerace3 = tolerance3,
            };

        }

        public override void ResetPositionAndRotation()
        {
            transform.localPosition = initalLocalPosition;
            transform.localRotation = initalLocalRotation;

        }

        public override MMConstraintType GetConstraintType()
        {
            return MMConstraintType.Position;
        }
    }
}

