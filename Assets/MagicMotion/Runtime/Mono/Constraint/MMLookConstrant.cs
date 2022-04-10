using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMLookConstrant : MonoBehaviour
    {
        public const float MIN__TOLERENCE = 0.1f;

        /// <summary>
        /// the constraint error's weight;
        /// </summary>
        [Range(0,1)]
        public float weight;
        /// <summary>
        /// the constraint error's tolerance;
        /// </summary>
        [Range(MIN__TOLERENCE, 180f)]
        public float tolerance;
        /// <summary>
        /// Constraint target
        /// </summary>
        public Transform targetJointTransform;
    }

}
