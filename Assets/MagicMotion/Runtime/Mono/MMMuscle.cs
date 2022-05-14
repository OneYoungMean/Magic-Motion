using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Burst.CompilerServices;

namespace MagicMotion.Mono
{
    public class MMMuscle:MonoBehaviour
    {
        /// <summary>
        /// muscle dof ,must from range 0-2
        /// </summary>
        [HideInInspector]
        public int dof;
        /// <summary>
        /// muscle to joint 
        /// </summary>
        public MMJoint joint;
        /// <summary>
        /// the muscle Name
        /// </summary>
        public string muscleName;
        /// <summary>
        /// the muscle Name
        /// </summary>
        [Range(-1,1)]
        public float value;

        internal MuscleData GetNativeData()
        {
            return new MuscleData()
            {
                jointIndex = joint.jointIndex,
                dof = dof,
            };
        }

        private void OnValidate()
        {
            if (joint!=null)
            {
                joint.OnValidate();
            }

        }
    }
}


