using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Burst.CompilerServices;

namespace MagicMotion
{
    public class MMMuscle:MonoBehaviour
    {
        /// <summary>
        /// the muscle index,range 0-95
        /// </summary>
        [HideInInspector]
        public int muscleIndex;
        /// <summary>
        /// muscle to joint index ,range0-55
        /// </summary>
        [HideInInspector]
        public int jointIndex;
        /// <summary>
        /// muscle dof range 0-2
        /// </summary>
        [HideInInspector]
        public int dof;

        /// <summary>
        /// the muscle Name
        /// </summary>
        public string muscleName;
        /// <summary>
        /// the muscle Name
        /// </summary>
        [Range(-1,1)]
        public float value;

        public MMMuscleNative GetNativeData()
        {
            return new MMMuscleNative()
            {
                muscleIndex = muscleIndex,
                jointIndex = jointIndex,
                dof = dof,
            };
        }
    }
}


