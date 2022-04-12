using UnityEngine;

namespace MagicMotion.Mono
{
    public abstract class MMConstraint:MonoBehaviour
    {
        /// <summary>
        /// data index in array
        /// </summary>
        [HideInInspector]
        public int index;
        public abstract MMConstraintNative GetNativeData();

        public abstract void ResetPositionAndRotation();

        public abstract void Initialize();

        public abstract MMConstraintType GetConstraintType();
    }
}