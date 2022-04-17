﻿using UnityEngine;

namespace MagicMotion.Mono
{
    public abstract class MMConstraint:MonoBehaviour
    {
        public abstract void ResetPositionAndRotation();

        public abstract void Initialize();

        public abstract MMConstraintType GetConstraintType();
    }
}