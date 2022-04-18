using System;
using UnityEngine;

namespace MagicMotion.Mono
{
    public abstract class MMConstraint:MonoBehaviour
    {
        public abstract void Reset();

        public abstract MMConstraintType GetConstraintType();

    }
}