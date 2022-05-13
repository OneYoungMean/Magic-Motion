using System;
using UnityEngine;

namespace MagicMotion.Mono
{
    internal class MMDirectionConstraint : MMConstraint
    {
        public Transform fromDirection;
        public Transform toDirection;
        public float tolerance;
        public float weight;

        public override MMConstraintType GetConstraintType()
        {
            return MMConstraintType.Direction;
        }

        public override void ReSet()
        {
            
        }

        internal DirectionConstraint GetNativeData()
        {
            return new DirectionConstraint()
            {
                weight = weight,
                tolerance = tolerance,
                direction = (toDirection.position - fromDirection.position).normalized,
            };
        }
    }
}