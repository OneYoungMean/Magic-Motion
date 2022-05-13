using UnityEngine;

namespace MagicMotion.Mono
{
    internal interface IGetConstraint
    {
        public MMConstraint GetConstraintTarget(HumanBodyBones humanBodyBones, MMConstraintType constraintType);
    }
}