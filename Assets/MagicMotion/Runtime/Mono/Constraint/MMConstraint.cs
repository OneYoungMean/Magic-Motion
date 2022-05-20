using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace MagicMotion.Mono
{
    public abstract class MMConstraint : MonoBehaviour
    {
        public MMJoint relationJoint;
        public bool isEnable;
        public abstract void ReSet();
        public abstract MMConstraintType ConstraintType { get; }
        internal abstract ConstriantContainer GetConstriantContainer();
        public virtual Transform TargetTransform => transform;

        public static MMConstraint CreateConstraint(MMConstraintType constraintType, MMJoint joint)
        {
            switch (constraintType)
            {
                case MMConstraintType.Position:
                    {

                        var positionConstraint = joint.gameObject.AddComponent<MMPositionConstraint>();
                        positionConstraint.relationJoint = joint;
                        positionConstraint.weight3 = Vector3.one;
                        joint.constraints[(int)constraintType] = positionConstraint;
                        return positionConstraint;
                    }
                case MMConstraintType.Rotation:
                    {
                        var rotationConstraint = joint.gameObject.AddComponent<MMRotationConstraint>();
                        rotationConstraint.weight = 1;
                        rotationConstraint.relationJoint = joint;
                        joint.constraints[(int)constraintType] = rotationConstraint;
                        return rotationConstraint;
                    }

                case MMConstraintType.LookAt:
                    {
                        var lookatConstraint = joint.gameObject.AddComponent<MMLookConstraint>();
                        lookatConstraint.relationJoint = joint;
                        lookatConstraint.weight = 1;
                        lookatConstraint.jointDirection = Vector3.up; //OYM：这里也不一定是up....啊乱死了
                        joint.constraints[(int)constraintType] = lookatConstraint;
                        return lookatConstraint;
                    }

                case MMConstraintType.Collider:
                    {
                        var colliderConstraint = joint.gameObject.AddComponent<MMLookConstraint>();
                        colliderConstraint.relationJoint = joint;
                        colliderConstraint.weight = 1;
                        joint.constraints[(int)constraintType] = colliderConstraint;
                        return colliderConstraint;
                    }

                case MMConstraintType.PositionChange:
                    {
                        var positionChangeConstraint = joint.gameObject.AddComponent<MMPositionChangeConstraint>();
                        positionChangeConstraint.relationJoint = joint;
                        positionChangeConstraint.weight3 = Vector3.one;
                        joint.constraints[(int)constraintType] = positionChangeConstraint;
                        return positionChangeConstraint;
                    }

                case MMConstraintType.DofChange:
                    {
                        var DofChangeConstraint = joint.gameObject.AddComponent<MMDofChangeConstraint>();
                        DofChangeConstraint.relationJoint = joint;
                        DofChangeConstraint.weight3 = Vector3.one;
                        joint.constraints[(int)constraintType] = DofChangeConstraint;
                        return DofChangeConstraint;
                    }

                case MMConstraintType.Direction:
                    {
                        var directionConstraint = joint.gameObject.AddComponent<MMDirectionConstraint>();
                        directionConstraint.relationJoint = joint;
                        directionConstraint.weight = 1;
                        joint.constraints[(int)constraintType] = directionConstraint;
                        return directionConstraint;
                    }
                default:
                    throw new NotImplementedException();
            }
        }

        [StructLayout(LayoutKind.Explicit)]
        internal struct ConstriantContainer
        {
            [FieldOffset(0)]
            public PositionConstraint positionConstraint;
            [FieldOffset(0)]
            public RotationConstraint rotationConstraint;
            [FieldOffset(0)]
            public DofConstraint DofConstraint;
            [FieldOffset(0)]
            public LookAtConstraint lookAtConstraint;
            [FieldOffset(0)]
            public ColliderConstraint colliderConstraint;
            [FieldOffset(0)]
            public PositionChangeConstraint positionChangeConstraint;
            [FieldOffset(0)]
            public DofChangeConstraint DofChangeConstraint;
            [FieldOffset(0)]
            public DirectionConstraint directionConstraint;
        }
    }
}
/*                        GameObject positionIK = new GameObject("IK_" + joint.name + "_Position");
                        positionIK.transform.parent = joint.controller.transform;
                        positionIK.transform.position = joint.transform.position;
                        positionConstraint.AddTarget(positionIK.transform);
*/
/*
constraint = positionConstraint;

if (isBuildConstraintRelation && joint.parent != null && joint.parent.constraints[(int)constraintType] != null)
{
positionIK.transform.parent = (joint.parent.constraints[(int)constraintType] as MMPositionConstraint).targetTransform;
}
else
{
positionIK.transform.parent = transform;
}
positionIK.transform.position = joint.transform.position;
break;*/
/*                    GameObject rotationIK = new GameObject("IK_" + joint.name + "_Rotation");
                    rotationIK.transform.parent = joint.controller.transform;
                    rotationIK.transform.position = joint.transform.position;
                    rotationIK.transform.rotation = joint.transform.rotation;
                    rotationConstraint.AddTarget(rotationIK.transform);*/

/*                    GameObject lookAtIK = new GameObject("IK_" + joint.name + "_Lookat");
                    lookAtIK.transform.parent = joint.transform;
                    lookAtIK.transform.position = joint.transform.position + 0.1f * joint.transform.up;
                    lookatConstraint.AddTarget(lookAtIK.transform);*/
