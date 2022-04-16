using BioIK2;
using MagicMotion;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class MagicMotionTest : MonoBehaviour
{
    // Start is called before the first frame update
    MagicMotionKernel motionKernel;
    public BioJoint[] bioJoints;
    public BioObjective[] bioConstraints;
    MMJointNative[] jointNativeArray;
    MMConstraintNative[] constraintNativeArray;
    TransformToConstraintNative[] transformToConstraintArray;
    Transform[] jointTransformArray;
    Transform[] constraintTransformArray;

    void Start()
    {
        bioJoints=gameObject.GetComponentsInChildren<BioJoint>();
        bioConstraints = gameObject.GetComponents<BioObjective>();

        jointNativeArray=new MMJointNative[bioJoints.Length];
        constraintNativeArray=new MMConstraintNative[bioJoints.Length];
        jointTransformArray=new Transform[bioJoints.Length];

        transformToConstraintArray=new TransformToConstraintNative[bioConstraints.Length];
        constraintTransformArray=new Transform[bioConstraints.Length];

        for (int i = 0; i < bioJoints.Length; i++)
        {
            var bioJoint = bioJoints[i];
            var bioParent = bioJoint.GetComponentInParent<BioJoint>();
            int parentIndex=Array.IndexOf(bioJoints, bioJoint.parent);
            MMJointNative currentJoint = new MMJointNative()
            {
                isVaild = true,
                maxRange = 15f,
                minRange = -15f,
                parentIndex = parentIndex,
                dof3Axis=new float3x3(
                    1,0,0,
                    0,1,0,
                    0,0,1
                    ),

            };

            if (parentIndex == -1)
            {
                currentJoint.localRotation = bioParent.transform.rotation;
                currentJoint.localPosition = bioParent.transform.position;
                currentJoint.length =  0;
            }
            else
            {
                currentJoint.localRotation = math.mul(math.inverse(bioParent.transform.rotation), bioParent.transform.rotation);
                currentJoint.localPosition = math.mul(math.inverse(bioParent.transform.rotation), (float3)(bioParent.transform.position - bioParent.transform.position));

                currentJoint.length = math.length(currentJoint.localPosition);
            }

            jointNativeArray[i]=currentJoint;
            jointTransformArray[i]=bioJoint.transform;
        }
        for (int i = 0; i < bioConstraints.Length; i++)
        {
            var bioConstraint = bioConstraints[i];

            int contraintIndex = Array.IndexOf(bioConstraints, bioConstraint.transform.parent);
            var constraint =constraintNativeArray[contraintIndex];
            var transformToConstraint = transformToConstraintArray[i];
            transformToConstraint.constraintIndex = contraintIndex;

            switch (bioConstraint.GetObjectiveType())
            {
                case ObjectiveType.Position:
                    constraint.positionConstraint.weight3= 1;
                    transformToConstraint.constraintType = MMConstraintType.Position;
                    constraintTransformArray[i] = (bioConstraint as Position).Target;
                    break;
                case ObjectiveType.LookAt:
                    constraint.lookAtConstraint.weight = 1;
                    transformToConstraint.constraintType = MMConstraintType.LookAt;
                    constraintTransformArray[i] = (bioConstraint as BioIK2.LookAt).Target;
                    break;
                default:
                    break;
            }
            constraintNativeArray[contraintIndex] = constraint;
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
