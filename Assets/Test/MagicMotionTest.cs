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
    MMMuscleNative[] muscleNativeArray;
    MMConstraintNative[] constraintNativeArray;
    TransformToConstraintNative[] transformToConstraintArray;
    Transform[] jointTransformArray;
    Transform[] constraintTransformArray;

    void Start()
    {
        bioJoints=gameObject.GetComponentsInChildren<BioJoint>();
        bioConstraints = gameObject.GetComponentsInChildren<BioObjective>();

        jointNativeArray=new MMJointNative[bioJoints.Length];
        constraintNativeArray=new MMConstraintNative[bioJoints.Length];
        jointTransformArray=new Transform[bioJoints.Length];
        muscleNativeArray = new MMMuscleNative[bioJoints.Length * 3];
transformToConstraintArray=new TransformToConstraintNative[bioConstraints.Length];
        constraintTransformArray=new Transform[bioConstraints.Length];

        for (int i = 0; i < muscleNativeArray.Length; i++)
        {
            var muscle = muscleNativeArray[i];
            muscle.dof = i % 3;
            muscle.jointIndex = i/3;
            muscle.muscleIndex = i;
            muscleNativeArray[i] = muscle;
        }

        for (int i = 0; i < bioJoints.Length; i++)
        {
            var bioJoint = bioJoints[i];
            var bioParent = bioJoint.parent?.gameObject. GetComponentInParent<BioJoint>();
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
                currentJoint.localRotation = bioJoint.transform.rotation;
                currentJoint.localPosition = bioJoint.transform.position;
                currentJoint.length =  0;
            }
            else
            {
                currentJoint.localRotation = math.mul(math.inverse(bioParent.transform.rotation), bioJoint.transform.rotation);
                currentJoint.localPosition = math.mul(math.inverse(bioParent.transform.rotation), (float3)(bioJoint.transform.position - bioParent.transform.position));

                currentJoint.length = math.length(currentJoint.localPosition);
            }

            jointNativeArray[i]=currentJoint;
            jointTransformArray[i]=bioJoint.transform;
        }
        for (int i = 0; i < bioConstraints.Length; i++)
        {
            var bioConstraint = bioConstraints[i];

            int contraintIndex = Array.IndexOf(jointTransformArray, bioConstraint.transform);
            var constraint =constraintNativeArray[contraintIndex];
            var transformToConstraint = transformToConstraintArray[i];
            transformToConstraint.constraintIndex = contraintIndex;

            var joint = jointNativeArray[contraintIndex];

            switch (bioConstraint.GetObjectiveType())
            {
                case ObjectiveType.Position:
                    constraint.positionConstraint.weight3= 1;
                    transformToConstraint.constraintType = MMConstraintType.Position;
                    constraintTransformArray[i] = (bioConstraint as Position).Target;
                    while (joint.parentIndex!=-1)
                    {
                        constraint.positionConstraint.lengthSum += joint.length;
                        joint = jointNativeArray[joint.parentIndex];
                    }
                    break;
                case ObjectiveType.LookAt:
                    constraint.lookAtConstraint.weight = 1;
                    constraint.lookAtConstraint.direction = (bioConstraint as BioIK2.LookAt).ViewingDirection.normalized;
                    transformToConstraint.constraintType = MMConstraintType.LookAt;
                    constraintTransformArray[i] = (bioConstraint as BioIK2.LookAt).Target;
                    break;
                default:
                    break;
            }
            constraintNativeArray[contraintIndex] = constraint;
            transformToConstraintArray[i] = transformToConstraint;
        }
        motionKernel = new MagicMotionKernel();
        motionKernel.SetMuscleSata(muscleNativeArray);
        motionKernel.SetJointData(jointNativeArray, jointTransformArray);
        motionKernel.SetConstraintData(constraintNativeArray, transformToConstraintArray, constraintTransformArray);
       motionKernel.Initialize();
    }

    // Update is called once per frame
    void Update()
    {
        motionKernel.Update(Time.deltaTime,40);
    }
    private void OnDestroy()
    {
        if (motionKernel.IsCreated)
        {
            motionKernel.Dispose();
        }
    }
}
