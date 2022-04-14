using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;
using System;

namespace MagicMotion
{
    internal class MagicMotionJobsTable
    {
        //OYM：确定hips的位置
        public struct RigHipositionJob : IJob
        {
            public int loopCount;
            [ReadOnly]
            public NativeArray<MMJointNative> jointNatives;
            [ReadOnly]
            public NativeArray<MMJointConstraintNative> positionConstraintNatives;
            public NativeArray<RigidTransform> jointTransformNatives;

            public void Execute()
            {
                float3 hipJointPosition = jointTransformNatives[0].pos;
                //OYM：计算根节点的位移，这个可以根据每个位置约束对跟节点产生的拉力进行计算
                //OYM：我认为这个步骤可以代替使用随机数生成的位置。
                for (int j0 = 0; j0 < loopCount; j0++)
                {
                    float3 deltaPosition = 0;

                    for (int i = 0; i < positionConstraintNatives.Length; i++)
                    {
                        MMJointConstraintNative jointConstraint = positionConstraintNatives[i];

                        MMPositionConstraint positionConstraint = jointConstraint.positionConstraint;

                        if (positionConstraint.isVaild)//OYM：据说这个人畜无害的小判断会破坏向量化，但是俺寻思这么一点计算量也看不出来
                        {
                            MMJointNative joint = jointNatives[i];

                            float3 hipsDirection = hipJointPosition - positionConstraint.position;

                            float hipsDirectionDistance = math.length(hipsDirection);

                            float restDistance = joint.cumulativeLength;

                            float forceLength = math.max(hipsDirectionDistance - restDistance, 0);

                            float3 force = (hipsDirection / hipsDirectionDistance) * forceLength * positionConstraint.weight3;

                            deltaPosition += force;
                        }


                    }
                    hipJointPosition += deltaPosition / positionConstraintNatives.Length;
                }

                float3 alldeltaPosition = hipJointPosition - jointTransformNatives[0].pos;
                for (int i = 0; i < jointTransformNatives.Length; i++)
                {
                    var rigid = jointTransformNatives[i];
                    rigid.pos += alldeltaPosition;
                    jointTransformNatives[i] = rigid;
                }
            }
        }

        public struct BuildTransform : IJobFor
        {
            [NativeDisableParallelForRestriction]
            public NativeSlice<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeSlice<MMJointNative> jointNatives;
            [ReadOnly]
            public NativeSlice<float3> Dof3Natives;
            [ReadOnly]
            public NativeSlice<bool> isUpdate;

            public void Execute(int index)
            {
                if (!isUpdate[index]) return;

                MMJointNative currentJoint = jointNatives[index];
                RigidTransform currentTransform = jointTransformNatives[index];
                float3 Dof3 = Dof3Natives[index];

                float3 Dof3toRadian = math.radians(
                    math.lerp(0, currentJoint.minRange, -math.clamp(Dof3, -1, 0))
                + math.lerp(0, currentJoint.maxRange, math.clamp(Dof3, 0, 1))
                );
                quaternion eulerAngle = quaternion.identity;
                if (Dof3[0] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[0], Dof3toRadian[0]), eulerAngle);
                }
                if (Dof3[1] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[1], Dof3toRadian[1]), eulerAngle);
                }
                if (Dof3[2] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[2], Dof3toRadian[2]), eulerAngle);
                }
                quaternion parentRotation; float3 parentPosition;

                if (currentJoint.parentIndex == -1)
                {
                    parentPosition = float3.zero;
                    parentRotation = quaternion.identity;
                }
                else
                {
                    RigidTransform parentTransform = jointTransformNatives[currentJoint.parentIndex];
                    parentPosition = parentTransform.pos;
                    parentRotation = parentTransform.rot;
                }
                currentTransform.pos = parentPosition + math.mul(parentRotation, currentJoint.localPosition);
                currentTransform.rot = math.mul(parentRotation, math.mul(currentJoint.localRotation, eulerAngle));
            }
        }

        public struct CaclulateFitnessJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<MMJointConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeArray<bool> isUpdate;
            [ReadOnly]
            public NativeSlice<float3> Dof3Natives;
            [WriteOnly]
            public NativeArray<MMJoinFitness> jointFitnessNatives;

            public void Execute(int index)
            {
                if (!isUpdate[index]) return;

                MMJointConstraintNative constraintNative = constraintNatives[index];
                MMJoinFitness jointFitness = jointFitnessNatives[index];
                RigidTransform jointTransform = jointTransformNatives[index];
                float3 Dof3 = Dof3Natives[index];

                if (constraintNative.positionConstraint.isVaild)
                {
                    UpdatePositionFitness(ref jointFitness, jointTransform, constraintNative);
                }
                if (constraintNative.muscleConstraint.isVaild)
                {
                    UpdateMuscleFitness(ref jointFitness, Dof3, constraintNative);
                }
                if (constraintNative.lookAtConstraint.isVaild)
                {
                    UpdateLookAtFitness(ref jointFitness, jointTransform, constraintNative);
                }
                if (constraintNative.colliderConstraint.isVaild)
                {
                    UpdateColliderConstraint(ref jointFitness, jointTransform, constraintNative);
                }

                if (constraintNative.positionChangeConstraint.isVaild)
                {
                    UpdatePositionChangeFitness(ref jointFitness, jointTransform, constraintNative);
                }
                if (constraintNative.muscleChangeConstraint.isVaild)
                {
                    UpdateMuscleChangeFitness(ref jointFitness, Dof3, constraintNative);
                }
            }
            private static void UpdatePositionFitness(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMJointConstraintNative constraintNative)
            {
                MMPositionConstraint positionConstraint = constraintNative.positionConstraint;
                float3 jointPosition = jointTransform.pos;

                float3 constraintPosition = positionConstraint.position;
                float3 torlerace3 = positionConstraint.tolerance3;
                float3 weight3 = positionConstraint.weight3;

                float3 direction = constraintPosition - jointPosition;

                direction = math.max(math.EPSILON, torlerace3);

                direction = direction / torlerace3;
                float directionLength = math.length(direction) - 1;
                directionLength = math.max(0, directionLength);

                direction = (direction * directionLength) * torlerace3;

                float fitness = math.csum(direction * weight3);
                jointFitness.positionFitness = fitness;
            }
            private static void UpdateMuscleFitness(ref MMJoinFitness jointFitness, float3 Dof3, MMJointConstraintNative constraintNative)
            {
                float3 tolerance3 = constraintNative.muscleConstraint.tolerance3;
                float3 weight3 = constraintNative.muscleConstraint.weight3;
                float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3,0);
                float fitness =math.csum(Dof3Outside * weight3);
                jointFitness.muscleFitness = fitness;
            }
            private static void UpdateLookAtFitness(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMJointConstraintNative constraintNative)
            { 
            MMLookAtConstraint lookAtConstraint = constraintNative.lookAtConstraint;
            float3 jointPosition = jointTransform.pos;
            quaternion jointRotation = jointTransform.rot;

            float3 constraintPosition = lookAtConstraint.position;
            float tolerance = lookAtConstraint.tolerance;
            float weight = lookAtConstraint.weight;

            float3 targetDirection = constraintPosition - jointPosition;
            float3 targetForward = math.mul(jointRotation, new float3(0, 0, 1));//OYM：后续会更改的

            float cosA = math.dot(targetForward, targetDirection) / (math.length(targetDirection));
            cosA = math.clamp(cosA, -1, 1);

            float fitness = math.acos(cosA);
            jointFitness.lookAtFitness= fitness;
            }
            private static void UpdateColliderConstraint(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMJointConstraintNative constraintNative)
            {
                //OYM：啊这个超级难写 
                //OYM：还要去构造AABB
                //OYM：不想写（摆烂
            }
            private static void UpdatePositionChangeFitness(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMJointConstraintNative constraintNative)
            {
                MMPositionChangeConstraint positionConstraint = constraintNative.positionChangeConstraint;
                float3 jointPosition = jointTransform.pos;

                float3 constraintPosition = positionConstraint.oldPosition;
                float3 torlerace3 = positionConstraint.tolerance3;
                float3 weight3 = positionConstraint.weight3;

                float3 direction = constraintPosition - jointPosition;

                direction = math.max(math.EPSILON, torlerace3);

                direction = direction / torlerace3;
                float directionLength = math.length(direction) - 1;
                directionLength = math.max(0, directionLength);

                direction = (direction * directionLength) * torlerace3;

                float fitness = math.csum(direction * weight3);
                jointFitness.positionFitness = fitness;
            }

            private static void UpdateMuscleChangeFitness(ref MMJoinFitness jointFitness, float3 Dof3, MMJointConstraintNative constraintNative)
            {
                float3 oldDof3 = constraintNative.muscleChangeConstraint.oldDof3;
                float3 torlerence3 = constraintNative.muscleChangeConstraint.torlerence3;
                float3 weight3 = constraintNative.muscleChangeConstraint.weight3;

                float3 Dof3Change = math.abs( Dof3 - oldDof3);
                Dof3Change = math.max(0, Dof3Change - torlerence3);
                float fitness = math.csum(Dof3Change);
                jointFitness.muscleChangeFitness = fitness;
            }
        }

        public struct L_BFGSJob : IJobParallelFor
        {
            //OYM：感觉计算量超级的大啊
            //OYM：等我回来在写注释
            //OYM：趁着现在灵感还在
            /// <summary>
            /// all joint constraint data
            /// </summary>
            [ReadOnly]
            public NativeArray<MMJointConstraintNative> constraintNatives;
            /// <summary>
            /// all joint transform data
            /// </summary>
            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            /// <summary>
            /// joint relatived chain, a index for muti relatived joint's index 
            /// </summary>
            [ReadOnly]
            public NativeMultiHashMap<int, int> jointRelativedChain;
            /// <summary>
            /// joint fitness
            /// </summary>
            [ReadOnly]
            public NativeArray<MMJoinFitness> jointFitnessNatives;
            /// <summary>
            /// setting does the joint update ,maybe optimize after i finish all work
            /// </summary>
            [WriteOnly]
            public NativeArray<bool> isUpdate;
            /// <summary>
            /// the Dof3, is map from muscle value.
            /// </summary>
            [WriteOnly]
            public NativeArray<float3> Dof3Natives;
            /// <summary>
            /// In fact it dont need that ,but for some reason( too much work and bug) we deside to 
            /// keep  that until we finish all.
            /// </summary>
            private NativeArray<float> musclesVarible;
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleNative> musclesNatives;
            /// <summary>
            /// work area ,I dont know what happend inside.
            /// </summary>
            private NativeArray<float> work;
            /// <summary>
            /// BFGS求解器
            /// </summary>
            [ReadOnly]
            public NativeArray<MMLBFGSNative> LBFGSNatives;

            /// <summary>
            ///  variable's count ,is the same as muscle's count.
            /// </summary>
            public int numberOfVariables;
            /// <summary>
            /// joint count ,so how need I use that?
            /// </summary>
            public int jointCount;
            /// <summary>
            /// group offset ,we must have more L_BFGS 
            /// </summary>
            public int offset;
            /// <summary>
            /// max loop count.
            /// </summary>
            public void Execute(int index)
            {
                PreOptimizeProcess();
                //LBFGSNatives[index].Optimize();
                PostOptimizeProcess();
            }

            private void PostOptimizeProcess()
            {
   
            }

            private void PreOptimizeProcess()
            {

            }
        }
    }
}
