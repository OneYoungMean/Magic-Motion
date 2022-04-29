using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;
using System;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;

namespace MagicMotion
{
    internal static unsafe class MagicMotionJobsTable
    {
        //OYM：确定hips的位置
        public struct InitializeMuscleJob : IJobParallelFor
        {
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleData> muscleDatas;
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<float> musclesValues;

            public void Execute(int index)
            {
                musclesValues[index] = math.clamp(musclesValues[index],0,0);
            }
        }
        public struct InitializeJointJob : IJobParallelForTransform
        {
            //OYM：没用
            public NativeArray<RigidTransform> jointTransformNatives;

            public int muscleLength;
            public int jointLength;

            public void Execute(int index, TransformAccess transform)
            {
                var rigid = jointTransformNatives[ index];
                rigid.pos = transform.position;
                rigid.rot = transform.rotation;
                jointTransformNatives[index] = rigid;

            }
        }
        [BurstCompile]
        public struct TransformToConstraintJob : IJobParallelForTransform
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<MMConstraintNative> constraintNatives;

            public NativeArray<TransformToConstraintNative> transformToConstrainNatives;
            public int muscleLength;
            public int jointLength;
            public void Execute(int index, TransformAccess transform)
            {
                int point = transformToConstrainNatives[index].jointIndex;
                MMConstraintType type = transformToConstrainNatives[index].constraintType;
               var constraint = constraintNatives[ point];

                switch (type)
                {
                    case MMConstraintType.Position:
                        constraint.positionConstraint.position = transform.position;
                        break;
                    case MMConstraintType.Rotation:
                        break;
                    case MMConstraintType.LookAt:
                        constraint.lookAtConstraint.position = transform.position;
                        break;
                    default:
                        break;
                }
                constraintNatives[point] = constraint;
            }
        }
        [BurstCompile]
        public struct ScheduleConstraintDataJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<JointRelationData> jointRelationDatas;
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransforms;
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            [NativeDisableParallelForRestriction]
            public NativeArray<MMConstraintNative> constraints;
            public void Execute(int index)
            {
                var jointRelation = jointRelationDatas[index];
                var originConstraint = constraints[jointRelation.jointIndex];
                var jointTransform = jointTransforms[jointRelation.jointIndex];
                var jointDof3 = Dof3s[jointRelation.jointIndex];

                originConstraint.positionChangeConstraint.oldPosition = jointTransform.pos;
                originConstraint.DofChangeConstraint.oldDof3= jointDof3;
                constraints[index] = originConstraint;
            }
        }
        //待考证
        public struct RigHipPositionJob : IJobParallelFor
        {
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<JointData> jointDatas;
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<MMConstraintNative> constraintNatives;
            [NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;

            public int loopCount;
            public int jointLength;
            //OYM：计算根节点的位移，这个可以根据每个位置约束对跟节点产生的拉力进行计算
            //OYM：我认为这个步骤可以代替使用随机数生成的位置。
            public void Execute(int index)
            {
                int offset = index * jointLength;

                float3 hipJointPosition = jointTransformNatives[offset].pos;

                for (int j0 = 0; j0 < loopCount; j0++)
                {
                    float3 deltaPosition = 0;

                    for (int i = 0; i < jointLength; i++)
                    {
                        MMConstraintNative jointConstraint = constraintNatives[offset+i];

                        PositionConstraint positionConstraint = jointConstraint.positionConstraint;

                        if (positionConstraint.isVaild)//OYM：据说这个人畜无害的小判断会破坏向量化，但是俺寻思这么一点计算量也看不出来
                        {
                            JointData joint = jointDatas[offset+i];

                            float3 hipsDirection = hipJointPosition - positionConstraint.position;

                            float hipsDirectionDistance = math.length(hipsDirection);

                            float restDistance = jointConstraint.lengthSum;

                            float forceLength = math.max(hipsDirectionDistance - restDistance, 0);

                            float3 force = (hipsDirection / hipsDirectionDistance) * forceLength * positionConstraint.weight3;

                            deltaPosition += force;
                        }
                    }
                    hipJointPosition += deltaPosition / constraintNatives.Length;
                }

                float3 alldeltaPosition = hipJointPosition - jointTransformNatives[offset].pos;
                for (int i = 0; i < jointLength; i++)
                {
                    var rigid = jointTransformNatives[offset+i];
                    rigid.pos += alldeltaPosition;
                    jointTransformNatives[offset+i] = rigid;
                }
            }
        }

        /*        [BurstCompile]
                public struct MuscleToJointJob : IJobParallelFor
                {

                    /// <summary>
                    /// global data ,to control loop's work
                    /// </summary>
                    [ReadOnly, NativeDisableParallelForRestriction]
                    internal NativeArray<MMGlobalData> globalDataNative;
                    [NativeDisableParallelForRestriction]
                    public NativeArray<float3> Dof3s;
                    [ReadOnly]
                    /// <summary>
                    /// muscles value ,containing joint index and dof index.
                    /// </summary>
                    public NativeArray<MMMuscleData> muscleDatas;
                    [ReadOnly]
                    /// <summary>
                    /// muscles value ,containing joint index and dof index.
                    /// </summary>
                    public NativeArray<float> musclesValues;
                    public int jointCount;

                    public int muscleCount;

                    public  void Execute(int index)
                    {
                        if (!globalDataNative[0].isContinue)
                        {
                            return;
                        }

                        int offset = index * jointCount;

                        for (int i = 0; i < muscleCount; i++)//OYM：第0行留给loss，1~muscleCount留给gradient
                        {
                            var muscleData = muscleDatas[i];
                            var muscleValue = musclesValues[i];
                            float3 targetDof3 = Dof3s[muscleData.jointIndex+ offset];
                            if (i +1== index )
                            {
                                targetDof3[muscleData.dof] = muscleValue + L_BFGSStatic. EPSILION;//OYM：计算gradient的值
                            }
                            else
                            {
                                targetDof3[muscleData.dof] = muscleValue;
                            }
                            Dof3s[muscleData.jointIndex + offset] = targetDof3;
                        }
                    }
                }*/
        [BurstCompile]
        public struct MuscleToDof3Job : IJobParallelFor
        {
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleData> muscleDatas;
            [NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            [NativeDisableParallelForRestriction, ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<float> muscleValues;
            public void Execute(int index)
            {
                var muscleData = muscleDatas[index];
                var muscleValue = muscleValues[index];
                float3 targetDof3 = Dof3s[muscleData.jointIndex];
                targetDof3[muscleData.dof] = muscleValue;
                Dof3s[muscleData.jointIndex] = targetDof3;
            }
        }
        [BurstCompile]
        public struct Dof3ToRotationJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Dof3s;
            public NativeArray<quaternion> Dof3Quaternions;
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<JointData> jointDatas;
            public void Execute(int index)
            {
                JointData currentJoint = jointDatas[index];

                float3 Dof3 = Dof3s[index];

                float3 Dof3toRadian = math.radians(
                    math.lerp(0, currentJoint.minRange, -math.clamp(Dof3, -1, 0))
                + math.lerp(0, currentJoint.maxRange, math.clamp(Dof3, 0, 1 + L_BFGSStatic.EPSILION))
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

                Dof3Quaternions[index] = eulerAngle;
            }
        }
        [BurstCompile]
        public struct ClacDof3EpsilionJob : IJobParallelFor
        {
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleData> muscleDatas;
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<quaternion> Dof3Quaternions;
            [ReadOnly, NativeDisableParallelForRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<JointData> jointDatas;
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            
            public NativeArray<quaternion> muscleGradientRotations;
            public void Execute(int index)
            {
                var muscleData = muscleDatas[index];
                var currentJoint = jointDatas[muscleData.jointIndex];

                float3 Dof3 = Dof3s[muscleData.jointIndex];

                Dof3[muscleData.dof] +=L_BFGSStatic.EPSILION;
                quaternion before = Dof3Quaternions[muscleData.jointIndex];
                quaternion after = quaternion.identity;
                float3 Dof3toRadian = math.radians(math.lerp(0, currentJoint.minRange, -math.clamp(Dof3, -1, 0)) + math.lerp(0, currentJoint.maxRange, math.clamp(Dof3, 0, 1 + L_BFGSStatic.EPSILION)));
                after = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[0], Dof3toRadian[0]), after);
                after = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[1], Dof3toRadian[1]), after);
                after = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[2], Dof3toRadian[2]), after);

                quaternion worldRot = jointTransformNatives[muscleData.jointIndex].rot;
                quaternion change = math.mul(worldRot, math.mul(math.mul(after, math.inverse(before)), math.inverse(worldRot)));
                muscleGradientRotations[index] = change;
            }
        }
        [BurstCompile]
        public struct BuildTransformJob : IJobFor
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeArray<JointData> jointDatas;
            [ReadOnly]
            public NativeArray<quaternion> Dof3Quaternions;
            public void Execute(int index)
            {
                JointData currentJoint = jointDatas[index];
                RigidTransform currentTransform = jointTransformNatives[index];
                quaternion eulerAngle = Dof3Quaternions[index];
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
                jointTransformNatives[index] = currentTransform;
            }
        }
        [BurstCompile]
        public struct CaclulatelossJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<MMConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeArray<JointRelationData> jointRelationDatas;
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<quaternion> muscleGradientRotations;

            public NativeArray<MMJoinloss> jointlossNatives;

            public void Execute(int index)
            {

                JointRelationData jointRelationData = jointRelationDatas[index];    
                MMConstraintNative constraintNative = constraintNatives[index];
                MMJoinloss jointloss = jointlossNatives[index];


                RigidTransform jointTransform = jointTransformNatives[jointRelationData.jointIndex];
                if (jointRelationData.relatedJointIndex!=-1)
                {
                    RigidTransform relatedTransform = jointTransformNatives[jointRelationData.relatedJointIndex];
                    quaternion muscleGradientRotation = muscleGradientRotations[jointRelationData.relatedMuscleIndex];
                    RebuildJointTransform(ref jointTransform, relatedTransform, muscleGradientRotation);
                }

                float3 Dof3 = Dof3s[jointRelationData.jointIndex];

                if (constraintNative.positionConstraint.isVaild)
                {
                    UpdatePositionloss(ref jointloss, jointTransform, constraintNative);
                }
                if (constraintNative.DofConstraint.isVaild)
                {
                    UpdateMuscleloss(ref jointloss, Dof3, constraintNative);
                }
                if (constraintNative.lookAtConstraint.isVaild)
                {
                    UpdateLookAtloss(ref jointloss, jointTransform, constraintNative);
                }
                if (constraintNative.colliderConstraint.isVaild)
                {
                    UpdateColliderConstraint(ref jointloss, jointTransform, constraintNative);
                }
                if (constraintNative.positionChangeConstraint.isVaild)
                {
                    UpdatePositionChangeloss(ref jointloss, jointTransform, constraintNative);
                }
                if (constraintNative.DofChangeConstraint .isVaild)
                {
                    UpdateMuscleChangeloss(ref jointloss, Dof3, constraintNative);
                }
                jointloss.Clacloss();
               jointlossNatives[index] = jointloss;
            }

            private void RebuildJointTransform(ref RigidTransform jointTransform,  RigidTransform relatedTransform, quaternion muscleGradientRotation)
            {
                float3 direction = jointTransform.pos - relatedTransform.pos;
                jointTransform.pos = math.mul(muscleGradientRotation,direction)+ relatedTransform.pos;
                jointTransform.rot = math.mul( muscleGradientRotation, jointTransform.rot);
            }

            private static void UpdatePositionloss(ref MMJoinloss jointloss, RigidTransform jointTransform, MMConstraintNative constraintNative)
            {
                PositionConstraint positionConstraint = constraintNative.positionConstraint;
                float3 jointPosition = jointTransform.pos;

                float3 constraintPosition = positionConstraint.position;

                float3 torlerace3 =math.max(math.EPSILON, positionConstraint.tolerance3);

                float3 weight3 = positionConstraint.weight3;

                float3 direction = constraintPosition - jointPosition;

                if (math.all(direction == 0))
                {
                    return;
                }
/*
                direction = direction / torlerace3;

                float directionLength = math.length(direction);

                float newDirectionLength = math.max(0, directionLength - 1);

                direction = (direction / directionLength * newDirectionLength) * torlerace3;*/

                float loss = math.csum(direction * direction * weight3);

                loss *= math.PI*math.PI/( constraintNative.lengthSum * constraintNative.lengthSum);

                jointloss.positionloss = loss;
            }
            private static void UpdateMuscleloss(ref MMJoinloss jointloss, float3 Dof3, MMConstraintNative constraintNative)
            {
                float3 tolerance3 = constraintNative.DofConstraint.tolerance3;
                float3 weight3 = constraintNative.DofConstraint.weight3;
                float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3,0);
                float loss =math.csum(Dof3Outside * weight3);
                jointloss.muscleloss = loss;
            }
            private static void UpdateLookAtloss(ref MMJoinloss jointloss, RigidTransform jointTransform, MMConstraintNative constraintNative)
            { 
            LookAtConstraint lookAtConstraint = constraintNative.lookAtConstraint;
            float3 jointPosition = jointTransform.pos;
            quaternion jointRotation = jointTransform.rot;

            float3 constraintPosition = lookAtConstraint.position;
            float tolerance = lookAtConstraint.tolerance;
            float weight = lookAtConstraint.weight;

            float3 targetDirection = constraintPosition - jointPosition;
            float3 targetForward = math.mul(jointRotation, lookAtConstraint.direction);//OYM：后续会更改的

            float cosA = math.dot(targetForward, targetDirection) / (math.length(targetDirection));
            cosA = math.clamp(cosA, -1, 1);

            float loss = math.acos(cosA);
                loss = math.max(0, math.abs(loss) - tolerance * math.PI);
            jointloss.lookAtloss= loss* loss*weight;
            }
            private static void UpdateColliderConstraint(ref MMJoinloss jointloss, RigidTransform jointTransform, MMConstraintNative constraintNative)
            {
                //OYM：啊这个超级难写 
                //OYM：还要去构造AABB
                //OYM：不想写（摆烂
            }
            private static void UpdatePositionChangeloss(ref MMJoinloss jointloss, RigidTransform jointTransform, MMConstraintNative constraintNative)
            {
                PositionChangeConstraint positionConstraint = constraintNative.positionChangeConstraint;
                float3 jointPosition = jointTransform.pos;

                float3 constraintPosition = positionConstraint.oldPosition;
                float3 torlerace3 = math.max(math.EPSILON, positionConstraint.tolerance3); 
                float3 weight3 = positionConstraint.weight3;

                float3 direction = constraintPosition - jointPosition;
                if (math.all(direction == 0))
                {
                    return;
                }
/*                direction = direction / torlerace3;

                float directionLength = math.length(direction);

                float newDirectionLength = math.max(0, directionLength - 1);

                direction = (direction / directionLength * newDirectionLength) * torlerace3;*/

                float loss = math.csum(direction * direction * weight3);

                loss *= 0.1f;
                //loss =;

               // jointloss.positionChangeloss = -loss;
            }

            private static void UpdateMuscleChangeloss(ref MMJoinloss jointloss, float3 Dof3, MMConstraintNative constraintNative)
            {
                float3 oldDof3 = constraintNative.DofChangeConstraint .oldDof3;
                float3 torlerence3 = constraintNative.DofChangeConstraint .torlerence3;
                float3 weight3 = constraintNative.DofChangeConstraint .weight3;

                float3 Dof3Change = math.abs( Dof3 - oldDof3);
                Dof3Change = math.max(0, Dof3Change - torlerence3)* weight3;
                float loss = math.csum(Dof3Change)/3;
                jointloss.muscleChangeloss = loss* loss;
            }
        }
       // [BurstCompile]
        public struct MainControllerJob : IJob
        {
            //OYM：感觉计算量超级的大啊
            //OYM：等我回来在写注释
            //OYM：趁着现在灵感还在

            #region LBFGS Data
            /// <summary>
            /// BFGS 求解应用到的变量
            /// </summary>
            public NativeArray<MMLBFGSSolver> LBFGSSolvers;
            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            internal NativeArray<MMGlobalData> globalDataNative;
            /// <summary>
            /// Gradient 
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<float> gradients;
            [NativeDisableParallelForRestriction]
            public NativeArray<float> diagonal;
            [NativeDisableParallelForRestriction]
            public NativeArray<float> gradientStore;
            [NativeDisableParallelForRestriction]
            public NativeArray<float> rho;
            [NativeDisableParallelForRestriction]
            public NativeArray<float> alpha;
            [NativeDisableParallelForRestriction]
            public NativeArray<float> steps;
            [NativeDisableParallelForRestriction]
            public NativeArray<float> delta;
            [NativeDisableParallelForRestriction]
            internal NativeArray<float> losses;
            [NativeDisableParallelForRestriction]
            internal NativeArray<float> gradientSums;
            #endregion


            /// <summary>
            /// joint loss
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<MMJoinloss> jointlossNatives;
            [NativeDisableParallelForRestriction, ReadOnly]
            public NativeArray<JointRelationData> relationDatas;
            [NativeDisableParallelForRestriction, ReadOnly]
            public NativeArray<int> muscleRelativeCounts;
            [NativeDisableParallelForRestriction]


            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<float> muscleValues;
            /// <summary>
            ///  variable's count ,is the same as muscle's count.
            /// </summary>
            public int parallelLength;
            /// <summary>
            ///  variable's count ,is the same as muscle's count.
            /// </summary>
            public int constraintLength;
            /// <summary>
            ///  variable's count ,is the same as muscle's count.
            /// </summary>
            public int muscleLength;
            /// <summary>
            /// joint count ,so how need I use that?
            /// </summary>
            public int jointLength;
            /// <summary>
            /// loss Value;
            /// </summary>
            public float loss;
            public void Execute()
            {
                int index = 0;
                var globalData = globalDataNative[index];

                PreOptimizeProcess();
                losses[globalData.leastLoopCount] = loss;
                var LBFGSSolver = LBFGSSolvers[index];
                float gradientSum = gradientSums[globalData.leastLoopCount];
                LBFGSSolver.Optimize(loss, ref globalData.leastLoopCount, ref gradientSum, diagonal, gradientStore, rho, alpha, steps, delta, muscleValues, gradients);
                gradientSums[globalData.leastLoopCount] = gradientSum;
                LBFGSSolvers[index] = LBFGSSolver;
                globalDataNative[index] = globalData;
            }




            private void PreOptimizeProcess()
            {
            
                loss = Collectloss(jointlossNatives, 0, constraintLength, jointLength);
                CollectGradient(jointlossNatives, relationDatas, loss, jointLength, parallelLength, constraintLength, gradients);
            }



            private static float Collectloss(NativeArray<MMJoinloss> losses, int offset, int constraintlength,int jointLength)
            {
                float loss = 0;
                if (constraintlength!=0)
                {
                    for (int i = 0; i < jointLength; i++)
                    {
                        loss += losses[i + offset].lossSum;
                    }

                    loss = math.sqrt(loss / constraintlength);
                }

                return loss;
            }
            private static void CollectGradient(NativeArray<MMJoinloss> losses,NativeArray<JointRelationData> relationDatas, float loss, int jointLength, int parallelLength, int constraintLength, NativeArray<float>gradients)
            {
                UnsafeUtility.MemClear(gradients.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<float>());

                for (int i = jointLength; i < parallelLength; i++)
                {
                    JointRelationData relationData = relationDatas[i];

                    float gradientTemp = losses[i].lossSum-losses[relationData.jointIndex].lossSum;
                    gradients[relationData.relatedMuscleIndex] += gradientTemp/ constraintLength;
                }
            }
        }

        public struct JointToTransformJob : IJobParallelForTransform
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<MMConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeSlice<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeSlice<float3> Dof3s;
            public int muscleLength;
            public int jointLength;
            public void Execute(int index, TransformAccess transform)
            {
                transform.position = jointTransformNatives[index].pos;
                transform.rotation=jointTransformNatives[index].rot;
            }
        }
    }
}
