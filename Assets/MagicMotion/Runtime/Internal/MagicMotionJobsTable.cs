using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;
using System;
using UnityEngine;

namespace MagicMotion
{
    internal static class MagicMotionJobsTable
    {
        //OYM：确定hips的位置
        public struct InitializeMuscleJob : IJobParallelFor
        {
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleNative> musclesNatives;
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<float> musclesValues;

            public void Execute(int index)
            {
                musclesValues[index] = 0;
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
                for (int i = 0; i < muscleLength; i++)
                {
                    int offset = i * jointLength;
                    var rigid = jointTransformNatives[offset + index];
                    rigid.pos = transform.position;
                    rigid.rot = transform.rotation;
                    jointTransformNatives[offset + index] = rigid;
                }

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
                int point = transformToConstrainNatives[index].constraintIndex;
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
                for (int i = 0; i < muscleLength + 1; i++)
                {
                    int offset = jointLength * i;
                    constraintNatives[offset + point] = constraint;
                }
            }
        }
        public struct RigHipPositionJob : IJobParallelFor
        {

            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<MMJointNative> jointNatives;
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

                        MMPositionConstraint positionConstraint = jointConstraint.positionConstraint;

                        if (positionConstraint.isVaild)//OYM：据说这个人畜无害的小判断会破坏向量化，但是俺寻思这么一点计算量也看不出来
                        {
                            MMJointNative joint = jointNatives[offset+i];

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

        [BurstCompile]
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
            public NativeArray<MMMuscleNative> musclesNatives;
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
                    var musclesNative = musclesNatives[i];
                    var muscleValue = musclesValues[i];
                    float3 targetDof3 = Dof3s[musclesNative.jointIndex+ offset];
                    if (i +1== index )
                    {
                        targetDof3[musclesNative.dof] = muscleValue + L_BFGSStatic. EPSILION;//OYM：计算gradient的值
                    }
                    else
                    {
                        targetDof3[musclesNative.dof] = muscleValue;
                    }
                    Dof3s[musclesNative.jointIndex + offset] = targetDof3;
                }
            }
        }
        [BurstCompile]
        public struct BuildTransformJob : IJobParallelFor
        {
            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            internal NativeArray<MMGlobalData> globalDataNative;
            [NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeArray<MMJointNative> jointNatives;
            [ReadOnly]
            public NativeArray<float3> Dof3s;

            public int jointLength;
/*            [ReadOnly]
            public NativeSlice<bool> isUpdate;*/

            public void Execute(int index)
            {
                if (!globalDataNative[0].isContinue)
                {
                    return;
                }

                int offset = index * jointLength;
                for (int i = 0; i < jointLength; i++)
                {
                    int point = offset + i;
                MMJointNative currentJoint = jointNatives[point];
                RigidTransform currentTransform = jointTransformNatives[point];
                float3 Dof3 = Dof3s[point];

                float3 Dof3toRadian = math.radians(
                    math.lerp(0, currentJoint.minRange, -math.clamp(Dof3, -1, 0))
                + math.lerp(0, currentJoint.maxRange, math.clamp(Dof3, 0, 1+L_BFGSStatic.EPSILION))
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
                    RigidTransform parentTransform = jointTransformNatives[offset+currentJoint.parentIndex];
                    parentPosition = parentTransform.pos;
                    parentRotation = parentTransform.rot;
                }
                currentTransform.pos = parentPosition + math.mul(parentRotation, currentJoint.localPosition);
                currentTransform.rot = math.mul(parentRotation, math.mul(currentJoint.localRotation, eulerAngle));
                    jointTransformNatives[point]=currentTransform;
                }
            }
        }
        [BurstCompile]
        public struct CaclulatelossJob : IJobParallelFor
        {
            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            internal NativeArray<MMGlobalData> globalDataNative;

            [ReadOnly]
            public NativeArray<MMConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeArray<float3> Dof3s;
            public NativeArray<MMJoinloss> jointlossNatives;

            public void Execute(int index)
            {
                if (!globalDataNative[0].isContinue)
                {
                    return;
                }
                MMConstraintNative constraintNative = constraintNatives[index];
                MMJoinloss jointloss = jointlossNatives[index];
                RigidTransform jointTransform = jointTransformNatives[index];
                float3 Dof3 = Dof3s[index];

                if (constraintNative.positionConstraint.isVaild)
                {
                    UpdatePositionloss(ref jointloss, jointTransform, constraintNative);
                }
                if (constraintNative.muscleConstraint.isVaild)
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
                if (constraintNative.muscleChangeConstraint.isVaild)
                {
                    UpdateMuscleChangeloss(ref jointloss, Dof3, constraintNative);
                }
                jointloss.Clacloss();
               jointlossNatives[index] = jointloss;
            }
            private static void UpdatePositionloss(ref MMJoinloss jointloss, RigidTransform jointTransform, MMConstraintNative constraintNative)
            {
                MMPositionConstraint positionConstraint = constraintNative.positionConstraint;
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
                float3 tolerance3 = constraintNative.muscleConstraint.tolerance3;
                float3 weight3 = constraintNative.muscleConstraint.weight3;
                float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3,0);
                float loss =math.csum(Dof3Outside * weight3);
                jointloss.muscleloss = loss;
            }
            private static void UpdateLookAtloss(ref MMJoinloss jointloss, RigidTransform jointTransform, MMConstraintNative constraintNative)
            { 
            MMLookAtConstraint lookAtConstraint = constraintNative.lookAtConstraint;
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
                MMPositionChangeConstraint positionConstraint = constraintNative.positionChangeConstraint;
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
                float3 oldDof3 = constraintNative.muscleChangeConstraint.oldDof3;
                float3 torlerence3 = constraintNative.muscleChangeConstraint.torlerence3;
                float3 weight3 = constraintNative.muscleChangeConstraint.weight3;

                float3 Dof3Change = math.abs( Dof3 - oldDof3);
                Dof3Change = math.max(0, Dof3Change - torlerence3)* weight3;
                float loss = math.csum(Dof3Change);
                jointloss.muscleChangeloss = loss* loss;
            }
        }
        [BurstCompile]
        public struct MainControllerJob : IJobParallelFor
        {
            //OYM：感觉计算量超级的大啊
            //OYM：等我回来在写注释
            //OYM：趁着现在灵感还在
            /// <summary>
            /// joint loss
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<MMJoinloss> jointlossNatives;
/*            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleNative> musclesNatives;*/
            /// <summary>
            /// muscle Value 
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<float> muscleValueNatives;
            /// <summary>
            /// Gradient 
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<float> gradients;

            #region LBFGS Data
            /// <summary>
            /// BFGS 求解应用到的变量
            /// </summary>
            public NativeArray<MMLBFGSSolver> LBFGSSolvers;
            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            internal NativeArray<MMGlobalData> globalDataNative;
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
            #endregion
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
            /// group offset ,we must have more L_BFGS 
            /// </summary>
            public int offset;
            /// <summary>
            /// max loop count.
            /// </summary>
            public float loss;

            public void Execute(int index)
            {
                PreOptimizeProcess();
                var LBFGSSolver = LBFGSSolvers[index];
                var globalData = globalDataNative[index];

                LBFGSSolver.Optimize(loss,ref globalData.leastLoopCount, diagonal, gradientStore, rho, alpha, steps, delta, muscleValueNatives, gradients);
                losses[globalData.leastLoopCount] = loss;

                LBFGSSolvers[index] = LBFGSSolver;
                globalDataNative[index] = globalData;
                PostOptimizeProcess();
            }

            private void PostOptimizeProcess()
            {
            }

            private void PreOptimizeProcess()
            {
                CollectlossAndGradient();
            }

            private void CollectlossAndGradient()
            {
                loss = Collectloss(jointlossNatives, 0, constraintLength,jointLength);

                for (int i = 0; i <  muscleLength; i++)
                {
                    int offset = jointLength + jointLength * i;
                    float lossTemp = Collectloss(jointlossNatives, offset, constraintLength, jointLength);
                    float gradientTemp = (lossTemp- loss)/ L_BFGSStatic.EPSILION;
                    gradients[i] = gradientTemp;
                }
            }

            private static float Collectloss(NativeArray<MMJoinloss> losses, int offset, int constraintlength,int jointLength)
            {
                float loss = 0;

                for (int i = 0; i < jointLength; i++)
                {
                    loss += losses[i+offset].lossSum;
                }
                loss = math.sqrt(loss / constraintlength);
                return loss;
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

                for (int i = 0; i < muscleLength+1; i++)
                {
                    int point = i * jointLength+ index;
                    var constraintNativeData = constraintNatives[point];
                    constraintNativeData.positionChangeConstraint.oldPosition = jointTransformNatives[index].pos;
                    constraintNativeData.muscleChangeConstraint.oldDof3 = Dof3s[index];
                    constraintNatives[point] = constraintNativeData;
                }
            }
        }
    }
}
