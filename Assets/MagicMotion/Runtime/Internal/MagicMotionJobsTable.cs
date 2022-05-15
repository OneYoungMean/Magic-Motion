using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;
using System;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;
using System.Runtime.CompilerServices;
using MagicMotion.Extern;

namespace MagicMotion
{
    using static Extrension;
    internal static unsafe class MagicMotionJobsTable
    {

        /// <summary>
        /// Set Constriant data 
        /// </summary>
        [BurstCompile]
        public struct CopyConstraintDataJob : IJobParallelFor
        {
            [ NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Joint relation data 
            /// </summary>
            public JointRelationData* jointRelationDatas;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// constraint data 
            /// </summary>
            public ConstraintData* constraintDatas;

            public int jointLength;
            public void Execute(int index)
            {
                JointRelationData* jointRelation = jointRelationDatas+index;
                constraintDatas[index] = constraintDatas[jointRelation->jointIndex];
            }
        }
        /// <summary>
        ///  muscles to joint dof value 
        /// </summary>
        [BurstCompile]
        public struct MuscleToDof3Job : IJobParallelFor
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public JointMusclesData* muscleDatas;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint muscles value 
            /// </summary>
            public float3* Dof3s;

            [NativeDisableUnsafePtrRestriction, ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public float* muscleValues;
            public void Execute(int index)
            {
                JointMusclesData* muscleData = muscleDatas+index;
                Dof3s[muscleData->jointIndex][muscleData->dof] = muscleValues[ index];
            }
        }
        /// <summary>
        ///  Convert dof3 to rotation value 
        /// </summary>
        [BurstCompile]
        
        public struct Dof3ToRotationJob : IJobParallelFor
        {
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Dof3 value 
            /// </summary>
            public float3* Dof3s;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Dof3 to rotation value
            /// </summary>
            public quaternion* Dof3Quaternions;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public JointData* jointDatas;

            [NativeDisableUnsafePtrRestriction, ReadOnly]
            /// <summary>
            /// joint relative muscle's index
            /// </summary>
            internal int3* jointMuscleIndexs;

            [NativeDisableUnsafePtrRestriction, ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public float* muscleValues;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            ///Epsilion to rotation
            /// </summary>
            public quaternion* muscleGradientRotations;
            public void Execute(int index)
            {
                JointData* currentJoint = jointDatas +index;
                float3* Dof3 = Dof3s + index;
                quaternion* Dof3Quaternion = Dof3Quaternions + index;
                int3* jointMuscleIndex = jointMuscleIndexs+index;

/*                float3 Dof3toRadian = math.radians(
                    math.lerp(0, currentJoint->minRange, -math.clamp(*Dof3, -1, 0))
                + math.lerp(0, currentJoint->maxRange, math.clamp(*Dof3, 0, 1))
                );
*/

                float4x3 axisRotations=float4x3.zero;
                float4x3 axisRotationsEpsilion = float4x3.zero;
                for (int i = 0; i < 3; i++)
                {
                    if ((*jointMuscleIndex)[i]!=-1)
                    {
                        
                        float muscleValue = muscleValues[(*jointMuscleIndex)[i]];
                        Dof3[i] = muscleValue;
                        float dofRadian = 0;
                        if (muscleValue < 0)
                        {
                            dofRadian=math.radians( math.lerp(0, currentJoint->minRange[i], -muscleValue));
                        }
                        else
                        {
                            dofRadian = math.radians(math.lerp(0, currentJoint->maxRange[i], muscleValue));
                        }
                        axisRotations[i]= quaternion.AxisAngle(currentJoint->dof3Axis[i], dofRadian).value;

                        muscleValue += L_BFGSStatic.EPSILION;
                        if (muscleValue < 0)
                        {
                            dofRadian = math.radians(math.lerp(0, currentJoint->minRange[i], -muscleValue));
                        }
                        else
                        {
                            dofRadian = math.radians(math.lerp(0, currentJoint->maxRange[i], muscleValue));
                        }
                        axisRotationsEpsilion[i] = quaternion.AxisAngle(currentJoint->dof3Axis[i], dofRadian).value;
                    }
                }
                *Dof3Quaternion = quaternion.identity;
                for (int i = 0; i < 3; i++)
                {
                    if ((*jointMuscleIndex) [i] != -1)
                    {
                        *Dof3Quaternion = math.mul(new quaternion(axisRotations[i]), *Dof3Quaternion);

                        quaternion* muscleGradientRotation = muscleGradientRotations + (*jointMuscleIndex)[i];
                        *muscleGradientRotation = quaternion.identity;
                        for (int ii = 0; ii < 3; ii++)
                        {
                            if ((*jointMuscleIndex)[ii] != -1)
                            {
                                if (i == ii)
                                {
                                    *muscleGradientRotation = math.mul(new quaternion(axisRotationsEpsilion[ii]), *muscleGradientRotation);
                                }
                                else
                                {
                                    *muscleGradientRotation = math.mul(new quaternion(axisRotations[ii]), *muscleGradientRotation);
                                }
                            }
                        }
                    }
                }

/*                if ((*Dof3)[0] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint->dof3Axis[0], Dof3toRadian[0]), eulerAngle);
                }
                if ((*Dof3)[1] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint->dof3Axis[1], Dof3toRadian[1]), eulerAngle);
                }
                if ((*Dof3)[2] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint->dof3Axis[2], Dof3toRadian[2]), eulerAngle);
                }*/
            }
        }
        /// <summary>
        /// Clactulate Epsillion value 
        /// for get joint's gradient ,you need add a Epsilion value to the muscle 
        /// and get the loss change  ad gradient.Optimize this process,you will 
        /// get this step.
        /// </summary>
        [BurstCompile]
        public struct ClacDof3EpsilionJob : IJobParallelFor
        {
            /*            [ReadOnly,NativeDisableUnsafePtrRestriction]
                        /// <summary>
                        /// muscles value ,containing joint index and dof index.
                        /// </summary>
                        public JointMusclesData* muscleDatas;*/

            /*            [ReadOnly, NativeDisableUnsafePtrRestriction]
                        /// <summary>
                        /// joint muscles value 
                        /// </summary>
                        public float3* Dof3s;*/

            [NativeDisableUnsafePtrRestriction, ReadOnly]
            /// <summary>
            /// joint relative muscle's index
            /// </summary>
            internal int3* jointMuscleIndexs;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint muscles value's rotation 
            /// </summary>
            public quaternion* Dof3Quaternions;

/*            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// containing muscles value ,containing joint index and dof index.
            /// </summary>
            public JointData* jointDatas;*/

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint transform value 
            /// </summary>
            public RigidTransform* jointTransformNatives;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            ///Epsilion to rotation
            /// </summary>
            public quaternion* muscleGradientRotations;

            public void Execute(int index)
            {
                /*                JointMusclesData* muscleData = muscleDatas+index;
                                JointData* currentJoint = jointDatas+muscleData->jointIndex;

                                float3 Dof3 = Dof3s[muscleData->jointIndex];

                                Dof3[muscleData->dof] +=L_BFGSStatic.EPSILION;
                */
/*                int3* jointMuscleIndex = jointMuscleIndexs + index;
                quaternion* before = Dof3Quaternions + index ;
                RigidTransform* relationTransform = jointTransformNatives + index ;
                for (int i = 0; i < 3; i++)
                {
                    if ((*jointMuscleIndex)[i] != -1)
                    {
                        quaternion* after = muscleGradientRotations + (*jointMuscleIndex)[i];
                        *after = math.mul(relationTransform->rot, math.mul(math.mul(*after, math.inverse(*before)), math.inverse(relationTransform->rot)));
                    }
                }*/


/*                quaternion* after = muscleGradientRotations + index;
                *after = quaternion.identity;
                Dof3 = math.radians(
                    math.lerp(0, currentJoint->minRange, -math.clamp(Dof3, -1, 0)) +
                    math.lerp(0, currentJoint->maxRange, math.clamp(Dof3, 0, 1)));
                if (Dof3[0] != 0)
                {
                    *after = math.mul(quaternion.AxisAngle(currentJoint->dof3Axis[0], Dof3[0]), *after);
                }
                if (Dof3[1] != 0)
                {
                    *after = math.mul(quaternion.AxisAngle(currentJoint->dof3Axis[1], Dof3[1]), *after);
                }
                if (Dof3[2] != 0)
                {
                    *after = math.mul(quaternion.AxisAngle(currentJoint->dof3Axis[2], Dof3[2]), *after);
                }

                RigidTransform* relationTransform = jointTransformNatives + muscleData->jointIndex;*/

            }
        }
        /// <summary>
        /// Build Joint world transform for muscle value and local transform
        /// </summary>
        [BurstCompile]
        public struct BuildTransformJob : IJobFor
        {
            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint world transform
            /// </summary>
            public RigidTransform* jointTransformNatives;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint datas 
            /// </summary>
            public JointData* jointDatas;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint muscle value's rotation 
            /// </summary>
            public quaternion* Dof3Quaternions;

            public float3 rootPosition;
            public quaternion rootRotation;

            public void Execute(int index)
            {
                JointData* currentJoint = jointDatas+index;
                RigidTransform* currentTransform = jointTransformNatives+index;
                quaternion* dof3Quaternion = Dof3Quaternions+index;

                if (currentJoint->parentIndex == -1)
                {
                    currentTransform->pos = rootPosition;
                    currentTransform->rot = math.mul(rootRotation, *dof3Quaternion);
                }
                else
                {
                    RigidTransform* parentTransform = jointTransformNatives+currentJoint->parentIndex;

                    currentTransform->pos = parentTransform->pos + math.mul(parentTransform->rot, currentJoint->localPosition);
                    currentTransform->rot = math.mul(parentTransform->rot, math.mul(currentJoint->localRotation, *dof3Quaternion));

                }
            }
        }
        /// <summary>
        /// Get target joint's loss value 
        /// every joint will containing some constriant data,
        /// this step will get the constraint loss from the constraint 's constrain
        /// </summary>
        [BurstCompile]
        public struct CaclulatelossJob : IJobParallelFor
        {
            [ReadOnly,NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Constriant data 
            /// </summary>
            public ConstraintData* constraintDatas;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Joint relation data ,for claculate the new world transf
            /// </summary>
            public JointRelationData* jointRelationDatas;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint world transform
            /// </summary>
            public RigidTransform* jointTransformNatives;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint muscles value 
            /// </summary>
            public float3* Dof3s;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Gradient's rotation
            /// </summary>
            public quaternion* muscleGradientRotations;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Loss data 
            /// </summary>
            public double* jointlossNatives;

            public void Execute(int index)
            {

                JointRelationData* jointRelationData = jointRelationDatas+index;    
                ConstraintData* constraintNative = constraintDatas+index;
                double* jointloss = jointlossNatives+index;


                RigidTransform jointTransform = jointTransformNatives[jointRelationData->jointIndex];
                RigidTransform* pJointTransform = &jointTransform;

                if (jointRelationData->relatedJointIndex!=-1)
                {
                    RigidTransform* relatedTransform = jointTransformNatives+jointRelationData->relatedJointIndex;
                    quaternion* muscleGradientRotation = muscleGradientRotations+jointRelationData->relatedMuscleIndex;
                    RebuildJointTransform(pJointTransform,  relatedTransform, muscleGradientRotation);
                }
              float3* Dof3 = Dof3s+jointRelationData->jointIndex;

                *jointloss = 0;
                if (constraintNative->positionConstraint.isVaild)
                {
                    UpdatePositionloss(jointloss, pJointTransform, constraintNative);
                }
/*                if (constraintNative.directionConstraint.isVaild)
                {
                    UpdatePositionloss(ref jointloss, ref jointTransform, ref constraintNative);
                }*/

                /*                if (constraintNative.DofConstraint.isVaild)
                                {
                                    UpdateMuscleloss(ref jointloss, ref Dof3, ref constraintNative);
                                }
                                if (constraintNative.lookAtConstraint.isVaild)
                                {
                                    UpdateLookAtloss(ref jointloss, ref jointTransform, ref constraintNative);
                                }
                                if (constraintNative.colliderConstraint.isVaild)
                                {
                                    UpdateColliderConstraint(ref jointloss, ref jointTransform, ref constraintNative);
                                }
                                if (constraintNative.positionChangeConstraint.isVaild)
                                {
                                    UpdatePositionChangeloss(ref jointloss, ref jointTransform, ref constraintNative);
                                }
                                if (constraintNative.DofChangeConstraint .isVaild)
                                {
                                    UpdateMuscleChangeloss(ref jointloss, ref Dof3, ref constraintNative);
                                }*/
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void RebuildJointTransform( RigidTransform* jointTransform, RigidTransform* relatedTransform, quaternion* muscleGradientRotation)
            {
                float3 direction = jointTransform->pos - relatedTransform->pos;
                jointTransform->pos = math.mul(*muscleGradientRotation,direction)+ relatedTransform->pos;
                jointTransform->rot = math.mul( *muscleGradientRotation, jointTransform->rot);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdatePositionloss(double* jointloss,RigidTransform* jointTransform,ConstraintData* constraintNative)
            {
                float3 jointPosition = jointTransform->pos;
                float lengthSum = constraintNative->lengthSum;

                float3 weight3 = constraintNative->positionConstraint.weight3;
                float3 constraintPosition = constraintNative->positionConstraint.position;
                float3 direction = constraintPosition - jointPosition;

                if (math.all(direction==0))
                {
                    return;
                }

                float lossCos = math.csum(direction* direction* weight3);
                lossCos /= ( lengthSum* lengthSum);
                lossCos *= math.PI * math.PI;
                *jointloss += lossCos;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateMuscleloss(ref JoinLoss jointloss, ref float3 Dof3, ref ConstraintData constraintNative)
            {
                float3 tolerance3 = constraintNative.DofConstraint.tolerance3;
                float3 weight3 = constraintNative.DofConstraint.weight3;
                float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3,0);
                float loss =math.csum(Dof3Outside * weight3);
                jointloss.lossSum += loss;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateLookAtloss(ref JoinLoss jointloss, ref RigidTransform jointTransform, ref ConstraintData constraintNative)
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
                jointloss.lossSum += loss * loss*weight;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateColliderConstraint(ref JoinLoss jointloss, ref RigidTransform jointTransform, ref ConstraintData constraintNative)
            {
                //OYM：啊这个超级难写 
                //OYM：还要去构造AABB
                //OYM：不想写（摆烂
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdatePositionChangeloss(ref JoinLoss jointloss, ref RigidTransform jointTransform, ref ConstraintData constraintNative)
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

                //loss =;

                jointloss.lossSum += loss;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateMuscleChangeloss(ref JoinLoss jointloss, ref float3 Dof3, ref ConstraintData constraintNative)
            {
                float3 oldDof3 = constraintNative.DofChangeConstraint .oldDof3;
                float3 tolerance3 = constraintNative.DofChangeConstraint .tolerance3;
                float3 weight3 = constraintNative.DofChangeConstraint .weight3;

                float3 Dof3Change = math.abs( Dof3 - oldDof3);
                Dof3Change = math.max(0, Dof3Change - tolerance3)* weight3;
                float loss = math.csum(Dof3Change)/3;
                jointloss.lossSum += loss * loss;
            }
        }
       [BurstCompile(FloatPrecision.High, FloatMode.Strict)]

        /// <summary>
        /// Main controller ,will collect loss and gradient
        /// and do BFGS solve.
        /// </summary>
        public struct MainControllerJob : IJob
        {
            //OYM：感觉计算量超级的大啊
            //OYM：等我回来在写注释
            //OYM：趁着现在灵感还在

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// BFGS 求解应用到的变量
            /// </summary>
            public LBFGSSolver* LBFGSSolver;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            internal GlobalData* globalData;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Gradient 
            /// </summary>
            public double* gradients;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// LBFGS data Store 
            /// </summary>
            public double* dataStore;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// loss value store
            /// </summary>
            internal double* losses;
            /*            [NativeDisableParallelForRestriction]
                        internal double* gradientAlls;
                        [NativeDisableParallelForRestriction]
                        internal float* muscleAlls;*/

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint loss
            /// </summary>
            public double* jointlossNatives;

            [NativeDisableUnsafePtrRestriction, ReadOnly]
            /// <summary>
            /// relative data
            /// </summary>
            public JointRelationData* relationDatas;
            
            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public float* muscleValues;

            /// <summary>
            /// constraint length
            /// </summary>
            public int constraintLength;
            /// <summary>
            ///  parallel's data count
            /// </summary>
            public int parallelLength;
            /// <summary>
            ///  variable's count ,is the same as muscle's count.
            /// </summary>
            public int muscleLength;
            /// <summary>
            /// joint count ,
            /// </summary>
            public int jointLength;

            public void Execute()
            {

                 double loss = Collectloss(jointlossNatives, 0, constraintLength, jointLength);
                CollectGradient(jointlossNatives, relationDatas, muscleLength, jointLength, parallelLength, gradients);
               // CollectTestData(globalData);
                losses[globalData->leastLoopCount] = loss;
                LBFGSSolver->Optimize(loss,  ref globalData->leastLoopCount, dataStore, muscleValues, gradients);
            }

/*            private void CollectTestData(GlobalData globalData)
            {
                UnsafeUtility.MemCpy((double*)gradientAlls.GetUnsafePtr() + globalData.leastLoopCount * muscleLength, gradients.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<double>());
                UnsafeUtility.MemCpy((float*)muscleAlls.GetUnsafePtr() + globalData.leastLoopCount * muscleLength, muscleValues.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<float>());
            }*/

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static double Collectloss(double* losses, int offset, int constraintlength,int jointLength)
            {
                double loss = 0;
                if (constraintlength!=0)
                {
                    for (int i = 0; i < jointLength; i++)
                    {
                        loss +=losses[i + offset]/ constraintlength;
                    }
                }

                return loss;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void CollectGradient(double* losses,JointRelationData* relationDatas , int muscleLength, int jointLength, int parallelLength,double*gradients)
            {
                ClearNativeArrayData<double>(gradients,muscleLength);
                for (int i = jointLength; i < parallelLength; i++)
                {
                    JointRelationData* relationData = relationDatas+i;
                    double gradientTemp = (double)losses[i] - (double)losses[relationData->jointIndex];
                    //gradientTemp = gradientTemp /jointRelativedCounts[relationData.relatedJointIndex] *jointRelativedCounts[0];
                    gradientTemp /= L_BFGSStatic.EPSILION;
                    

                    gradients[relationData->relatedMuscleIndex] += gradientTemp;
                    //gradients[relationData.relatedMuscleIndex] += gradientTemp;
                }

            }
        }

        //OYM：阿伟去写点正经的东西好不好
        //OYM：老想着优化是没有前途的
        [BurstCompile]
        public struct TestJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction]
            public NativeSlice<float> testData;
            public void Execute(int index)
            {
                testData[index] = math.cos(testData[index]);
            }
        }
    }
}
//待考证
/*
public struct RigHipPositionJob : IJobParallelFor
{
    [ReadOnly,NativeDisableParallelForRestriction]
    public JointData* jointDatas;
    [ReadOnly, NativeDisableParallelForRestriction]
    public ConstraintData* constraintNatives;
    [NativeDisableParallelForRestriction]
    public RigidTransform* jointTransformNatives;

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
                ConstraintData jointConstraint = constraintNatives[offset+i];

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
*/
/*        [BurstCompile]
        public struct MuscleToJointJob : IJobParallelFor
        {

            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            internal MMGlobalData* globalDataNative;
            [NativeDisableParallelForRestriction]
            public float3* Dof3s;
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public MMMuscleData* muscleDatas;
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public float* musclesValues;
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

/*     
/// <summary>
///  Initialize Muscle Value
/// </summary>
public struct InitializeMuscleJob : IJobParallelFor
{
    [ReadOnly]
    /// <summary>
    /// muscles value ,containing joint index and dof index.
    /// </summary>
    public MuscleData* muscleDatas;
    /// <summary>
    /// muscles value ,containing joint index and dof index.
    /// </summary>
    public float* musclesValues;

    public void Execute(int index)
    {
        musclesValues[index] = math.clamp(musclesValues[index],0,0);
    }
}
/// <summary>
/// Set joint Transform data before update
/// </summary>
public struct InitializeJointJob : IJobParallelFor
{
    /// <summary>
    /// Joint's world transform data
    /// </summary>
    public RigidTransform* jointTransformDatas;
    /// <summary>
    /// Joint's world transform data
    /// </summary>
    public Vector3* jointPositions;
    /// <summary>
    /// Joint's world transform data
    /// </summary>
    public quaternion* jointRotations;
    public void Execute(int index)
    {
        var rigid = jointTransformDatas[ index];
        rigid.pos = jointPositions[index];
        rigid.rot = jointRotations[index];
        jointTransformDatas[index] = rigid;

    }
}
/// <summary>
/// To read constraint data from transform. e.g. position constriant,lookat constraint 
/// </summary>
[BurstCompile]
public struct TransformToConstraintJob : IJobParallelFor
{
    /// <summary>
    /// constraint Native data
    /// </summary>
    [NativeDisableParallelForRestriction]
    public ConstraintData* constraintDatas;
    /// <summary>
    /// TransformToConstrain Data 
    /// </summary>
    public TransformToConstraintData* transformToConstrainDatas;
    /// <summary>
    ///  constraint positions
    /// </summary>
    public Vector3* constraintPositions;
    /// <summary>
    /// TransformToConstrain Data 
    /// </summary>
    public Quaternion* constraintRotations;
    public void Execute(int index)
    {
        int point = transformToConstrainDatas[index].jointIndex;
        MMConstraintType type = transformToConstrainDatas[index].constraintType;
         var constraintPosition=constraintPositions[index];
        var constraintRotation=constraintRotations[index];

       var constraint = constraintDatas[ point];

        switch (type)
        {
            case MMConstraintType.Position:
                constraint.positionConstraint.position = constraintPosition;
                break;
            case MMConstraintType.Rotation:
                break;
            case MMConstraintType.LookAt:
                constraint.lookAtConstraint.position = constraintPosition;
                break;
            default:
                break;
        }
        constraintDatas[point] = constraint;
    }
}
        public struct JointToTransformJob : IJobParallelForTransform
        {

            [ReadOnly]
            public RigidTransform* jointTransformNatives;
            public void Execute(int index, TransformAccess transform)
            {
                transform.position = jointTransformNatives[index].pos;
                transform.rotation=jointTransformNatives[index].rot;
            }
        }
 */