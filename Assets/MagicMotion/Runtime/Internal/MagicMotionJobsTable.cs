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
        [BurstCompile(OptimizeFor = OptimizeFor.Performance)]
        public struct CopyConstraintDataJob : IJobFor
        {
            [NoAlias]
            /// <summary>
            /// Joint relation data 
            /// </summary>
            public NativeArray<ParallelRelationData> parallelRelationDatas;

            [NoAlias,NativeDisableParallelForRestriction]
            /// <summary>
            /// constraint data 
            /// </summary>
            public NativeArray<ConstraintData> constraintDatas;

            public int jointLength;
            public void Execute(int index)
            {
                constraintDatas[ index]= constraintDatas[parallelRelationDatas [ index].jointIndex];
            }
        }


        /// <summary>
        /// Set muscles data 
        /// </summary>
        [BurstCompile(OptimizeFor = OptimizeFor.Performance)]
        public struct SortingFactoryJob : IJob
        {
            /// <summary>
            /// all group's setting
            /// </summary>
            public NativeArray<GroupSettingData> settingData;
            /// <summary>
            /// all group's loss
            /// </summary>
            public NativeArray<GroupLossData> groupLosses;
            /// <summary>
            /// all group's muscle value
            /// </summary>
            public NativeArray<float> musclesValues;
            /// <summary>
            /// last iterationMuscleValue;
            /// </summary>
            internal NativeArray<float> oldMuslceValues;

            public void Execute()
            {
                var setting = settingData[0];

                //OYM：后面会想办法接入遗传算法或者粒子群算法什么的吧(大概)
                //OYM：可以利用梯度获取比较优的梯度值,可以依据loss进行杂交,可以依梯度正交矩阵计算正交线上的值
                //OYM：有很多很棒的想法,不过先一笔带过,先拿最稳定的结果来计算



                for (int i = 0; i < settingData[0].groupLength; i++)
                {
                    var loss = groupLosses[i];
                    loss.index = i;
                    groupLosses[i] = loss;
                }
                groupLosses.Sort();
                int bestStartPoint = groupLosses[0].index;
                NativeArray<float>.Copy(musclesValues, bestStartPoint * setting.muscleLength, musclesValues, 0, setting.muscleLength);

                if (setting.outsideLoopIndex == setting.outsideLoopCount - 1)
                {
                    return;
                }

                //OYM：查重
                float originDimLength = 0;
                for (int i = 0; i < setting.muscleLength; i++)
                {
                    originDimLength += musclesValues[i] * musclesValues[i];
                }
                originDimLength = math.sqrt(originDimLength);
                for (int groupIndex = 1; groupIndex < setting.groupLength; groupIndex++)
                {
                    float targetDimLength = 0;
                    float dot = 0;
                    for (int ii = 0; ii < setting.muscleLength; ii++)
                    {
                        int muscleIndex = ii + groupIndex * setting.muscleLength;
                        targetDimLength += musclesValues[muscleIndex] * musclesValues[muscleIndex];
                        dot += musclesValues[muscleIndex] * musclesValues[ii];
                    }
                    targetDimLength=math.sqrt(targetDimLength);

                    float cos = dot  / (targetDimLength * originDimLength);
                    if (cos < 0.99f&& targetDimLength!=0&& originDimLength!=0)//OYM：查重合格
                    {
                        continue;
                    }
                    for (int ii = 0; ii < setting.muscleLength; ii++)
                    {
                        int muscleIndex = ii + groupIndex * setting.muscleLength;
                        float bestMusles = musclesValues[ii];
                        float result = GetValueByGroup(groupIndex, bestMusles, ii);
                        musclesValues[muscleIndex] = result;
                    }
                    
                }

                oldMuslceValues.CopyFrom(musclesValues);//OYM：虽然不知道有什么用,但是总感觉会用上

                setting.outsideLoopIndex++;
                settingData[0] = setting;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static float GetValueByGroup(int groupIndex, float refValue, int index)
            {
                switch (groupIndex)
                {
                    case 0:
                        break;
                    case 1:
                        refValue = 0;
                        break;
                    case 2:
                        //refValue = math.lerp(refValue, -1, 1e-4f * lastLoss);
                        refValue = 1;
                        break;
                    case 3:
                        //refValue = math.lerp(refValue, 1, 1e-4f * lastLoss);
                        refValue = -1;
                        break;
                    case 4:
                        refValue = math.lerp(refValue, -1, 1e-3f );
                        break;
                    case 5:
                        refValue = math.lerp(refValue, 1, 1e-3f );
                        break;
                    case 6:
                        refValue = math.lerp(refValue, -1, 1e-2f );
                        break;
                    case 7:
                        refValue = math.lerp(refValue, 1, 1e-2f );
                        break;
                    case >= 8 and < 16:
                        refValue = math.clamp(refValue * groupIndex / 16f, -1, 1);
                        break;
                    case >= 16 and < 32:
                        refValue = math.lerp(-1, 1, (groupIndex - 16) / 16f);
                        break;
                    default:
                        break;
                }
                return refValue;
            }
        }
        /*            /// <summary>
                    ///  muscles to joint dof value 
                    /// </summary>
                    [BurstCompile]
                public struct MuscleToDof3Job : IJobParallelFor
                {
                    [ReadOnly, NativeDisableUnsafePtrRestriction]
                    /// <summary>
                    /// muscles value ,containing joint index and dof index.
                    /// </summary>
                    public MusclesData* muscleDatas;

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
                        MusclesData* muscleData = muscleDatas+index;
                        Dof3s[muscleData->jointIndex][muscleData->dof] = muscleValues[ index];
                    }
                }*/
        [BurstCompile(FloatMode = FloatMode.Default, OptimizeFor = OptimizeFor.Performance)]
        public struct LinerSearchJob : IJobFor
        {
            #region ReadOnly
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Group Setting data
            /// Only one
            /// </summary>
            public  GroupSettingData* settingData;

            [NativeDisableParallelForRestriction, ReadOnly]
            /// <summary>
            /// relative data
            /// </summary>
            public NativeArray< ParallelRelationData> parallelRelationDatas;

            [ReadOnly, NativeDisableParallelForRestriction]
            /// <summary>
            /// Constriant data 
            /// </summary>
            public NativeArray< ConstraintData> constraintDatas;

            [ReadOnly, NativeDisableParallelForRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray< JointData> jointDatas;

            [NativeDisableParallelForRestriction, ReadOnly]
            /// <summary>
            /// joint relative muscle's index
            /// </summary>
            public NativeArray<int3> jointMuscleIndexs;

            [NativeDisableParallelForRestriction, ReadOnly]
            /// <summary>
            /// joint relative constraints count
            /// </summary>
            public NativeArray<int> jointConstraintRelativeCounts;
            #endregion

            #region ReadWrite
            [NativeDisableParallelForRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray< float> muscleValues;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// Dof3 value 
            /// </summary>
            public NativeArray< float3> Dof3s;

/*            [NativeDisableParallelForRestriction]
            /// <summary>
            /// Dof3 to rotation value
            /// </summary>
            public NativeArray< quaternion> muscleCurrentRotations;*/

            [NativeDisableParallelForRestriction]
            /// <summary>
            ///Epsilion to rotation
            /// </summary>
            public NativeArray< quaternion> muscleGradientRotations;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// joint world transform
            /// </summary>
            public NativeArray< RigidTransform> jointTransformNatives;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// BFGS-Solver containing bfgs temp data
            /// Only 1
            /// </summary>
            public  LBFGSSolver* LBFGSSolver;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// This group's loss
            /// Only 1
            /// </summary>
            public  GroupLossData* currentGroupLoss;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// Gradient 
            /// </summary>
            public NativeArray< double> gradients;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// LBFGS data Store 
            /// </summary>
            public NativeArray< double> dataStore;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// loss value store
            /// </summary>
            public NativeArray<double> lossesRecorder;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// joint loss
            /// </summary>
            public NativeArray< double> jointLosses;

            #endregion

            public void Execute(int loopIndex)
            {
                int leastLoopCount = settingData[0].insideLoopCount - loopIndex - 1;
                #region Muscle To Transform
                for (int index = 0; index < settingData[0].jointLength; index++)
                {

                    JointData currentJoint = jointDatas [ index];
                    int3 jointMuscleIndex = jointMuscleIndexs [index];

                    float3 Dof3 = Dof3s [ index];
                    //quaternion currentRotation = muscleCurrentRotations[index];
                    RigidTransform currentTransform = jointTransformNatives [ index];

                    //Convert muscle value to rotation without and with epsilion at the same time
                    #region Step1: Muslce Tot Rotation
                    float4x3 axisRotations = float4x3.zero;
                    float4x3 axisRotationsEpsilion = float4x3.zero;
                    for (int i = 0; i < 3; i++)
                    {
                        if (jointMuscleIndex[i] != -1)
                        {
                            //OYM：Read Muscle vale
                            int muscleIndex = jointMuscleIndex[i];
                            float muscleValue = muscleValues[muscleIndex];
                            Dof3[i] = muscleValue;

                            //OYM：build CurrentRotation
                            float dofRadian = 0;
                            if (muscleValue < 0)
                            {
                                dofRadian = math.radians(math.lerp(0, currentJoint.minRange[i], -muscleValue));
                            }
                            else
                            {
                                dofRadian = math.radians(math.lerp(0, currentJoint.maxRange[i], muscleValue));
                            }
                            axisRotations[i] = quaternion.AxisAngle(currentJoint.dof3Axis[i], dofRadian).value;

                            //OYM：Build Gradient use rotation
                            muscleValue += L_BFGSStatic.EPSILION;
                            if (muscleValue < 0)
                            {
                                dofRadian = math.radians(math.lerp(0, currentJoint.minRange[i], -muscleValue));
                            }
                            else
                            {
                                dofRadian = math.radians(math.lerp(0, currentJoint.maxRange[i], muscleValue));
                            }

                            axisRotationsEpsilion[i] = quaternion.AxisAngle(currentJoint.dof3Axis[i], dofRadian).value;
                        }
                    }

                    //OYM：Clac current rotation
                    quaternion currentRotation = quaternion.identity;
                    for (int i = 0; i < 3; i++)
                    {
                        if (jointMuscleIndex[i] != -1)
                        {
                            int muscleIndex = jointMuscleIndex[i];
                            currentRotation = math.mul(new quaternion(axisRotations[i]), currentRotation);
                           // muscleCurrentRotations[index] = currentRotation;

                            //OYM：Clac gradient rotation
                            quaternion muscleGradientRotation = muscleGradientRotations [muscleIndex];
                            muscleGradientRotation = quaternion.identity;
                            for (int ii = 0; ii < 3; ii++)
                            {
                                if (jointMuscleIndex[ii] != -1)
                                {
                                    if (i == ii)
                                    {
                                        muscleGradientRotation = math.mul(new quaternion(axisRotationsEpsilion[ii]), muscleGradientRotation);
                                    }
                                    else
                                    {
                                        muscleGradientRotation = math.mul(new quaternion(axisRotations[ii]), muscleGradientRotation);
                                    }
                                }
                            }
                            muscleGradientRotations[muscleIndex]=muscleGradientRotation;
                        }
                    }

                    #endregion

                    // Build Joint world transform for muscle value and local transform
                    #region Step2: Rotation To JointTransform

                    if (currentJoint.parentIndex == -1)
                    {
                        //OYM： build root position
                        currentTransform.pos = settingData[0].rootTransform.pos;
                        //OYM：build root rotation
                        currentTransform.rot = math.mul(settingData[0].rootTransform.rot, currentRotation);
                    }
                    else
                    {
                        //OYM：get parent transform
                        RigidTransform parentTransform = jointTransformNatives[currentJoint.parentIndex];
                        //OYM：build child position
                        currentTransform.pos = parentTransform.pos + math.mul(parentTransform.rot, currentJoint.localPosition);
                        //OYM：build child rotation
                        currentTransform.rot = math.mul(parentTransform.rot, math.mul(currentJoint.localRotation, currentRotation));
                    }
                    jointTransformNatives[index] = currentTransform;
                    #endregion

                    // Clactulate Epsillion value 
                    // for get joint's gradient ,you need add a Epsilion value to the muscle 
                    // and get the loss change  ad gradient.Optimize this process,you will 
                    // get this step.
                    #region Step3: Transform To GradientRotation

                    for (int i = 0; i < 3; i++)
                    {
                        if (jointMuscleIndex[i] != -1)
                        {
                            //OYM：get rotation after add epsilion
                            quaternion after = muscleGradientRotations [jointMuscleIndex[i]];
                            //OYM：get rotation add epsilion's change in joint transform
                            //OYM：for example ,if you want know how influence for epsilion's change ,you can just get currentTransform->rot*after to clac next;
                            after = math.mul(currentTransform.rot, math.mul(math.mul(after, math.inverse(currentRotation)), math.inverse(currentTransform.rot)));
                            muscleGradientRotations[jointMuscleIndex[i]]=after;
                        }
                    }
                    #endregion

                }
                #endregion

                #region Clac Loss
                ClearNativeArrayData<double>(jointLosses);
                for (int index = 0; index < settingData[0].jointLength; index++)
                {

                    ParallelRelationData parallelRelationData = parallelRelationDatas [index];
                    ConstraintData constraintData = constraintDatas[ index];
                    double jointLoss = jointLosses[index];
                    RigidTransform jointTransform = jointTransformNatives[ index];
                    float3 Dof3 = Dof3s [ index];

                    if (constraintData.positionConstraint.isVaild)
                    {
                        ClacPositionLoss(constraintData, jointTransform,ref jointLoss);
                    }
                    if (constraintData.directionConstraint.isVaild)
                    {
                        RigidTransform parentTransform = parallelRelationData.parentJointIndex == -1 ? RigidTransform.identity : jointTransformNatives[parallelRelationData.parentJointIndex];

                        ClacDirectionLoss(constraintData, jointTransform, parentTransform, ref jointLoss);
                    }
                    if (constraintData.positionChangeConstraint.isVaild)
                    {
                        ClacPositionChangeLoss(constraintData, jointTransform, ref jointLoss);
                    }
                    if (constraintData.DofChangeConstraint.isVaild)
                    {
                        ClacMuscleChangeLoss(constraintData, Dof3, ref jointLoss);
                    }
                  jointLosses[index] = jointLoss;
                }
                #endregion

                #region Clac Gradient 

                for (int index = settingData[0].jointLength; index < settingData[0].parallelLength; index++)
                {
                    ParallelRelationData parallelRelationData = parallelRelationDatas[ index];
                    ConstraintData constraintData = constraintDatas [ index];
                    double jointLoss = jointLosses [ index];
                    float3 Dof3 = Dof3s [parallelRelationData.jointIndex];

                    RigidTransform jointTransform = jointTransformNatives[parallelRelationData.jointIndex];

                    #region  ReBuildTransform

                    RigidTransform relatedTransform = jointTransformNatives[parallelRelationData.relatedJointIndex];
                    quaternion muscleGradientRotation = muscleGradientRotations [parallelRelationData.relatedMuscleIndex];

                    if (parallelRelationData.relatedJointIndex != parallelRelationData.jointIndex)
                    {
                        RebuildTransform(relatedTransform,ref jointTransform, muscleGradientRotation);
                    }

                    #endregion

                    if (constraintData.positionConstraint.isVaild)
                    {
                        ClacPositionLoss(constraintData, jointTransform,ref jointLoss);
                    }
                    if (constraintData.directionConstraint.isVaild)
                    {
                        RigidTransform parentTransform = parallelRelationData.parentJointIndex == -1 ? RigidTransform.identity : jointTransformNatives[parallelRelationData.parentJointIndex];
                        if (parallelRelationData.parentJointIndex >= parallelRelationData.relatedJointIndex)
                        {
                            RebuildTransform(relatedTransform, ref parentTransform, muscleGradientRotation);
                        }

                        ClacDirectionLoss(constraintData, jointTransform, parentTransform,ref jointLoss);
                    }
                    if (constraintData.positionChangeConstraint.isVaild)
                    {
                        ClacPositionChangeLoss(constraintData, jointTransform,ref jointLoss);
                    }
                    if (constraintData.DofChangeConstraint.isVaild)
                    {
                        ClacMuscleChangeLoss(constraintData, Dof3,ref jointLoss);
                    }
                    jointLosses[index]=jointLoss;
                }
                #endregion

                #region Collect Loss 

                ClearNativeArrayData<double>(gradients);
                for (int i = settingData[0].jointLength; i < settingData[0].parallelLength; i++)
                {
                    ParallelRelationData relationData = parallelRelationDatas[i];
                    double gradientTemp = jointLosses[i] - jointLosses[relationData.jointIndex];

                    gradientTemp /= L_BFGSStatic.EPSILION;
                    gradientTemp *= settingData[0].loopConvergence;
                    gradients[relationData.relatedMuscleIndex] += gradientTemp;
                }
                double loss = 0;
                for (int i = 0; i < settingData[0].jointLength; i++)
                {
                    loss += jointLosses[i] / settingData[0].constraintLength;

                    /* int3 jointMuscleIndex = jointMuscleIndexs[i];
                                       int constraintCount = jointConstraintRelativeCounts[i];
                                        for (int ii = 0; ii < 3; ii++)
                                        {
                                            if ((*jointMuscleIndex)[ii] != -1)
                                            {
                                                double* gradient = gradients + (*jointMuscleIndex)[ii];
                                                *gradient  /= constraintCount;
                                            }
                                        }*/
                }

                #endregion

                #region LBFGS-Optimize
                // CollectTestData(globalData);

                LBFGSSolver->Optimize(leastLoopCount, ref loss, dataStore, muscleValues, gradients);
                currentGroupLoss->loss = loss;

                #endregion
                //OYM：for test,must be removed in environments other than develop
                lossesRecorder[loopIndex + settingData[0].outsideLoopIndex * settingData[0].insideLoopCount] = loss;

            }

            private void ClacPositionChangeLoss(ConstraintData constraintData, RigidTransform jointTransform,ref double jointLoss)
            {
                //OYM：还没有思考出震颤的问题怎么解决
                float3 jointPosition = jointTransform.pos;
                float lengthSum = constraintData.lengthSum;

                float3 tolerance = constraintData.positionChangeConstraint.tolerance3;
                float3 weight3 = constraintData.positionChangeConstraint.weight3;
                float3 constraintPosition = constraintData.positionChangeConstraint.oldPosition;
                float3 direction = (constraintPosition - jointPosition);

                if (math.any(direction != 0))
                {
                    double lossCos = math.csum(direction * direction * weight3);
                    lossCos *= 0.5f;
                    lossCos /= (lengthSum * lengthSum);
                    //lossCos *= math.PI* math.PI;
                    jointLoss += lossCos;
                }
            }

            private void ClacMuscleChangeLoss(ConstraintData constraintData, float3 dof3,ref double jointLoss)
            {
                float3 oldDof3 = constraintData.DofChangeConstraint.oldDof3;
                float3 tolerance3 = constraintData.DofChangeConstraint.tolerance3;
                float3 weight3 = constraintData.DofChangeConstraint.weight3;

                float3 Dof3Change = math.abs(dof3 - oldDof3);
                Dof3Change = math.max(0, Dof3Change - tolerance3) * weight3;
                double loss = math.csum(Dof3Change);
                jointLoss += loss * loss;
            }

            private static void RebuildTransform(RigidTransform relatedTransform,ref RigidTransform jointTransform, quaternion muscleGradientRotation)
            {
                jointTransform.pos = math.mul(muscleGradientRotation, jointTransform.pos - relatedTransform.pos) + relatedTransform.pos;
                jointTransform.rot = math.mul(muscleGradientRotation, jointTransform.rot);
            }

            private static void ClacDirectionLoss(ConstraintData constraintData, RigidTransform jointTransform, RigidTransform parentTransform,ref double jointLoss)
            {
                float3 targetDirection = constraintData.directionConstraint.direction;
                float weight = constraintData.directionConstraint.weight;
                float tolerance = constraintData.directionConstraint.tolerance;
                float lengthSum = constraintData.lengthSum;

                float3 jointPosition = jointTransform.pos;
                quaternion jointRotation = jointTransform.rot;
                float3 parentPosition = parentTransform.pos;


                float3 originDirection = jointPosition - parentPosition;

                float length = math.length(originDirection);
                float3 targetPosition = targetDirection * length + parentPosition;

                float3 direction = (targetPosition - jointPosition);

                if (math.any(direction != 0))
                {
                    double lossCos = math.csum(direction * direction * weight);
                    lossCos /= (lengthSum * lengthSum);
                    //lossCos *= math.PI * math.PI;
                    jointLoss += lossCos;
                }

            }

            private static void ClacPositionLoss(ConstraintData constraintData, RigidTransform jointTransform,ref double jointLoss)
            {
                float3 jointPosition = jointTransform.pos;
                float lengthSum = constraintData.lengthSum;

                float3 weight3 = constraintData.positionConstraint.weight3;
                float3 constraintPosition = constraintData.positionConstraint.position;
                float3 direction = (constraintPosition - jointPosition);

                if (math.any(direction != 0))
                {
                    double lossCos = math.csum(direction * direction * weight3);
                    //lossCos /= (lengthSum * lengthSum);
                    //lossCos *= math.PI* math.PI;
                    jointLoss += lossCos;
                }
            }
        }
        /// <summary>
        /// Optimize Function
        ///  It's been optimized many times, 
        ///  so it seems a little hard to understand,
        ///  I'm sorry, my naming skills are not good enough to handle it :(
        /// </summary>
        [BurstCompile(FloatMode = FloatMode.Default, OptimizeFor = OptimizeFor.Performance)]

        public struct LinerSearchJobUnsafe : IJobFor
        {
            #region ReadOnly
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Group Setting data
            /// Only one
            /// </summary>
            public GroupSettingData* settingData;

            [NativeDisableUnsafePtrRestriction, ReadOnly]
            /// <summary>
            /// relative data
            /// </summary>
            public ParallelRelationData* parallelRelationDatas;

            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Constriant data 
            /// </summary>
            public ConstraintData* constraintDatas;

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
            /// joint relative constraints count
            /// </summary>
            internal int* jointConstraintRelativeCounts;
            #endregion

            #region ReadWrite
            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public float* muscleValues;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Dof3 value 
            /// </summary>
            public float3* Dof3s;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// Dof3 to rotation value
            /// </summary>
            public quaternion* muscleCurrentRotations;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            ///Epsilion to rotation
            /// </summary>
            public quaternion* muscleGradientRotations;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint world transform
            /// </summary>
            public RigidTransform* jointTransformNatives;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// BFGS-Solver containing bfgs temp data
            /// Only 1
            /// </summary>
            public LBFGSSolver* LBFGSSolver;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// This group's loss
            /// Only 1
            /// </summary>
            public GroupLossData* currentGroupLoss;

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
            internal double* lossesRecorder;

            [NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// joint loss
            /// </summary>
            public double* jointLosses;

            #endregion

            public void Execute(int loopIndex)
            {
                int leastLoopCount = settingData->insideLoopCount - loopIndex - 1;
                #region Muscle To Transform
                for (int index = 0; index < settingData->jointLength; index++)
                {

                    JointData* currentJoint = jointDatas + index;
                    int3* jointMuscleIndex = jointMuscleIndexs + index;

                    float3* Dof3 = Dof3s + index;
                    quaternion* currentRotation = muscleCurrentRotations + index;
                    RigidTransform* currentTransform = jointTransformNatives + index;

                    //Convert muscle value to rotation without and with epsilion at the same time
                    #region Step1: Muslce Tot Rotation
                    float4x3 axisRotations = float4x3.zero;
                    float4x3 axisRotationsEpsilion = float4x3.zero;
                    for (int i = 0; i < 3; i++)
                    {
                        if ((*jointMuscleIndex)[i] != -1)
                        {
                            //OYM：Read Muscle vale
                            int muscleIndex = (*jointMuscleIndex)[i];
                            float muscleValue = muscleValues[muscleIndex];
                            Dof3[i] = muscleValue;

                            //OYM：build CurrentRotation
                            float dofRadian = 0;
                            if (muscleValue < 0)
                            {
                                dofRadian = math.radians(math.lerp(0, currentJoint->minRange[i], -muscleValue));
                            }
                            else
                            {
                                dofRadian = math.radians(math.lerp(0, currentJoint->maxRange[i], muscleValue));
                            }
                            axisRotations[i] = quaternion.AxisAngle(currentJoint->dof3Axis[i], dofRadian).value;

                            //OYM：Build Gradient use rotation
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

                    //OYM：Clac current rotation
                    *currentRotation = quaternion.identity;
                    for (int i = 0; i < 3; i++)
                    {
                        if ((*jointMuscleIndex)[i] != -1)
                        {
                            int muscleIndex = (*jointMuscleIndex)[i];
                            *currentRotation = math.mul(new quaternion(axisRotations[i]), *currentRotation);

                            //OYM：Clac gradient rotation
                            quaternion* muscleGradientRotation = muscleGradientRotations + muscleIndex;
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

                    #endregion

                    // Build Joint world transform for muscle value and local transform
                    #region Step2: Rotation To JointTransform

                    if (currentJoint->parentIndex == -1)
                    {
                        //OYM： build root position
                        currentTransform->pos = settingData->rootTransform.pos;
                        //OYM：build root rotation
                        currentTransform->rot = math.mul(settingData->rootTransform.rot, *currentRotation);
                    }
                    else
                    {
                        //OYM：get parent transform
                        RigidTransform* parentTransform = jointTransformNatives + currentJoint->parentIndex;
                        //OYM：build child position
                        currentTransform->pos = parentTransform->pos + math.mul(parentTransform->rot, currentJoint->localPosition);
                        //OYM：build child rotation
                        currentTransform->rot = math.mul(parentTransform->rot, math.mul(currentJoint->localRotation, *currentRotation));
                    }
                    #endregion

                    // Clactulate Epsillion value 
                    // for get joint's gradient ,you need add a Epsilion value to the muscle 
                    // and get the loss change  ad gradient.Optimize this process,you will 
                    // get this step.
                    #region Step3: Transform To GradientRotation

                    for (int i = 0; i < 3; i++)
                    {
                        if ((*jointMuscleIndex)[i] != -1)
                        {
                            //OYM：get rotation after add epsilion
                            quaternion* after = muscleGradientRotations + (*jointMuscleIndex)[i];
                            //OYM：get rotation add epsilion's change in joint transform
                            //OYM：for example ,if you want know how influence for epsilion's change ,you can just get currentTransform->rot*after to clac next;
                            *after = math.mul(currentTransform->rot, math.mul(math.mul(*after, math.inverse(*currentRotation)), math.inverse(currentTransform->rot)));
                        }
                    }
                    #endregion

                }
                #endregion

                #region Clac Loss
                ClearNativeArrayData<double>(jointLosses, settingData->parallelLength);
                for (int index = 0; index < settingData->jointLength; index++)
                {

                    ParallelRelationData* parallelRelationData = parallelRelationDatas + index;
                    ConstraintData* constraintData = constraintDatas + index;
                    double* jointLoss = jointLosses + index;
                    RigidTransform* jointTransform = jointTransformNatives + index;
                    float3* Dof3 = Dof3s + index;

                    if (constraintData->positionConstraint.isVaild)
                    {
                        ClacPositionLoss(constraintData, jointTransform, jointLoss);
                    }
                    if (constraintData->directionConstraint.isVaild)
                    {
                        RigidTransform parentTransform = parallelRelationData->parentJointIndex == -1 ? RigidTransform.identity : jointTransformNatives[parallelRelationData->parentJointIndex];

                        ClacDirectionLoss(constraintData, jointTransform, &parentTransform, jointLoss);
                    }
                    if (constraintData->positionChangeConstraint.isVaild)
                    {
                        ClacPositionChangeLoss(constraintData, jointTransform, jointLoss);
                    }
                    if (constraintData->DofChangeConstraint.isVaild)
                    {
                        ClacMuscleChangeLoss(constraintData, Dof3, jointLoss);
                    }
                }
                #endregion

                #region Clac Gradient 

                for (int index = settingData->jointLength; index < settingData->parallelLength; index++)
                {
                    ParallelRelationData* parallelRelationData = parallelRelationDatas + index;
                    ConstraintData* constraintData = constraintDatas + index;
                    double* jointLoss = jointLosses + index;
                    float3* Dof3 = Dof3s + parallelRelationData->jointIndex;

                    RigidTransform jointTransform = jointTransformNatives[parallelRelationData->jointIndex];

                    #region  ReBuildTransform

                    RigidTransform* relatedTransform = jointTransformNatives + parallelRelationData->relatedJointIndex;
                    quaternion* muscleGradientRotation = muscleGradientRotations + parallelRelationData->relatedMuscleIndex;

                    if (parallelRelationData->relatedJointIndex != parallelRelationData->jointIndex)
                    {
                        RebuildTransform(relatedTransform, &jointTransform, muscleGradientRotation);

                    }

                    #endregion

                    if ( constraintData->positionConstraint.isVaild)
                    {
                        ClacPositionLoss(constraintData, &jointTransform, jointLoss);
                    }
                    if (constraintData->directionConstraint.isVaild)
                    {
                        RigidTransform parentTransform = parallelRelationData->parentJointIndex == -1 ? RigidTransform.identity : jointTransformNatives[parallelRelationData->parentJointIndex];
                        if (parallelRelationData->parentJointIndex >= parallelRelationData->relatedJointIndex)
                        {
                            RebuildTransform(relatedTransform, &parentTransform, muscleGradientRotation);
                        }

                        ClacDirectionLoss(constraintData, &jointTransform, &parentTransform, jointLoss);
                    }
                    if (constraintData->positionChangeConstraint.isVaild)
                    {
                        ClacPositionChangeLoss(constraintData, &jointTransform, jointLoss);
                    }
                    if (constraintData->DofChangeConstraint.isVaild)
                    {
                        ClacMuscleChangeLoss(constraintData, Dof3, jointLoss);
                    }
                }
                #endregion

                #region Collect Loss 

                ClearNativeArrayData<double>(gradients, settingData->muscleLength);
                for (int i = settingData->jointLength; i < settingData->parallelLength; i++)
                {
                    ParallelRelationData* relationData = parallelRelationDatas + i;
                    double gradientTemp = jointLosses[i] - jointLosses[relationData->jointIndex];

                    gradientTemp /= L_BFGSStatic.EPSILION;
                    gradientTemp *= settingData->loopConvergence;
                    gradients[relationData->relatedMuscleIndex] += gradientTemp;
                }
                double loss = 0;
                for (int i = 0; i < settingData->jointLength; i++)
                {
                    loss += jointLosses[i] / settingData->constraintLength;

                    int3* jointMuscleIndex = jointMuscleIndexs + i;
/*                    int constraintCount = jointConstraintRelativeCounts[i];
                    for (int ii = 0; ii < 3; ii++)
                    {
                        if ((*jointMuscleIndex)[ii] != -1)
                        {
                            double* gradient = gradients + (*jointMuscleIndex)[ii];
                            *gradient  /= constraintCount;
                        }
                    }*/
                }

                #endregion

                #region LBFGS-Optimize
                // CollectTestData(globalData);
                LBFGSSolver->OptimizeUnsafe(leastLoopCount, ref loss, dataStore, muscleValues, gradients);
                currentGroupLoss->loss = loss;

                #endregion
                //OYM：for test,must be removed in environments other than develop
                lossesRecorder[loopIndex + settingData->outsideLoopIndex * settingData->insideLoopCount] = loss;

            }

            private void ClacPositionChangeLoss(ConstraintData* constraintData, RigidTransform* jointTransform, double* jointLoss)
            {
                return;
                //OYM：还没有思考出震颤的问题怎么解决
                float3 jointPosition = jointTransform->pos;
                float lengthSum = constraintData->lengthSum;

                float3 weight3 = constraintData->positionChangeConstraint.weight3;
                float3 constraintPosition = constraintData->positionChangeConstraint.oldPosition;
                float3 direction = (constraintPosition - jointPosition);

                if (math.any(direction != 0))
                {
                    double lossCos = math.csum(direction * direction * weight3);
                    lossCos *= 0.5f;
                    lossCos /= (lengthSum * lengthSum);
                    //lossCos *= math.PI* math.PI;
                    *jointLoss += lossCos;
                }
            }

            private void ClacMuscleChangeLoss(ConstraintData* constraintData, float3* dof3, double* jointLoss)
            {
                float3 oldDof3 = constraintData->DofChangeConstraint.oldDof3;
                float3 tolerance3 = constraintData->DofChangeConstraint.tolerance3;
                float3 weight3 = constraintData->DofChangeConstraint.weight3;

                float3 Dof3Change = math.abs(*dof3 - oldDof3);
                Dof3Change = math.max(0, Dof3Change - tolerance3) * weight3;
                double loss = math.csum(Dof3Change);
                *jointLoss += loss * loss;
            }

            private static void RebuildTransform(RigidTransform* relatedTransform, RigidTransform* jointTransform, quaternion* muscleGradientRotation)
            {
                jointTransform->pos = math.mul(*muscleGradientRotation, jointTransform->pos - relatedTransform->pos) + relatedTransform->pos;
                jointTransform->rot = math.mul(*muscleGradientRotation, jointTransform->rot);
            }

            private static void ClacDirectionLoss(ConstraintData* constraintData, RigidTransform* jointTransform, RigidTransform* parentTransform, double* jointLoss)
            {
                float3 targetDirection = constraintData->directionConstraint.direction;
                float weight = constraintData->directionConstraint.weight;
                float tolerance = constraintData->directionConstraint.tolerance;
                float lengthSum = constraintData->lengthSum;

                float3 jointPosition = jointTransform->pos;
                quaternion jointRotation = jointTransform->rot;
                float3 parentPosition = parentTransform->pos;


                float3 originDirection = jointPosition - parentPosition;

                float length =math.length(originDirection);
                float3 targetPosition = targetDirection * length+ parentPosition;

                float3 direction = (targetPosition - jointPosition);

                if (math.any(direction != 0))
                {
                    double lossCos = math.csum(direction * direction * weight);
                    lossCos /= (lengthSum * lengthSum);
                    //lossCos *= math.PI * math.PI;
                    *jointLoss += lossCos;
                }

            }

            private static void ClacPositionLoss(ConstraintData* constraintData, RigidTransform* jointTransform, double* jointLoss)
            {
                float3 jointPosition = jointTransform->pos;
                float lengthSum = constraintData->lengthSum;

                float3 weight3 = constraintData->positionConstraint.weight3;
                float3 constraintPosition = constraintData->positionConstraint.position;
                float3 direction = (constraintPosition - jointPosition);

                if (math.any(direction != 0))
                {
                    double lossCos = math.csum(direction * direction * weight3);
                    lossCos /= (lengthSum * lengthSum);
                    //lossCos *= math.PI* math.PI;
                    *jointLoss+= lossCos;
                }
            }
        }
    }
}

#region note
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


/*                float3 Dof3toRadian = math.radians(
math.lerp(0, currentJoint->minRange, -math.clamp(*Dof3, -1, 0))
+ math.lerp(0, currentJoint->maxRange, math.clamp(*Dof3, 0, 1))
);
*/
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
/*            [ReadOnly, NativeDisableUnsafePtrRestriction]
            /// <summary>
            /// containing muscles value ,containing joint index and dof index.
            /// </summary>
            public JointData* jointDatas;*/
/*                JointMusclesData* muscleData = muscleDatas+index;
                JointData* currentJoint = jointDatas+muscleData->jointIndex;

                float3 Dof3 = Dof3s[muscleData->jointIndex];

                Dof3[muscleData->dof] +=L_BFGSStatic.EPSILION;
*/


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
/*/// <summary>

/// </summary>
[BurstCompile]
public struct ClacDof3EpsilionJob : IJobParallelFor
{
    [NativeDisableUnsafePtrRestriction, ReadOnly]
    /// <summary>
    /// joint relative muscle's index
    /// </summary>
    internal int3* jointMuscleIndexs;

    [ReadOnly, NativeDisableUnsafePtrRestriction]
    /// <summary>
    /// joint muscles value's rotation 
    /// </summary>
    public quaternion* muscleCurrentRotation;


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

        int3* jointMuscleIndex = jointMuscleIndexs + index;
        quaternion* before = muscleCurrentRotation + index;
        RigidTransform* relationTransform = jointTransformNatives + index;
        for (int i = 0; i < 3; i++)
        {
            if ((*jointMuscleIndex)[i] != -1)
            {
                quaternion* after = muscleGradientRotations + (*jointMuscleIndex)[i];
                *after = math.mul(relationTransform->rot, math.mul(math.mul(*after, math.inverse(*before)), math.inverse(relationTransform->rot)));
            }
        }
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
    public quaternion* muscleCurrentRotation;

    public float3 rootPosition;
    public quaternion rootRotation;

    public void Execute(int index)
    {
        JointData* currentJoint = jointDatas + index;
        RigidTransform* currentTransform = jointTransformNatives + index;
        quaternion* dof3Quaternion = muscleCurrentRotation + index;


        if (currentJoint->parentIndex == -1)
        {
            currentTransform->pos = rootPosition;
            currentTransform->rot = math.mul(rootRotation, *dof3Quaternion);
        }
        else
        {
            RigidTransform* parentTransform = jointTransformNatives + currentJoint->parentIndex;

            currentTransform->pos = parentTransform->pos + math.mul(parentTransform->rot, currentJoint->localPosition);
            currentTransform->rot = math.mul(parentTransform->rot, math.mul(currentJoint->localRotation, *dof3Quaternion));

        }
    }
}*/
/*
/// <summary>
/// Get target joint's loss value 
/// every joint will containing some constriant data,
/// this step will get the constraint loss from the constraint 's constrain
/// </summary>
[BurstCompile]
public struct CaclulatelossJob : IJobParallelFor
{
    [ReadOnly, NativeDisableUnsafePtrRestriction]
    /// <summary>
    /// Constriant data 
    /// </summary>
    public ConstraintData* constraintDatas;

    [ReadOnly, NativeDisableUnsafePtrRestriction]
    /// <summary>
    /// Joint relation data ,for claculate the new world transf
    /// </summary>
    public parallelRelationData* parallelRelationDatas;

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
        parallelRelationData* parallelRelationData = parallelRelationDatas + index;
        ConstraintData* constraintNative = constraintDatas + index;
        double* jointloss = jointlossNatives + index;


        RigidTransform jointTransform = jointTransformNatives[parallelRelationData->jointIndex];
        RigidTransform* pJointTransform = &jointTransform;

        if (parallelRelationData->relatedJointIndex != -1)
        {
            RigidTransform* relatedTransform = jointTransformNatives + parallelRelationData->relatedJointIndex;
            quaternion* muscleGradientRotation = muscleGradientRotations + parallelRelationData->relatedMuscleIndex;
            float3 direction = pJointTransform->pos - relatedTransform->pos;
            pJointTransform->pos = math.mul(*muscleGradientRotation, direction) + relatedTransform->pos;
            pJointTransform->rot = math.mul(*muscleGradientRotation, pJointTransform->rot);
        }
        float3* Dof3 = Dof3s + parallelRelationData->jointIndex;

        *jointloss = 0;
        if (constraintNative->positionConstraint.isVaild)
        {
            float3 jointPosition = pJointTransform->pos;
            float lengthSum = constraintNative->lengthSum;

            float3 weight3 = constraintNative->positionConstraint.weight3;
            float3 constraintPosition = constraintNative->positionConstraint.position;
            double3 direction = constraintPosition - jointPosition;

            if (math.all(direction == 0))
            {
                return;
            }

            double lossCos = math.csum(direction * direction * weight3);
            lossCos /= (lengthSum * lengthSum);
            lossCos *= math.PI * math.PI;
            *jointloss += lossCos;
        }
                        if (constraintNative.directionConstraint.isVaild)
                        {
                            UpdatePositionloss(ref jointloss, ref jointTransform, ref constraintNative);
                        }

        if (constraintNative.DofConstraint.isVaild)
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
        if (constraintNative.DofChangeConstraint.isVaild)
        {
            UpdateMuscleChangeloss(ref jointloss, ref Dof3, ref constraintNative);
        }
                    }
                    [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void RebuildJointTransform(RigidTransform* jointTransform, RigidTransform* relatedTransform, quaternion* muscleGradientRotation)
        {
            float3 direction = jointTransform->pos - relatedTransform->pos;
            jointTransform->pos = math.mul(*muscleGradientRotation, direction) + relatedTransform->pos;
            jointTransform->rot = math.mul(*muscleGradientRotation, jointTransform->rot);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void UpdatePositionloss(double* jointloss, RigidTransform* jointTransform, ConstraintData* constraintNative)
        {
            float3 jointPosition = jointTransform->pos;
            float lengthSum = constraintNative->lengthSum;

            float3 weight3 = constraintNative->positionConstraint.weight3;
            float3 constraintPosition = constraintNative->positionConstraint.position;
            double3 direction = constraintPosition - jointPosition;

            if (math.all(direction == 0))
            {
                return;
            }

            double lossCos = math.csum(direction * direction * weight3);
            lossCos /= (lengthSum * lengthSum);
            lossCos *= math.PI * math.PI;
            *jointloss += lossCos;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void UpdateMuscleloss(ref JoinLoss jointloss, ref float3 Dof3, ref ConstraintData constraintNative)
        {
            float3 tolerance3 = constraintNative.DofConstraint.tolerance3;
            float3 weight3 = constraintNative.DofConstraint.weight3;
            float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3, 0);
            float loss = math.csum(Dof3Outside * weight3);
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
            jointloss.lossSum += loss * loss * weight;
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
                           direction = direction / torlerace3;

                            float directionLength = math.length(direction);

                            float newDirectionLength = math.max(0, directionLength - 1);

                            direction = (direction / directionLength * newDirectionLength) * torlerace3;

        float loss = math.csum(direction * direction * weight3);

    //loss =;

    jointloss.lossSum += loss;
}
[MethodImpl(MethodImplOptions.AggressiveInlining)]
private static void UpdateMuscleChangeloss(ref JoinLoss jointloss, ref float3 Dof3, ref ConstraintData constraintNative)
{
    float3 oldDof3 = constraintNative.DofChangeConstraint.oldDof3;
    float3 tolerance3 = constraintNative.DofChangeConstraint.tolerance3;
    float3 weight3 = constraintNative.DofChangeConstraint.weight3;

    float3 Dof3Change = math.abs(Dof3 - oldDof3);
    Dof3Change = math.max(0, Dof3Change - tolerance3) * weight3;
    float loss = math.csum(Dof3Change) / 3;
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
                [NativeDisableParallelForRestriction]
                internal double* gradientAlls;
                [NativeDisableParallelForRestriction]
                internal float* muscleAlls;

    [ReadOnly, NativeDisableUnsafePtrRestriction]
    /// <summary>
    /// joint loss
    /// </summary>
    public double* jointlossNatives;

    [NativeDisableUnsafePtrRestriction, ReadOnly]
    /// <summary>
    /// relative data
    /// </summary>
    public parallelRelationData* relationDatas;

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

        /*                double loss = 0;
                        for (int i = 0; i < jointLength; i++)
                        {
                            loss += jointlossNatives[i] / constraintLength;
                        }
                        ClearNativeArrayData<double>(gradients, muscleLength);
                        for (int i = jointLength; i < parallelLength; i++)
                        {
                            parallelRelationData* relationData = relationDatas + i;
                            double gradientTemp = losses[i] - losses[relationData->jointIndex];

                            gradientTemp /= L_BFGSStatic.EPSILION;
                            gradients[relationData->relatedMuscleIndex] += gradientTemp; ;
                        }
                        // CollectTestData(globalData);
                        LBFGSSolver->Optimize(loss,  ref globalData->leastLoopCount, dataStore, muscleValues, gradients);
                        losses[globalData->leastLoopCount] = LBFGSSolver->loss;
    }

          private void CollectTestData(GlobalData globalData)
                {
                    UnsafeUtility.MemCpy((double*)gradientAlls.GetUnsafePtr() + globalData.leastLoopCount * muscleLength, gradients.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<double>());
                    UnsafeUtility.MemCpy((float*)muscleAlls.GetUnsafePtr() + globalData.leastLoopCount * muscleLength, muscleValues.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<float>());
                }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Collectloss(double* losses, int offset, int constraintlength, int jointLength)
    {
        double loss = 0;
        if (constraintlength != 0)
        {
            for (int i = 0; i < jointLength; i++)
            {
                loss += losses[i + offset] / constraintlength;
            }
        }
        return loss;
    }

    //Note: The current gradient function is
    //the one that I have tried many times and concluded that it works best.
    //If you are confident, you can write a better one
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void CollectGradient(double* losses, parallelRelationData* relationDatas, int muscleLength, int jointLength, int parallelLength, double* gradients)
    {
        ClearNativeArrayData<double>(gradients, muscleLength);
        for (int i = jointLength; i < parallelLength; i++)
        {
            parallelRelationData* relationData = relationDatas + i;
            double gradientTemp = losses[i] - losses[relationData->jointIndex];

            gradientTemp /= L_BFGSStatic.EPSILION;
            gradients[relationData->relatedMuscleIndex] += gradientTemp; ;
        }

    }
}
*/
#endregion
#region note2
/*                for (int i = settingData->jointLength; i < settingData->parallelLength; i++)
                {
                    ParallelRelationData* relationData = parallelRelationDatas + i;
                    double gradientTemp = jointlosses[i] - jointlosses[relationData->jointIndex];

                    gradientTemp /= L_BFGSStatic.EPSILION;
                    gradients[relationData->relatedMuscleIndex] += gradientTemp; 
                }

                double loss = 0;
                for (int i = 0; i < settingData->jointLength; i++)
                {
                    loss += jointlosses[i].positionloss / settingData->constraintLength;

                    int3* jointMuscleIndex = jointMuscleIndexs + i;
                    int constraintCount = jointConstraintRelativeCounts[i];
                    for (int ii = 0; ii < 3; ii++)
                    {
                        if ((*jointMuscleIndex)[ii] != -1)
                        {
                            double* gradient = gradients + (*jointMuscleIndex)[ii];
                            *gradient  /= constraintCount;
                            *gradient *= settingData->loopConvergence;
                        }
                    }
                }
*/
/*                double3 direction = constraintData->directionConstraint.direction;
                float weight = constraintData->directionConstraint.weight;
                float tolerance = constraintData->directionConstraint.tolerance;

                float3 jointPosition = jointTransform->pos;
                quaternion jointRotation = jointTransform->rot;
                float3 parentPosition = parentTransform->pos;


                double3 targetDirection = jointPosition - parentPosition;
                if (math.any(targetDirection != 0))
                {
                    targetDirection=math.normalize(targetDirection);
                    double cosA = math.dot(direction, targetDirection);
                    cosA = math.clamp(cosA, -1, 1);

                    double loss = 2 * (1 - cosA);
                    //loss = math.max(0, math.abs(loss) - tolerance * math.PI);
                    loss = loss * loss * weight;
                   // loss /=math.pow( constraintData->lengthSum ,2);

                    * jointLoss += loss * loss * weight;
                }*/
/*                    for (int i = 0; i < setting.muscleLength; i++)
                    {
                        float currentPoint = oldMuslceValues[i];
                        for (int iii = 1; iii < setting.groupLength; iii++)
                        {
                            int offset = iii * setting.muscleLength;
                            currentPoint += musclesValues[i + offset];
                        }
                        currentPoint /= setting.groupLength;
                        for (int ii = 0; ii < setting.groupLength; ii++)
                        {
                            int offset = ii * setting.muscleLength;
                            float lastMuscle = oldMuslceValues[i + offset];
                            musclesValues[i+ offset] = math.lerp(currentPoint, lastMuscle, 0.9f);
                        }
                    }*/
#endregion