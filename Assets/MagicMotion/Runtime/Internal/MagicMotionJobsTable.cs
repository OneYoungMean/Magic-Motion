using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;
using System;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;
using System.Runtime.CompilerServices;

namespace MagicMotion
{
    internal static unsafe class MagicMotionJobsTable
    {
        /// <summary>
        ///  Initialize Muscle Value
        /// </summary>
        public struct InitializeMuscleJob : IJobParallelFor
        {
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MuscleData> muscleDatas;
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<float> musclesValues;

            public void Execute(int index)
            {
                musclesValues[index] = math.clamp(musclesValues[index],0,0);
            }
        }
        /// <summary>
        /// Set joint Transform data before update
        /// </summary>
        public struct InitializeJointJob : IJobParallelForTransform
        {
            /// <summary>
            /// Joint's world transform data
            /// </summary>
            public NativeArray<RigidTransform> jointTransformDatas;

            public int muscleLength;
            public int jointLength;

            public void Execute(int index, TransformAccess transform)
            {
                var rigid = jointTransformDatas[ index];
                rigid.pos = transform.position;
                rigid.rot = transform.rotation;
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
            public NativeArray<ConstraintData> constraintDatas;
            /// <summary>
            /// TransformToConstrain Data 
            /// </summary>
            public NativeArray<TransformToConstraintData> transformToConstrainDatas;
            /// <summary>
            ///  constraint positions
            /// </summary>
            public NativeArray<Vector3> constraintPositions;
            /// <summary>
            /// TransformToConstrain Data 
            /// </summary>
            public NativeArray<Quaternion> constraintRotations;
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
        /// <summary>
        /// Set Constriant data 
        /// </summary>
        [BurstCompile]
        public struct BuildConstraintDataJob : IJobParallelFor
        {
            /// <summary>
            /// Joint relation data 
            /// </summary>
            [ReadOnly]
            public NativeArray<JointRelationData> jointRelationDatas;
            /// <summary>
            /// Joint world position
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<Vector3> jointPositions;
            /// <summary>
            /// Joint world rotation
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<Quaternion> jointRotations;
            /// <summary>
            /// joint dof3 value 
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            /// <summary>
            /// constraint data 
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<ConstraintData> constraintDatas;
            public void Execute(int index)
            {
                var jointRelation = jointRelationDatas[index];
                var originConstraint = constraintDatas[jointRelation.jointIndex];
                var jointPosition = jointPositions[jointRelation.jointIndex];
                var jointRotation = jointRotations[jointRelation.jointIndex];
                var jointDof3 = Dof3s[jointRelation.jointIndex];

                originConstraint.positionChangeConstraint.oldPosition = jointPosition;
                originConstraint.DofChangeConstraint.oldDof3= jointDof3;
                constraintDatas[index] = originConstraint;
            }
        }
        /// <summary>
        ///  muscles to joint dof value 
        /// </summary>
        [BurstCompile]
        public struct MuscleToDof3Job : IJobParallelFor
        {
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MuscleData> muscleDatas;

            [NativeDisableParallelForRestriction]
            /// <summary>
            /// joint muscles value 
            /// </summary>
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
        /// <summary>
        ///  Convert dof3 to rotation value 
        /// </summary>
        [BurstCompile]
        
        public struct Dof3ToRotationJob : IJobParallelFor
        {
            /// <summary>
            /// Dof3 value 
            /// </summary>
            [ReadOnly]
            public NativeArray<float3> Dof3s;
            /// <summary>
            /// Dof3 to rotation value
            /// </summary>
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

                Dof3Quaternions[index] = eulerAngle;
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
            [ReadOnly]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MuscleData> muscleDatas;
            /// <summary>
            /// joint muscles value 
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            /// <summary>
            /// joint muscles value's rotation 
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<quaternion> Dof3Quaternions;
            /// <summary>
            /// joint data 
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<JointData> jointDatas;
            /// <summary>
            /// joint transform value 
            /// </summary>
            [ReadOnly,NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            /// <summary>
            /// joint transform value 
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<GlobalData> globalDatas;
            /// <summary>
            ///Epsilion to rotation
            /// </summary>
            public NativeArray<quaternion> muscleGradientRotations;
            public void Execute(int index)
            {
                var muscleData = muscleDatas[index];
                var currentJoint = jointDatas[muscleData.jointIndex];

                float3 Dof3 = Dof3s[muscleData.jointIndex];

                Dof3[muscleData.dof] +=L_BFGSStatic.EPSILION;
                quaternion before = Dof3Quaternions[muscleData.jointIndex];
                quaternion after = quaternion.identity;
                float3 Dof3toRadian = math.radians(
                    math.lerp(0, currentJoint.minRange, -math.clamp(Dof3, -1, 0)) +
                    math.lerp(0, currentJoint.maxRange, math.clamp(Dof3, 0, 1)));
                if (Dof3[0]!=0)
                {
                    after = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[0], Dof3toRadian[0]), after);
                }
                if (Dof3[1] != 0)
                {
                    after = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[1], Dof3toRadian[1]), after);
                }
                if (Dof3[2] != 0)
                {
                    after = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[2], Dof3toRadian[2]), after);
                }


                quaternion worldRot = jointTransformNatives[muscleData.jointIndex].rot;
                quaternion change = math.mul(worldRot, math.mul(math.mul(after, math.inverse(before)), math.inverse(worldRot)));
                muscleGradientRotations[index] = change;
            }
        }
        /// <summary>
        /// Build Joint world transform for muscle value and local transform
        /// </summary>
        [BurstCompile]
        public struct BuildTransformJob : IJobFor
        {
            /// <summary>
            /// joint world transform
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            /// <summary>
            /// joint datas 
            /// </summary>
            [ReadOnly]
            public NativeSlice<JointData> jointDatas;
            /// <summary>
            /// joint muscle value's rotation 
            /// </summary>
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
        /// <summary>
        /// Get target joint's loss value 
        /// every joint will containing some constriant data,
        /// this step will get the constraint loss from the constraint 's constrain
        /// </summary>
        [BurstCompile]
        public struct CaclulatelossJob : IJobParallelFor
        {
            /// <summary>
            /// Constriant data 
            /// </summary>
            [ReadOnly]
            public NativeArray<ConstraintData> constraintNatives;
            /// <summary>
            /// Joint relation data ,for claculate the new world transf
            /// </summary>
            [ReadOnly]
            public NativeArray<JointRelationData> jointRelationDatas;
            /// <summary>
            /// joint world transform
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<RigidTransform> jointTransformNatives;
            /// <summary>
            /// joint muscles value 
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<float3> Dof3s;
            /// <summary>
            /// Gradient's rotation
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<quaternion> muscleGradientRotations;
            /// <summary>
            /// Loss data 
            /// </summary>
            public NativeArray<JoinLoss> jointlossNatives;

            public void Execute(int index)
            {

                JointRelationData jointRelationData = jointRelationDatas[index];    
                ConstraintData constraintNative = constraintNatives[index];
                JoinLoss jointloss = jointlossNatives[index];


                RigidTransform jointTransform = jointTransformNatives[jointRelationData.jointIndex];
                if (jointRelationData.relatedJointIndex!=-1)
                {
                    RigidTransform relatedTransform = jointTransformNatives[jointRelationData.relatedJointIndex];
                    quaternion muscleGradientRotation = muscleGradientRotations[jointRelationData.relatedMuscleIndex];
                    RebuildJointTransform(ref jointTransform, ref relatedTransform, ref muscleGradientRotation);
                }

                float3 Dof3 = Dof3s[jointRelationData.jointIndex];

                if (constraintNative.positionConstraint.isVaild)
                {
                    UpdatePositionloss(ref jointloss, ref jointTransform, ref constraintNative);
                }
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
                jointloss.Clacloss();
               jointlossNatives[index] = jointloss;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void RebuildJointTransform(ref RigidTransform jointTransform, ref RigidTransform relatedTransform, ref quaternion muscleGradientRotation)
            {
                float3 direction = jointTransform.pos - relatedTransform.pos;
                jointTransform.pos = math.mul(muscleGradientRotation,direction)+ relatedTransform.pos;
                jointTransform.rot = math.mul( muscleGradientRotation, jointTransform.rot);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdatePositionloss(ref JoinLoss jointloss, ref RigidTransform jointTransform, ref ConstraintData constraintNative)
            {
                PositionConstraint positionConstraint = constraintNative.positionConstraint;
                float3 jointPosition = jointTransform.pos;
                float lengthSum = constraintNative.lengthSum;

                float3 constraintPosition = positionConstraint.position;
                float3 direction = constraintPosition - jointPosition;
                float lossCos = math.lengthsq(direction);
                lossCos /= ( lengthSum* lengthSum);
                lossCos *= math.PI * math.PI;
                jointloss.positionloss = lossCos;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateMuscleloss(ref JoinLoss jointloss, ref float3 Dof3, ref ConstraintData constraintNative)
            {
                float3 tolerance3 = constraintNative.DofConstraint.tolerance3;
                float3 weight3 = constraintNative.DofConstraint.weight3;
                float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3,0);
                float loss =math.csum(Dof3Outside * weight3);
                jointloss.muscleloss = loss;
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
            jointloss.lookAtloss= loss* loss*weight;
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

                jointloss.positionChangeloss = loss*10;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void UpdateMuscleChangeloss(ref JoinLoss jointloss, ref float3 Dof3, ref ConstraintData constraintNative)
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

            #region LBFGS Data
            /// <summary>
            /// BFGS 求解应用到的变量
            /// </summary>
            public NativeArray<LBFGSSolver> LBFGSSolvers;
            /// <summary>
            /// global data ,to control loop's work
            /// </summary>
            internal NativeArray<GlobalData> globalDataNative;
            /// <summary>
            /// Gradient 
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<double> gradients;
            [NativeDisableParallelForRestriction]
            public NativeArray<double> diagonal;
            [NativeDisableParallelForRestriction]
            public NativeArray<double> gradientStore;
            [NativeDisableParallelForRestriction]
            public NativeArray<double> rho;
            [NativeDisableParallelForRestriction]
            public NativeArray<double> alpha;
            [NativeDisableParallelForRestriction]
            public NativeArray<double> steps;
            [NativeDisableParallelForRestriction]
            public NativeArray<double> delta;
            [NativeDisableParallelForRestriction]
            internal NativeArray<double> losses;
/*            [NativeDisableParallelForRestriction]
            internal NativeArray<double> gradientAlls;
            [NativeDisableParallelForRestriction]
            internal NativeArray<float> muscleAlls;*/
            #endregion

            /// <summary>
            /// joint loss
            /// </summary>
            [ReadOnly, NativeDisableParallelForRestriction]
            public NativeArray<JoinLoss> jointlossNatives;
            /// <summary>
            /// relative data
            /// </summary>
            [NativeDisableParallelForRestriction, ReadOnly]
            public NativeArray<JointRelationData> relationDatas;
            /// <summary>
            /// muscle relative constraint count;
            /// </summary>
            [NativeDisableParallelForRestriction, ReadOnly]
            public NativeArray<int> jointRelativedCounts;
            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            [NativeDisableParallelForRestriction]
            public NativeArray<float> muscleValues;
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
                int index = 0;
                var globalData = globalDataNative[index];

                 double loss = Collectloss(jointlossNatives, 0, jointRelativedCounts[0], jointLength);
                CollectGradient(jointlossNatives, relationDatas, globalData, jointRelativedCounts, muscleLength, jointLength, parallelLength, gradients);
               // CollectTestData(globalData);
                losses[globalData.leastLoopCount] = loss;
                var LBFGSSolver = LBFGSSolvers[index];
                LBFGSSolver.Optimize(loss, ref globalData.leastLoopCount, diagonal, gradientStore, rho, alpha, steps, delta, muscleValues, gradients);
                LBFGSSolvers[index] = LBFGSSolver;
                globalDataNative[index] = globalData;
            }

/*            private void CollectTestData(GlobalData globalData)
            {
                UnsafeUtility.MemCpy((double*)gradientAlls.GetUnsafePtr() + globalData.leastLoopCount * muscleLength, gradients.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<double>());
                UnsafeUtility.MemCpy((float*)muscleAlls.GetUnsafePtr() + globalData.leastLoopCount * muscleLength, muscleValues.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<float>());
            }*/

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static double Collectloss(NativeArray<JoinLoss> losses, int offset, int constraintlength,int jointLength)
            {
                double loss = 0;
                if (constraintlength!=0)
                {
                    for (int i = 0; i < jointLength; i++)
                    {
                        loss +=losses[i + offset].lossSum/ constraintlength;
                    }
                }

                return loss;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void CollectGradient(NativeArray<JoinLoss> losses,NativeArray<JointRelationData> relationDatas ,GlobalData globalData, NativeArray<int> jointRelativedCounts, int muscleLength, int jointLength, int parallelLength,NativeArray<double>gradients)
            {
                UnsafeUtility.MemClear(gradients.GetUnsafePtr(), gradients.Length * UnsafeUtility.SizeOf<double>());
                for (int i = jointLength; i < parallelLength; i++)
                {
                    JointRelationData relationData = relationDatas[i];
                    double gradientTemp = (double)losses[i].lossSum - (double)losses[relationData.jointIndex].lossSum;
                    gradientTemp /= jointRelativedCounts[relationData.jointIndex];
                    gradientTemp /= L_BFGSStatic.EPSILION;
                    

                    gradients[relationData.relatedMuscleIndex] += gradientTemp;
                    //gradients[relationData.relatedMuscleIndex] += gradientTemp;
                }

            }
        }

        public struct JointToTransformJob : IJobParallelForTransform
        {

            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            public void Execute(int index, TransformAccess transform)
            {
                transform.position = jointTransformNatives[index].pos;
                transform.rotation=jointTransformNatives[index].rot;
            }
        }
    }
}
//待考证
/*
public struct RigHipPositionJob : IJobParallelFor
{
    [ReadOnly,NativeDisableParallelForRestriction]
    public NativeArray<JointData> jointDatas;
    [ReadOnly, NativeDisableParallelForRestriction]
    public NativeArray<ConstraintData> constraintNatives;
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