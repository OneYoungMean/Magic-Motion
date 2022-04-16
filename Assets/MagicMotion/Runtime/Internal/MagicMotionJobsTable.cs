using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;
using System;

namespace MagicMotion
{
    internal static class MagicMotionJobsTable
    {
        //OYM：确定hips的位置

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

                    for (int i = 0; i < constraintNatives.Length; i++)
                    {
                        MMConstraintNative jointConstraint = constraintNatives[offset+i];

                        MMPositionConstraint positionConstraint = jointConstraint.positionConstraint;

                        if (positionConstraint.isVaild)//OYM：据说这个人畜无害的小判断会破坏向量化，但是俺寻思这么一点计算量也看不出来
                        {
                            MMJointNative joint = jointNatives[offset+i];

                            float3 hipsDirection = hipJointPosition - positionConstraint.position;

                            float hipsDirectionDistance = math.length(hipsDirection);

                            float restDistance = positionConstraint.cumulativeLength;

                            float forceLength = math.max(hipsDirectionDistance - restDistance, 0);

                            float3 force = (hipsDirection / hipsDirectionDistance) * forceLength * positionConstraint.weight3;

                            deltaPosition += force;
                        }
                    }
                    hipJointPosition += deltaPosition / constraintNatives.Length;
                }

                float3 alldeltaPosition = hipJointPosition - jointTransformNatives[offset].pos;
                for (int i = 0; i < jointTransformNatives.Length; i++)
                {
                    var rigid = jointTransformNatives[offset+i];
                    rigid.pos += alldeltaPosition;
                    jointTransformNatives[offset+i] = rigid;
                }
            }
        }
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
        public struct MuscleToJointJob : IJobParallelFor
        {
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
                var musclesNative = musclesNatives[index];
                var muscleValue = musclesValues[index];

                for (int i = 0; i < muscleCount; i++)//OYM：第0行留给fitness，1~muscleCount留给gradient
                {
                    float3 targetDof3 = Dof3s[musclesNative.jointIndex+i* jointCount];
                    if (i +1== index )
                    {
                        targetDof3[musclesNative.dof] = musclesNative.dof + L_BFGSStatic. EPSILION;//OYM：计算gradient的值
                    }
                    else
                    {
                        targetDof3[musclesNative.muscleIndex] = musclesNative.dof;
                    }
                    Dof3s[musclesNative.jointIndex + i * jointCount] = targetDof3;
                }
            }
        }

        public struct BuildTransformJob : IJobParallelFor
        {
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
                int offset = index * jointLength;
                for (int i = 0; i < jointLength; i++)
                {
                    int point = offset + i;
                MMJointNative currentJoint = jointNatives[point];
                RigidTransform currentTransform = jointTransformNatives[point];
                float3 Dof3 = Dof3s[point];

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
                    jointTransformNatives[point]=currentTransform;
                }
            }
        }

        public struct CaclulateFitnessJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<MMConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            [ReadOnly]
            public NativeArray<float3> Dof3s;
            [WriteOnly]
            public NativeArray<MMJoinFitness> jointFitnessNatives;

            public void Execute(int index)
            {
                MMConstraintNative constraintNative = constraintNatives[index];
                MMJoinFitness jointFitness = jointFitnessNatives[index];
                RigidTransform jointTransform = jointTransformNatives[index];
                float3 Dof3 = Dof3s[index];

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
            private static void UpdatePositionFitness(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMConstraintNative constraintNative)
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

                float fitness =math.csum(direction* direction*weight3);
                fitness *= math.PI * math.PI / (positionConstraint.cumulativeLength * positionConstraint.cumulativeLength);

                jointFitness.positionFitness = fitness;
            }
            private static void UpdateMuscleFitness(ref MMJoinFitness jointFitness, float3 Dof3, MMConstraintNative constraintNative)
            {
                float3 tolerance3 = constraintNative.muscleConstraint.tolerance3;
                float3 weight3 = constraintNative.muscleConstraint.weight3;
                float3 Dof3Outside = math.max(math.abs(Dof3) - tolerance3,0);
                float fitness =math.csum(Dof3Outside * weight3);
                jointFitness.muscleFitness = fitness;
            }
            private static void UpdateLookAtFitness(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMConstraintNative constraintNative)
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
            jointFitness.lookAtFitness= fitness* fitness*weight;
            }
            private static void UpdateColliderConstraint(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMConstraintNative constraintNative)
            {
                //OYM：啊这个超级难写 
                //OYM：还要去构造AABB
                //OYM：不想写（摆烂
            }
            private static void UpdatePositionChangeFitness(ref MMJoinFitness jointFitness, RigidTransform jointTransform, MMConstraintNative constraintNative)
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

                float fitness = math.lengthsq(direction * weight3);
                jointFitness.positionFitness = fitness;
            }

            private static void UpdateMuscleChangeFitness(ref MMJoinFitness jointFitness, float3 Dof3, MMConstraintNative constraintNative)
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

        public struct MainControllerJob : IJobParallelFor
        {
            //OYM：感觉计算量超级的大啊
            //OYM：等我回来在写注释
            //OYM：趁着现在灵感还在
            /// <summary>
            /// joint fitness
            /// </summary>
            [ReadOnly]
            public NativeArray<MMJoinFitness> jointFitnessNatives;
            /// <summary>
            /// the Dof3, is map from muscle value.
            /// </summary>
            [WriteOnly]
            public NativeArray<float3> Dof3s;
/*            /// <summary>
            /// muscles value ,containing joint index and dof index.
            /// </summary>
            public NativeArray<MMMuscleNative> musclesNatives;*/
            /// <summary>
            /// muscle Value 
            /// </summary>
            public NativeArray<float> muscleValueNatives;
            /// <summary>
            /// Gradient 
            /// </summary>
            public NativeArray<float> gradients;

            #region LBFGS Data
            /// <summary>
            /// BFGS 求解应用到的变量
            /// </summary>
            public NativeArray<MMLBFGSNative> LBFGSNatives;

            public NativeArray<float> diagonal;

            public NativeArray<float> gradientStore;

            public NativeArray<float> rho;

            public NativeArray<float> alpha;

            public NativeArray<float> steps;

            public NativeArray<float> delta;


            #endregion
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
            public float fitness;



            public void Execute(int index)
            {
                PreOptimizeProcess();
                LBFGSNatives[index].Optimize(diagonal,gradientStore,rho,alpha,steps,delta, muscleValueNatives, gradients);
                PostOptimizeProcess();
            }

            private void PostOptimizeProcess()
            {
            }

            private void PreOptimizeProcess()
            {
                CollectFitnessAndGradient();
            }

            private void CollectFitnessAndGradient()
            {
                fitness = CollectFitness(jointFitnessNatives, 0, jointLength);


                for (int i = 0; i <  muscleLength; i++)
                {
                    int offset = jointLength + jointLength * i;
                    float fitnessTemp = CollectFitness(jointFitnessNatives, offset, jointLength);
                    float gradientTemp = (fitnessTemp- fitness)/ L_BFGSStatic.EPSILION;
                    gradients[i] = gradientTemp;
                }
            }

            private static float CollectFitness(NativeArray<MMJoinFitness> fitnesses, int offset, int jointCount)
            {
                float fitness = 0;
                for (int i = 0; i < jointCount; i++)
                {
                    fitnesses[i+ offset].ClacFitness();
                    fitness += fitnesses[i+offset].fitnessSum;
                }
                fitness = math.sqrt(fitness / jointCount);
                return fitness;
            }

        }

        public struct JointToTransformJob : IJobParallelForTransform
        {
            [ReadOnly]
            public NativeSlice<RigidTransform> jointTransformNatives;
            public void Execute(int index, TransformAccess transform)
            {
                transform.position = jointTransformNatives[index].pos;
                transform.rotation=jointTransformNatives[index].rot;
            }
        }
    }
}
