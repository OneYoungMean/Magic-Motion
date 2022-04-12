using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;
using Unity.Collections;

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
            public NativeArray<MMConstraintNative> positionConstraintNatives;
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
                        MMConstraintNative constraint = positionConstraintNatives[i];

                        if (constraint.targetJointIndex == -1)
                        {
                            continue;
                        }

                        MMJointNative joint = jointNatives[i];

                        float3 hipsDirection = hipJointPosition - constraint.position;

                        float hipsDirectionDistance = math.length(hipsDirection);

                        float restDistance = joint.cumulativeLength;

                        float forceLength = math.max(hipsDirectionDistance - restDistance, 0);

                        float3 force = (hipsDirection / hipsDirectionDistance) * forceLength * constraint.weight3;

                        deltaPosition += force;
                    }
                    hipJointPosition += deltaPosition;
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
        /// <summary>
        /// clac position fitness
        /// </summary>
        public struct ClacFitnessJob_Position : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<MMJointNative> jointNatives;
            [ReadOnly]
            public NativeArray<MMConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            public NativeArray<float> fitnessNative;
            public void Execute(int index)
            {
                float3 jointPosition = jointTransformNatives[index].pos;
                float3 constraintPosition = constraintNatives[index].position;
                float3 torlerace3 = constraintNatives[index].torlerace3;
                float3 weight3 =constraintNatives[index].weight3;

                float3 direction = constraintPosition - jointPosition;

                direction = math.max(math.EPSILON, torlerace3);

                direction = direction / torlerace3;
                float directionLength = math.length(direction) - 1;
                directionLength= math.max(0, directionLength);

                direction = (direction * directionLength) * torlerace3;

                float fitness = math.csum(direction * weight3);
                fitnessNative[index] = fitness;
            }
        }
        /// <summary>
        ///  clac lookat fitness
        /// </summary>
        public struct ClacFitnessJob_LookAt : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<MMJointNative> jointNatives;
            [ReadOnly]
            public NativeArray<MMConstraintNative> constraintNatives;
            [ReadOnly]
            public NativeArray<RigidTransform> jointTransformNatives;
            public NativeArray<float> fitnessNative;
            public void Execute(int index)
            {
                float3 jointPosition = jointTransformNatives[index].pos;
                quaternion jointRotation = jointTransformNatives[index].rot;

                float3 constraintPosition = constraintNatives[index].position;
                float3 torlerace3 = constraintNatives[index].torlerace3;
                float3 weight3 = constraintNatives[index].weight3;

                float3 targetDirection = constraintPosition - jointPosition;
                float3 targetForward = math.mul(jointRotation, new float3(0, 0, 1));//OYM：后续会更改的

                float cosA = math.dot(targetForward, targetDirection) / (math.length(targetDirection));
                cosA = math.clamp(cosA, -1, 1);

                float fitness = math.acos(cosA);
                fitnessNative[index] = fitness;
                /*
                 *             float3 targetForward = math.mul(worldRotation, ViewingDirection);
            float3 targetDirection =   (float3)targetPositon- worldPosition;
            float cosA = math.dot(targetForward, targetDirection) / (math.length(targetForward) * math.length(targetDirection));
            cosA= math.clamp(cosA, - 1, 1);
            return math.acos(cosA);
                 */
            }
        }
    }
}