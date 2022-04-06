using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
#if UNITY_2019_3_OR_NEWER
using UnityEngine.Animations;
#else
using UnityEngine.Experimental.Animations;
#endif

public struct DampingJob : IAnimationJob
{
    public static float3 SmoothDamp(float3 prePosition, float3 targetPosition, ref float3 currentVelocity, float smoothTime,  float maxSpeed, float deltaTime)
    {
        float3 currentPosition = 0;
        /*        float num = 0f;
                float num2 = 0f;
                float num3 = 0f;*/

        smoothTime = Mathf.Max(math.EPSILON, smoothTime);
        deltaTime= Mathf.Max(math.EPSILON, deltaTime);
        float twoDivSmoothTime = 2f / smoothTime;
        float targetDeltaTime = twoDivSmoothTime * deltaTime;
        float timeScale = 1f / (1f + targetDeltaTime + 0.48f * targetDeltaTime * targetDeltaTime + 0.235f * targetDeltaTime * targetDeltaTime * targetDeltaTime);
        /*
         *         smoothTime = Mathf.Max(0.0001f, smoothTime);
                float num4 = 2f / smoothTime;
                float num5 = num4 * deltaTime;
                float num6 = 1f / (1f + num5 + 0.48f * num5 * num5 + 0.235f * num5 * num5 * num5);
         */

        float3 preDirection = prePosition - targetPosition;
        /*        float num7 = current.x - target.x;
        float num8 = current.y - target.y;
        float num9 = current.z - target.z;*/
        float3 preTarget = targetPosition;

        float limitSpeed = maxSpeed * smoothTime;
        float directionLengthSqr = math.lengthsq(preDirection);
        /*        float num10 = maxSpeed * smoothTime;
                float num11 = num10 * num10;
                float num12 = num7 * num7 + num8 * num8 + num9 * num9;*/
        //OYM：reclac directionLength;
        if (directionLengthSqr > limitSpeed* limitSpeed)
        {
            float directionLength = (float)math.sqrt(directionLengthSqr);
            preDirection = preDirection / directionLength * limitSpeed;
        }
        /*        if (num12 > num11)
                {
                    float num13 = (float)Math.Sqrt(num12);
                    num7 = num7 / num13 * num10;
                    num8 = num8 / num13 * num10;
                    num9 = num9 / num13 * num10;
                }
        */
        //OYM：reClac targetPosition Lenght;
        targetPosition = prePosition - preDirection;
        /*        target.x = current.x - num7;
                target.y = current.y - num8;
                target.z = current.z - num9;*/
        //OYM：
        float3 currentMove = (currentVelocity + twoDivSmoothTime * preDirection) * deltaTime;
        /*        float num14 = (currentVelocity.x + halfSmoothTime * num7) * deltaTime;
                float num15 = (currentVelocity.y + halfSmoothTime * num8) * deltaTime;
                float num16 = (currentVelocity.z + halfSmoothTime * num9) * deltaTime;*/

        currentVelocity = (currentVelocity - twoDivSmoothTime * currentMove) * timeScale;
        /*       currentVelocity.x = (currentVelocity.x - halfSmoothTime * num14) * timeScale;
               currentVelocity.y = (currentVelocity.y - halfSmoothTime * num15) * timeScale;
               currentVelocity.z = (currentVelocity.z - halfSmoothTime * num16) * timeScale;*/
        currentPosition = targetPosition + (preDirection + currentMove) * timeScale;
        /*               num = target.x + (num7 + num14) * timeScale;
                num2 = target.y + (num8 + num15) * timeScale;
                num3 = target.z + (num9 + num16) * timeScale;*/
        float3 currentDirection = preTarget - prePosition;
        /*        float num17 = preTarget.x - current.x;
                float num18 = preTarget.y - current.y;
                float num19 = preTarget.z - current.z;*/

        float3 currentDistance = currentPosition - preTarget;
/*        float num20 = num - preTarget.x;
        float num21 = num2 - preTarget.y;
        float num22 = num3 - preTarget.z;*/
        if (math.dot(currentDirection,currentDistance) > 0f)
        {
            currentPosition = preTarget;
            /*            num = preTarget.x;
                        num2 = preTarget.y;
                        num3 = preTarget.z;*/
            currentVelocity = (currentPosition - preTarget) / deltaTime;
/*            currentVelocity.x = (num - preTarget.x) / deltaTime;
            currentVelocity.y = (num2 - preTarget.y) / deltaTime;
            currentVelocity.z = (num3 - preTarget.z) / deltaTime;*/
        }

        return currentPosition;
    }
    public TransformStreamHandle rootHandle;
    public NativeArray<TransformStreamHandle> jointHandles;
    public NativeArray<Vector3> localPositions;
    public NativeArray<Quaternion> localRotations;
    public NativeArray<Vector3> positions;
    public NativeArray<Vector3> velocities;

    /// <summary>
    /// Transfer the root position and rotation through the graph.
    /// </summary>
    /// <param name="stream">The animation stream</param>
    public void ProcessRootMotion(AnimationStream stream)
    {
        // Get root position and rotation.
        //OYM：获取roothandle的position

        var rootPosition = rootHandle.GetPosition(stream);
        var rootRotation = rootHandle.GetRotation(stream);
        //OYM：一些处理rootposition的任务
        //OYM：写回
        // The root always follow the given position and rotation.
        rootHandle.SetPosition(stream, rootPosition);
        rootHandle.SetRotation(stream, rootRotation);
    }

    /// <summary>
    /// Procedurally generate the joints rotation.
    /// </summary>
    /// <param name="stream">The animation stream</param>
    public void ProcessAnimation(AnimationStream stream)
    {
        if (jointHandles.Length < 2)
            return;

        ComputeDampedPositions(stream);
        ComputeJointLocalRotations(stream);
    }

    /// <summary>
    /// Compute the new global positions of the joints.
    ///
    /// The position of the first joint is driven by the root's position, and
    /// then the other joints positions are recomputed in order to follow their
    /// initial local positions, smoothly.
    ///
    /// Algorithm breakdown:
    ///     1. Compute the target position;
    ///     2. Damp this target position based on the current position;
    ///     3. Constrain the damped position to the joint initial length;
    ///     4. Iterate on the next joint.
    /// </summary>
    /// <param name="stream">The animation stream</param>
    private void ComputeDampedPositions(AnimationStream stream)
    {
        // Get root position and rotation.
        //OYM：获取root点的旋转和位置
        var rootPosition = rootHandle.GetPosition(stream);
        var rootRotation = rootHandle.GetRotation(stream);

        // The first non-root joint follows the root position,
        // but its rotation is damped (see ComputeJointLocalRotations).
        //OYM：获取第一个节点的位置与旋转
        var parentPosition = rootPosition + rootRotation * localPositions[0];
        var parentRotation = rootRotation * localRotations[0];
        positions[0] = parentPosition;
        for (var i = 1; i < jointHandles.Length; ++i)
        {
            // The target position is the global position, without damping.
            //OYM：计算当前节点根据当前的父节点的位置与父节点的初始旋转
            //OYM：有点绕，我简单解释一下
            //OYM：首先，假设父节点的localrotation没有发生变化的情况下
            //OYM：那么此时它的旋转就应该是parentrotation，乘以当前节点的localposition再加上父节点的worldposition就是当前应该在的位置
            var newPosition = parentPosition + (parentRotation * localPositions[i]);

            // Apply damping on this target.
           float3 velocity = velocities[i];
            // newPosition = Vector3.SmoothDamp(positions[i], newPosition, ref velocity, 0.15f, Mathf.Infinity, stream.deltaTime);
            //OYM：计算damppoisition。这个函数我抄出来了，但是也不大好理解（至少我没看懂）
            newPosition = SmoothDamp(positions[i], newPosition, ref velocity, 0.15f, Mathf.Infinity, stream.deltaTime);
            // Apply constraint: keep original length between joints.
            //OYM：限制与父节点的相对长度
            newPosition = parentPosition + (newPosition - parentPosition).normalized * localPositions[i].magnitude;

            // Save new velocity and position for next frame.
            //OYM：数据写入
            velocities[i] = velocity;
            positions[i] = newPosition;

            // Current joint is now the parent of the next joint.
            //OYM：当前节点作为下一节点的父节点进一步迭代
            parentPosition = newPosition;
            parentRotation = parentRotation * localRotations[i];
        }
    }

    /// <summary>
    /// Compute the new local rotations of the joints.
    ///
    /// Based on the global positions computed in ComputeDampedPositions,
    /// recompute the local rotation of each joint.
    ///
    /// Algorithm breakdown:
    ///     1. Compute the rotation between the current and new directions of the joint;
    ///     2. Apply this rotation on the current joint rotation;
    ///     3. Compute the local rotation and set it in the stream;
    ///     4. Iterate on the next joint.
    /// </summary>
    /// <param name="stream">The animation stream</param>
    private void ComputeJointLocalRotations(AnimationStream stream)
    {
        //OYM：根据上一次的结果，重新计算每一个节点的旋转
        //OYM：依靠旋转驱动节点位移
        //OYM：这部分看不懂的话建议先看lookatjob

        //OYM：root节点的旋转
        var parentRotation = rootHandle.GetRotation(stream);
        for (var i = 0; i < jointHandles.Length - 1; ++i)
        {
            // Get the current joint rotation.
            //OYM：计算当前应该所处的旋转
            var rotation = parentRotation * localRotations[i];

            // Get the current joint direction
            //OYM：计算当前子节点应该所处的位置（注意这个结果只会受到root的旋转影响）
            var direction = (rotation * localPositions[i + 1]).normalized;

            // Get the wanted joint direction.
            //OYM：计算当前节点到子节点的朝向（注意这个结果是上一步得到的）
            var newDirection = (positions[i + 1] - positions[i]).normalized;

            // Compute the rotation from the current direction to the new direction.
            //OYM：计算可以将direction旋转到newdirection的旋转
            var currentToNewRotation = Quaternion.FromToRotation(direction, newDirection);

            // Pre-rotate the current rotation, to get the new global rotation.
            //OYM：计算当前的世界旋转
            rotation = currentToNewRotation * rotation;

            // Set the new local rotation.
            var newLocalRotation = Quaternion.Inverse(parentRotation) * rotation;//OYM：反算localrotation
            jointHandles[i].SetLocalRotation(stream, newLocalRotation);//OYM：赋值

            // Set the new parent for the next joint.
            //OYM：迭代
            parentRotation = rotation;
        }

        
    }

}
