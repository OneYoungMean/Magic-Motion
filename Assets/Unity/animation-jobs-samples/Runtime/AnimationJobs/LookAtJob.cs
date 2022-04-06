using UnityEngine;

#if UNITY_2019_3_OR_NEWER
using UnityEngine.Animations;
#else
using UnityEngine.Experimental.Animations;
#endif

public struct LookAtJob : IAnimationJob
{
    public TransformStreamHandle joint;
    public TransformSceneHandle target;
    public Vector3 axis;
    public float minAngle;
    public float maxAngle;

    public void ProcessRootMotion(AnimationStream stream)
    {
    }

    public void ProcessAnimation(AnimationStream stream)
    {
        Solve(stream, joint, target, axis, minAngle, maxAngle);
    }

    private static void Solve(
        AnimationStream stream,
        TransformStreamHandle joint,
        TransformSceneHandle target,
        Vector3 jointAxis,
        float minAngle,
        float maxAngle)
    {
        //OYM：获取joint的位置，旋转， 目标的旋转
        var jointPosition = joint.GetPosition(stream);
        var jointRotation = joint.GetRotation(stream);
        var targetPosition = target.GetPosition(stream);

        //OYM：计算当前的direction
        var fromDir = jointRotation * jointAxis;
        //OYM：计算目标的direction
        var toDir = targetPosition - jointPosition;

        /*       var axis = Vector3.Cross(fromDir, toDir).normalized;
               var angle = Vector3.Angle(fromDir, toDir);
               angle = Mathf.Clamp(angle, minAngle, maxAngle);
               var jointToTargetRotation = Quaternion.AngleAxis(angle, axis);*/
        //OYM：上面一串都是fromto的描述。。。
        var jointToTargetRotation = Quaternion.FromToRotation(fromDir, toDir);
               jointRotation = jointToTargetRotation * jointRotation;
        //OYM：设置目标旋转
        joint.SetRotation(stream, jointRotation);
    }
}
