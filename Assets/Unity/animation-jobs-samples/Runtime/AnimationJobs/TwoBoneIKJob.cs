using UnityEngine;

#if UNITY_2019_3_OR_NEWER
using UnityEngine.Animations;
#else
using UnityEngine.Experimental.Animations;
#endif


public struct TwoBoneIKJob : IAnimationJob
{
    public TransformSceneHandle effector;

    public TransformStreamHandle top;
    public TransformStreamHandle mid;
    public TransformStreamHandle low;

    public void Setup(Animator animator, Transform topX, Transform midX, Transform lowX, Transform effectorX)
    {
        top = animator.BindStreamTransform(topX);
        mid = animator.BindStreamTransform(midX);
        low = animator.BindStreamTransform(lowX);

        effector = animator.BindSceneTransform(effectorX);
    }

    public void ProcessRootMotion(AnimationStream stream)
    {
    }

    public void ProcessAnimation(AnimationStream stream)
    {
        Solve(stream, top, mid, low, effector);
    }

    /// <summary>
    /// Returns the angle needed between v1 and v2 so that their extremities are
    /// spaced with a specific length.
    /// </summary>
    /// <returns>The angle between v1 and v2.</returns>
    /// <param name="aLen">The desired length between the extremities of v1 and v2.</param>
    /// <param name="v1">First triangle edge.</param>
    /// <param name="v2">Second triangle edge.</param>
    private static float TriangleAngle(float aLen, Vector3 v1, Vector3 v2) //OYM：计算在已知第三条边的长度的情况下，这条边对应的夹角（单位是弧度）
    {
        float aLen1 = v1.magnitude;
        float aLen2 = v2.magnitude;
        float c = Mathf.Clamp((aLen1 * aLen1 + aLen2 * aLen2 - aLen * aLen) / (aLen1 * aLen2) / 2.0f, -1.0f, 1.0f);//OYM：余弦定理
        return Mathf.Acos(c);
    }

    private static void Solve(AnimationStream stream, TransformStreamHandle topHandle, TransformStreamHandle midHandle, TransformStreamHandle lowHandle, TransformSceneHandle effectorHandle)
    {
        //OYM：看明白这一部分的话，可能需要一些草稿纸


        //OYM：骨骼的旋转，需要知道目标的旋转与倒数第二节与第三节的旋转
        Quaternion aRotation = topHandle.GetRotation(stream);
        Quaternion bRotation = midHandle.GetRotation(stream);
        Quaternion eRotation = effectorHandle.GetRotation(stream);
        //OYM：骨骼的位置，全部获取
        Vector3 aPosition = topHandle.GetPosition(stream);
        Vector3 bPosition = midHandle.GetPosition(stream);
        Vector3 cPosition = lowHandle.GetPosition(stream);
        Vector3 ePosition = effectorHandle.GetPosition(stream);
        //OYM：获取向量，AB ，BC， AC（三角形的三条边）以及AE（倒数第三节点到目标节点的距离）
        Vector3 ab = bPosition - aPosition;
        Vector3 bc = cPosition - bPosition;
        Vector3 ac = cPosition - aPosition;
        Vector3 ae = ePosition - aPosition;
         
        float abcAngle = TriangleAngle(ac.magnitude, ab, bc);//OYM：计算已AC，AB，BC组成的三角形中<ABC的夹角
        float abeAngle = TriangleAngle(ae.magnitude, ab, bc);//OYM：计算AE,AB,BE组成的三角形中<ABE的夹角
        float angle = (abcAngle - abeAngle) * Mathf.Rad2Deg; //OYM：夹角之差
        Vector3 axis = Vector3.Cross(ab, bc).normalized;//OYM：ab与ac构成的平面的法向量

        Quaternion fromToRotation = Quaternion.AngleAxis(angle, axis);//OYM：计算从ac到ae长度之间变化的四元数

        Quaternion worldQ = fromToRotation * bRotation; //OYM：点b旋转后所处的旋转
        midHandle.SetRotation(stream, worldQ);//OYM：设置b点的旋转

        cPosition = lowHandle.GetPosition(stream);//OYM：获取c点的位置
        ac = cPosition - aPosition;//OYM：计算ac之间的距离
        Quaternion fromTo = Quaternion.FromToRotation(ac, ae);//OYM：设置从ac到ae的旋转
        topHandle.SetRotation(stream, fromTo * aRotation);//OYM：设置跟节点的旋转

        lowHandle.SetRotation(stream, eRotation);//OYM：设置c节点的旋转
    }
}
