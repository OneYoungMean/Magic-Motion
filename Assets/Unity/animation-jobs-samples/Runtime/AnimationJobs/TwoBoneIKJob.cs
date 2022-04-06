using UnityEngine;

#if UNITY_2019_3_OR_NEWER
using UnityEngine.Animations;
#else
using UnityEngine.Experimental.Animations;
#endif


public struct TwoBoneIKJob : IAnimationJob
{
    public TransformSceneHandle effector;
    public Quaternion oldEffectorRotation;
    public TransformStreamHandle top;
    public TransformStreamHandle mid;
    public TransformStreamHandle low;

    public void Setup(Animator animator, Transform topX, Transform midX, Transform lowX, Transform effectorX)
    {
        top = animator.BindStreamTransform(topX);
        mid = animator.BindStreamTransform(midX);
        low = animator.BindStreamTransform(lowX);

        effector = animator.BindSceneTransform(effectorX);
        oldEffectorRotation = effectorX.rotation;
    }

    public void ProcessRootMotion(AnimationStream stream)
    {
    }

    public void ProcessAnimation(AnimationStream stream)
    {
        Solve(stream, top, mid, low, effector, oldEffectorRotation);
    }
    /**
 * Use the swing-twist decomposition to get the component of a rotation
 * around the given axis.
 *
 * N.B. assumes direction is normalized (to save work in calculating projection).
 * 
 * @param rotation  The rotation.
 * @param direction The axis.
 * @return The component of rotation about the axis.
 */
    private static Quaternion GetRotationComponentAboutAxis(
                Quaternion rotation, Vector3 direction)
    {
        direction = direction.normalized;//OYM����һ��
        Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);//OYM����ȡ��Ԫ����ת���ᣨ��ʱ���Ǳ�������cosa��
        float dotProduct = Vector3.Dot(direction,rotationAxis);
        // Shortcut calculation of `projection` requires `direction` to be normalized
        Vector3 projection = dotProduct* direction;
        Quaternion twist = new Quaternion(
                projection.x, projection.y, projection.z, rotation.w).normalized;
        if (dotProduct < 0.0)
        {
            // Ensure `twist` points towards `direction`
            twist.x = -twist.x;
            twist.y = -twist.y;
            twist.z = -twist.z;
            twist.w = -twist.w;
            // Rotation angle `twist.angle()` is now reliable
        }
        return twist;
    }
    /// <summary>
    /// Returns the angle needed between v1 and v2 so that their extremities are
    /// spaced with a specific length.
    /// </summary>
    /// <returns>The angle between v1 and v2.</returns>
    /// <param name="aLen">The desired length between the extremities of v1 and v2.</param>
    /// <param name="v1">First triangle edge.</param>
    /// <param name="v2">Second triangle edge.</param>
    private static float TriangleAngle(float aLen, Vector3 v1, Vector3 v2) //OYM����������֪�������ߵĳ��ȵ�����£������߶�Ӧ�ļнǣ���λ�ǻ��ȣ�
    {
        float aLen1 = v1.magnitude;
        float aLen2 = v2.magnitude;
        float c = Mathf.Clamp((aLen1 * aLen1 + aLen2 * aLen2 - aLen * aLen) / (aLen1 * aLen2) / 2.0f, -1.0f, 1.0f);//OYM�����Ҷ���
        return Mathf.Acos(c);
    }

    private static void Solve(AnimationStream stream, TransformStreamHandle topHandle, TransformStreamHandle midHandle, TransformStreamHandle lowHandle, TransformSceneHandle effectorHandle, Quaternion oldEffectorRotation)
    {
        //OYM����������һ���ֵĻ���������ҪһЩ�ݸ�ֽ


        //OYM����������ת����Ҫ֪��Ŀ�����ת�뵹���ڶ���������ڵ���ת
        Quaternion aRotation = topHandle.GetRotation(stream);
        Quaternion bRotation = midHandle.GetRotation(stream);
        Quaternion eRotation = effectorHandle.GetRotation(stream);
        //OYM��������λ�ã�ȫ����ȡ
        Vector3 aPosition = topHandle.GetPosition(stream);
        Vector3 bPosition = midHandle.GetPosition(stream);
        Vector3 cPosition = lowHandle.GetPosition(stream);
        Vector3 ePosition = effectorHandle.GetPosition(stream);
        //OYM����ȡ������AB ��BC�� AC�������ε������ߣ��Լ�AE�����������ڵ㵽Ŀ��ڵ�ľ��룩
        Vector3 ab = bPosition - aPosition;
        Vector3 bc = cPosition - bPosition;
        Vector3 ac = cPosition - aPosition;
        Vector3 ae = ePosition - aPosition;
         
        float abcAngle = TriangleAngle(ac.magnitude, ab, bc);//OYM��������AC��AB��BC��ɵ���������<ABC�ļн�
        float abeAngle = TriangleAngle(ae.magnitude, ab, bc);//OYM������AE,AB,BE��ɵ���������<ABE�ļн�
        float angle = (abcAngle - abeAngle) * Mathf.Rad2Deg; //OYM���н�֮��
        Vector3 axis = Vector3.Cross(ab, bc).normalized;//OYM��ab��ac���ɵ�ƽ��ķ�����

        Quaternion fromToRotation = Quaternion.AngleAxis(angle, axis);//OYM�������ac��ae����֮��仯����Ԫ��

        Quaternion worldQ = fromToRotation * bRotation; //OYM����b��ת����������ת

        Quaternion deltaRotation = eRotation * Quaternion.Inverse(oldEffectorRotation);

        Quaternion axisRotationB = GetRotationComponentAboutAxis(deltaRotation, bc);//OYM����ȡdeltarotation��bc�����ת

        //worldQ = axisRotationB * worldQ;

        midHandle.SetRotation(stream, worldQ);//OYM������b�����ת

        cPosition = lowHandle.GetPosition(stream);//OYM����ȡc���λ��

        ac = cPosition - aPosition;//OYM������ac֮��ľ���

        Quaternion fromTo = Quaternion.FromToRotation(ac, ae);//OYM�����ô�ac��ae����ת

        aRotation = fromTo * aRotation;//OYM������fromto����ת

        Quaternion axisRotationA = GetRotationComponentAboutAxis(axisRotationB, ab);//OYM����ȡaxisRotationB��ab�����ת

        //aRotation = axisRotationA * aRotation;

        topHandle.SetRotation(stream, aRotation);//OYM�����ø��ڵ����ת

        lowHandle.SetRotation(stream, eRotation);//OYM������c�ڵ����ת
    }
}
