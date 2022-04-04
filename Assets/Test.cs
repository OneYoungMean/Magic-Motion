using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{
    // Start is called before the first frame update
    public Animator animator;
    private HumanPose currentPose;

    void Start()
    {


        // In Limit
        /*        destinationPose.muscles[leftEyeInOutMuscleIndex] = 1f;
                humanPoseHandler.SetHumanPose(ref destinationPose);
                Quaternion leftEyeMaxInRotation = leftEye.rotation;
                float leftEyeMaxInLimit = Quaternion.Angle(leftEyeTaredRotation, leftEyeMaxInRotation);*/

        HumanPoseHandler humanPoseHandler = new HumanPoseHandler(animator.avatar, transform);
        int leftEyeInOutMuscleIndex = HumanTrait.MuscleFromBone((int)HumanBodyBones.LeftEye, 1);
        Transform leftEye = animator.GetBoneTransform(HumanBodyBones.LeftEye);
        Quaternion leftEyeTaredRotation = leftEye.rotation;
         currentPose = new HumanPose();
        humanPoseHandler.GetHumanPose(ref currentPose);
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(currentPose.muscles[85]);
        

/*        for (int i = 0; i < currentPose.muscles.Length; i++)
        {
            humanPoseHandler.SetHumanPose(ref destinationPose);
        }*/
/*        // Out Limit

        */
    }
}
