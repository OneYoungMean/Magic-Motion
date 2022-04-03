using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Test : MonoBehaviour
{
    // Start is called before the first frame update
    public Animator animatorA;
   public Animator animatorB;
    private HumanPoseHandler humanPoseHandlerA;
    private HumanPoseHandler humanPoseHandlerB;
    private HumanPose currentPoseA;
    private HumanPose currentPoseB;

    // Update is called once per framep
    public void Start()
    {
         humanPoseHandlerA = new HumanPoseHandler(animatorA.avatar, transform);
        humanPoseHandlerB = new HumanPoseHandler(animatorB.avatar, transform);
        currentPoseA = new HumanPose();
        humanPoseHandlerA.GetHumanPose(ref currentPoseA);

         currentPoseB = currentPoseA;
    }
    void Update()
    {


        //int leftEyeInOutMuscleIndex = HumanTrait.MuscleFromBone((int)HumanBodyBones.LeftEye, 1);
        //Transform leftEye = animator.GetBoneTransform(HumanBodyBones.LeftEye);
        //Quaternion leftEyeTaredRotation = leftEye.rotation;

        humanPoseHandlerA.GetHumanPose(ref currentPoseA);

        for (int i = 0; i < currentPoseA.muscles.Length; i++)
        {
            Debug.Log(currentPoseA.muscles[i]- currentPoseB.muscles[i]);
            currentPoseB.muscles[i] = -1;
        }
        humanPoseHandlerB.SetHumanPose(ref currentPoseB); 
currentPoseB = currentPoseA;


    }
}
