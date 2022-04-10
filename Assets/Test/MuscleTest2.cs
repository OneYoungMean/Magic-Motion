using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MuscleTest2 : MonoBehaviour
{
    // Start is called before the first frame update
    private HumanPoseHandler humanPoseHandler;
    public HumanPose currentPose;
    public Animator animator;

    void Start()
    {
        var avatar = animator.avatar;
        var humanDiscription = avatar.humanDescription;
        humanPoseHandler = new HumanPoseHandler(avatar, animator.transform);
        currentPose = new HumanPose();
        humanPoseHandler.GetHumanPose(ref currentPose);

        currentPose.muscles = new float[currentPose.muscles.Length];
    }

    // Update is called once per frame
    void Update()
    {
        humanPoseHandler.SetHumanPose(ref currentPose);
    }
}
