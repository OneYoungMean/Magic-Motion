using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace BioIK2
{
    public class BIOIK2Editor: MonoBehaviour
    {
        Animator animator;
        bool isInitial = false;

        Transform HeadLookAt;
        //OYM: ÷
        Transform leftHandPosition;
        Transform rightHandPosition;

        // Start is called before the first frame update
        void Start()
        {
            animator = transform.GetComponent<Animator>();
            if (animator == null || !animator.isHuman)
            {
                return;
            }
            BuildBodyBioObject();
        }

        private void BuildBodyBioObject()
        {

            var pelvis = animator.GetBoneTransform(HumanBodyBones.Hips);
            var spine = animator.GetBoneTransform(HumanBodyBones.Spine);

            var leftUpperLeg = animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
            var leftLowerLeg = animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
            var leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
            var leftToes = animator.GetBoneTransform(HumanBodyBones.LeftToes);

            var rightUpperLeg = animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);
            var rightLowerLeg = animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
            var rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
            var rightToes = animator.GetBoneTransform(HumanBodyBones.RightToes);

            var leftUpperArm = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            var leftLowerArm = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
            var leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);


            var rightUpperArm = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
            var rightLowerArm = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
            var rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
        }

        // Update is called once per frame
        void Update()
    {
        
    }
}
}