using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System;
using System.Linq;

namespace MagicMotion.Mono
{
    /// <summary>
    /// The joint and constraint manager
    /// </summary>
    public class MMHumanGenerator : MonoBehaviour,IGetConstraint
    {
        public const float LOOKAT_LENGTH = 0.1f;

        #region  Field&Property
        [HideInInspector]
        /// <summary>
        /// Constraint root
        /// </summary>
        public GameObject ConstraintAimRoot;

        /// <summary>
        /// target animator
        /// </summary>
        public Animator animator;

        [HideInInspector]
        /// <summary>
        /// is controller initialize?
        /// </summary>
        public bool isInitialize;

        public bool isIndepentdentHand;
        public bool isBuildConstraintRelation;

        public List<MMJointController> jointControllers;
        public List<MMConstraint> motionConstraints;
        public MMJoint[] motionJoints;
        public MMMuscle[] motionMuscles;


        #endregion

        #region UnityFunc
        private void Start()
        {
            if (!isInitialize)
            {
                Generate();
            }
        }

        #endregion

        #region LocalFunc
        /// <summary>

        /// <summary>
        /// initiailize data
        /// </summary>
        /// <param name="animator"></param>
        /// <param name="isRefresh"></param>
        /// <exception cref="InvalidOperationException"></exception>
        public void Generate(Animator animator = null, bool isRefresh = false)
        {
            if (!isRefresh && isInitialize)
            {
                return;
            }

            if (animator == null)
            {
                animator = gameObject.GetComponent<Animator>();
                if (animator == null || !animator.isHuman)
                {
                    throw new InvalidOperationException("invaild animator");
                }
            }
            this.animator = animator;

            Initialize1();
            Initialize2();
            PostProcess();
            isInitialize = true;
        }

        /// <summary>
        ///  Get Constraint By HumanBone
        /// </summary>

        public MMConstraint GetConstraintTarget(HumanBodyBones humanBodyBones, MMConstraintType constraintType)
        {
            var targetJoint = motionJoints.FirstOrDefault(x => x !=null&& x.humanBodyBone == humanBodyBones);
            if (targetJoint == null) return null;

            switch (constraintType)
            {
                case MMConstraintType.Position:
                    return targetJoint.constraints[0];
                case MMConstraintType.LookAt:
                    return targetJoint.constraints[1] ;
                default:
                    return null ;
            }
        }
        private void PostProcess()
        {
            jointControllers = new List<MMJointController>();
            if (!isIndepentdentHand)
            {
                
              var jointController =   gameObject.AddComponent<MMJointController>();
                jointController.rootTransform = animator.transform;
                jointController.motionJoints = motionJoints.Where(x => x != null).ToArray();
   
                jointControllers.Add(jointController);
            }
            else
            {
                var bodyController = gameObject.AddComponent<MMJointController>();
                bodyController.rootTransform = animator.transform;
                bodyController.motionJoints = motionJoints.Where(x=> x!=null&&GetJointKind( x.humanBodyBone)==JointKind.Body).ToArray();

                jointControllers.Add(bodyController);

                var leftHandController = gameObject.AddComponent<MMJointController>();
                leftHandController.rootTransform = animator.GetBoneTransform(HumanBodyBones.LeftHand);
                leftHandController.motionJoints = motionJoints.Where(x => x != null && GetJointKind(x.humanBodyBone) == JointKind.LeftHand).ToArray();

                jointControllers.Add(leftHandController);
                var rightHandController = gameObject.AddComponent<MMJointController>();
                rightHandController.rootTransform = animator.GetBoneTransform(HumanBodyBones.RightHand);
                rightHandController.motionJoints = motionJoints.Where(x => x != null && GetJointKind(x.humanBodyBone) == JointKind.RightHand).ToArray();

                jointControllers.Add(rightHandController);
            }
            for (int i = 0; i < jointControllers.Count; i++)
            {
                jointControllers[i].Initialize();
            }
        }

        /// <summary>
        /// step one : build joint and muscle
        /// </summary>
        private void Initialize1()
        {

            Avatar avatar = animator.avatar;
            var humandescription = avatar.humanDescription;

            HumanPoseHandler humanPoseHandler = new HumanPoseHandler(animator.avatar, animator.transform);
            HumanPose currentPose = new HumanPose();
            humanPoseHandler.GetHumanPose(ref currentPose);
            currentPose.bodyPosition = new Vector3(0, currentPose.bodyPosition.y, 0);
            currentPose.bodyRotation = Quaternion.identity;

            float[] muscleValue = currentPose.muscles;
            for (int i = 0; i < muscleValue.Length; i++)
            {
                muscleValue[i]=math.clamp(muscleValue[i], -1, 1);
            }

            motionJoints = new MMJoint[HumanTrait.BoneCount];
            motionMuscles = new MMMuscle[muscleValue.Length];


            #region Generate Muscle and joint
            for (int i = 0 ;
                i < muscleValue.Length;
                i++)
            {
                int muscleIndex = i;
                if (GetMuscleData(muscleIndex, out int jointIndex, out int dof, out string muscleName, out string jointName))
                {
                    Transform targetJoint = animator.GetBoneTransform((HumanBodyBones)jointIndex);
                    if (targetJoint != null)
                    {

                        motionMuscles[i] = targetJoint.gameObject.AddComponent<MMMuscle>();
                        //motionMuscles[i].value = muscleValue[muscleIndex];
                        motionMuscles[i].dof = dof;
                        motionMuscles[i].muscleIndex = muscleIndex;
                        motionMuscles[i].muscleName = muscleName;

                        GetMuscleRangeAndAxis(targetJoint, currentPose, humanPoseHandler, muscleIndex, out float minAngle, out float maxAngle, out float3 axis);


                        MMJoint motionJoint = motionJoints[jointIndex];
                        if (motionJoint == null)
                        {
                            motionJoint = targetJoint.gameObject.AddComponent<MMJoint>();
                            /*                            motionJoint.position = targetJoint.position;
                                                        motionJoint.rotation = targetJoint.rotation;*/
                            motionJoint.humanBodyBone = (HumanBodyBones)jointIndex;
                            motionJoint.jointName = jointName;

                        }
                        motionJoint.muscles[dof] = motionMuscles[i];
                        motionMuscles[i].joint = motionJoint;

                        float3x3 dof3Axis = motionJoint.dof3Axis;
                        dof3Axis[dof] = axis;
                        motionJoint.dof3Axis = dof3Axis;

                        float3 minRange = motionJoint.minRange;
                        minRange[dof] = minAngle;
                        motionJoint.minRange = minRange;

                        float3 maxRange = motionJoint.maxRange;
                        maxRange[dof] = maxAngle;
                        motionJoint.maxRange = maxRange;

                        motionJoints[jointIndex] = motionJoint;
                    }
                }
            }

            #endregion
            #region Clac localPosition and localRotation

            for (int i = 0; i < motionJoints.Length; i++)
            {
                var currentJoint = motionJoints[i];
                if (motionJoints[i] == null)
                {
                    continue;
                }
                currentJoint.jointIndex = i;
                for (Transform parentTransform = currentJoint.transform.parent;
                    parentTransform != null;
                    parentTransform = parentTransform.parent)
                {
                    MMJoint parentJoint = motionJoints.FirstOrDefault(x => x!=null&& x.transform == parentTransform);
                    if (parentJoint != null)
                    {
                        currentJoint.parent = parentJoint;
                        break;
                    }
                }
                motionJoints[i] = currentJoint;
            }
            #endregion

        }
        /// <summary>
        /// step two: build constraitnt and other thing
        /// </summary>
        private void Initialize2()
        {
            motionConstraints = new List<MMConstraint>();
/*            ConstraintAimRoot = new GameObject("Aim Root");


            ConstraintAimRoot.transform.parent = transform;
            ConstraintAimRoot.transform.localPosition = Vector3.zero;
            ConstraintAimRoot.transform.localRotation = Quaternion.identity;*/

            AddConstraint(MMConstraintType.Position);
           //AddConstraint(MMConstraintType.DofChange);
        }
        /// <summary>
        /// add constraint on your joint 
        /// </summary>
        /// <param name="constraintType"></param>
        private void AddConstraint(MMConstraintType constraintType)
        {
            for (int i = 1; i < motionJoints.Length; i++)
            {
                var joint = motionJoints[i];
                if (joint==null)
                {
                    continue;
                }
                MMConstraint constraint;
                switch (constraintType)
                {
                    case MMConstraintType.Position:
                        GameObject positionIK = new GameObject("IK_" + this.motionJoints[i].name + "_Position");


                        var positionConstraint =
                       joint.gameObject.AddComponent<MMPositionConstraint>();
                        
                        positionConstraint.weight3 =Vector3.one;
                        positionConstraint.AddTarget(positionIK.transform);
                        constraint = positionConstraint;

                        if (isBuildConstraintRelation&&joint.parent != null && joint.parent.constraints[(int)constraintType] != null)
                        {
                            positionIK.transform.parent =( joint.parent.constraints[(int)constraintType] as MMPositionConstraint).targetTransform;
                        }
                        else
                        {
                            positionIK.transform.parent = transform;
                        }
                        positionIK.transform.position = joint.transform.position;
                        break;
                    case MMConstraintType.Rotation:
                        constraint = null;
                        break;
                    case MMConstraintType.LookAt:

                        GameObject lookAtIK = new GameObject("IK_" + motionJoints[i].name + "_Lookat");
                        lookAtIK.transform.parent = joint.transform;
                        lookAtIK.transform.position = joint.transform.position + LOOKAT_LENGTH * joint.transform.forward;
                        var lookatConstraint= joint.gameObject.AddComponent<MMLookConstraint>();
                        lookatConstraint.weight=1;
                        lookatConstraint.jointDirection = Vector3.forward;
                        lookatConstraint.AddTarget(lookAtIK.transform);
                        constraint = lookatConstraint;

                        break;
                    case MMConstraintType.Collider:
                        constraint = null;
                        break;
                    case MMConstraintType.PositionChange:
                        var positionChangeConstraint = joint.gameObject.AddComponent<MMPositionChangeConstraint>();
                        positionChangeConstraint.weight3 = Vector3.one;
                        constraint=positionChangeConstraint;
                        break;
                    case MMConstraintType.DofChange:
                        var DofChangeConstraint = joint.gameObject.AddComponent<MMDofChangeConstraint>();
                        DofChangeConstraint.weight3 = Vector3.one;
                        constraint = DofChangeConstraint;
                        break;
                    default:
                        constraint= null;
                        break;
                }
                joint.constraints[(int)constraintType] = constraint;
                if (constraint!=null)
                {
                    motionConstraints.Add(constraint);
                }
            }
        }

        #endregion

        #region StaticFunc
        /// <summary>
        /// Get muscle's axis and range.
        /// </summary>
        /// <param name="targetBone"></param>
        /// <param name="currentPose"></param>
        /// <param name="humanPoseHandler"></param>
        /// <param name="muscleIndex"></param>
        /// <param name="minAngle"></param>
        /// <param name="maxAngle"></param>
        /// <param name="axis"></param>
        private static void GetMuscleRangeAndAxis(Transform targetBone, HumanPose currentPose, HumanPoseHandler humanPoseHandler, int muscleIndex, out float minAngle, out float maxAngle, out float3 axis)
        {
            float[] muscleValue = currentPose.muscles;
            float defaultValue = muscleValue[muscleIndex];

/*            minAngle = HumanTrait.GetMuscleDefaultMin(muscleIndex);
            maxAngle = HumanTrait.GetMuscleDefaultMax(muscleIndex);*/

            muscleValue[muscleIndex] = 1;
            humanPoseHandler.SetHumanPose(ref currentPose);
            Quaternion positiveLocalRotation = targetBone.localRotation;


            muscleValue[muscleIndex] = -1;
            humanPoseHandler.SetHumanPose(ref currentPose);
            Quaternion negativeLocalRotation = targetBone.localRotation;

            muscleValue[muscleIndex] = 0;
            humanPoseHandler.SetHumanPose(ref currentPose);
            Quaternion initialLocalRotation = targetBone.localRotation;

            muscleValue[muscleIndex] = defaultValue;
            humanPoseHandler.SetHumanPose(ref currentPose);

            //OYM£ºpositiveLocalRotation=initialLocalRotation *positiveRotate,it should under the initialLocalRotation axis.
            Quaternion positiveRotate = Quaternion.Inverse(initialLocalRotation) * positiveLocalRotation;
            Quaternion negativeRotate = Quaternion.Inverse(initialLocalRotation) * negativeLocalRotation;

            float initMaxAngle, initMinAngle;
            positiveRotate.ToAngleAxis(out initMaxAngle, out Vector3 axis1); //OYM£ºsome bone will loss some rotation,so use default rotation (like upper arm)
            negativeRotate.ToAngleAxis(out initMinAngle, out Vector3 axis2);

            if (defaultValue>0)
            {
                maxAngle = math.lerp(initMaxAngle, 0, defaultValue);
                minAngle =-( initMinAngle +initMaxAngle) + maxAngle;
            }
            else
            {
                minAngle = -math.lerp(initMaxAngle, 0, -defaultValue);
                maxAngle = initMinAngle + initMaxAngle + minAngle;
            }
            axis = axis1;
        }
        /// <summary>
        /// get muscles's message
        /// </summary>
        /// <param name="muscleIndex"></param>
        /// <param name="boneIndex"></param>
        /// <param name="dof"></param>
        /// <param name="muscleName"></param>
        /// <param name="boneName"></param>
        /// <returns></returns>
        private static bool GetMuscleData(int muscleIndex, out int boneIndex, out int dof, out string muscleName, out string boneName)
        {
            dof = -1;
            muscleName = null;
            boneName = null;
            boneIndex = HumanTrait.BoneFromMuscle(muscleIndex);
            if (boneIndex == -1)
            {
                return false;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    int muscleIndexCmp = HumanTrait.MuscleFromBone(boneIndex, i);
                    if (muscleIndexCmp == muscleIndex)
                    {
                        dof = i;
                        break;
                    }
                }
                muscleName = HumanTrait.MuscleName[muscleIndex];
                boneName = HumanTrait.BoneName[boneIndex];
                return true;
            }
        }

        private static JointKind GetJointKind(HumanBodyBones humanBodyBones)
        {
            int v = (int)humanBodyBones;
            return v switch
            {
                < 24 or >=54 => JointKind.Body,
                >=24  and <39 => JointKind.LeftHand,
                >=39 and <54=>JointKind.RightHand,
            };
        }
        #endregion
    }
}
#region note
/*          

    public static unsafe void RemoveAllWhitespace(ref string str)
{
    fixed (char* pfixed = str)
    {
        char* dst = pfixed;
        for (char* p = pfixed; *p != 0; p++)
        {
            switch (*p)
            {
                case '\u0020':
                case '\u00A0':
                case '\u1680':
                case '\u2000':
                case '\u2001':
                case '\u2002':
                case '\u2003':
                case '\u2004':
                case '\u2005':
                case '\u2006':
                case '\u2007':
                case '\u2008':
                case '\u2009':
                case '\u200A':
                case '\u202F':
                case '\u205F':
                case '\u3000':
                case '\u2028':
                case '\u2029':
                case '\u0009':
                case '\u000A':
                case '\u000B':
                case '\u000C':
                case '\u000D':
                case '\u0085':
                    continue;

                default:
                    *dst++ = *p;
                    break;
            }
        }

        uint* pi = (uint*)pfixed;
        ulong len = ((ulong)dst - (ulong)pfixed) >> 1;
        pi[-1] = (uint)len;
        pfixed[len] = '\0';
    }
}
*/
/*        var avatar = animator.avatar;
        var humanDiscription = avatar.humanDescription;
         humanPoseHandler = new HumanPoseHandler(avatar, animator.transform);
        currentPose = new HumanPose();
        humanPoseHandler.GetHumanPose(ref currentPose);

        joints = new MagicMotionJoint[(int)HumanBodyBones.LastBone];

        var defalutMuscleValue = new HumanPose();
        defalutMuscleValue.bodyPosition = currentPose.bodyPosition;
        defalutMuscleValue.bodyRotation = currentPose.bodyRotation;

        defalutMuscleValue.muscles = new float[currentPose.muscles.Length];
        currentPose.muscles = new float[currentPose.muscles.Length];
        *//*        currentPose.muscles[0] = value;
                currentPose.muscles[1] = value1;
                currentPose.muscles[2] = value2;*//*
        humanPoseHandler.SetHumanPose(ref defalutMuscleValue);
        for (int i = 0; i < humanDiscription.human.Length; i++)
        {
            var jointDescription = humanDiscription.human[i];
            int boneIndex= Array.IndexOf(HumanTrait.BoneName, jointDescription.humanName);
            HumanBodyBones humanBodyBone = (HumanBodyBones)boneIndex;
            Transform jointTransform = animator.GetBoneTransform(humanBodyBone);
            if (jointTransform==null)
            {
                continue;
            }

            MagicMotionJoint motionJoint = new MagicMotionJoint(true);
            motionJoint.humanBodyBone = humanBodyBone;
            var limit = jointDescription.limit;
            float3 dof3 = 0;
            float3 min = 0;
            float3 max = 0;
            for (int j0 = 0; j0 < 3; j0++)//OYM£ºdof
            {
                int muscleIndex = HumanTrait.MuscleFromBone((int)humanBodyBone, j0);
                if (muscleIndex != -1)
                {
                    dof3[j0] = currentPose.muscles[muscleIndex];
                    min[j0] = HumanTrait.GetMuscleDefaultMin(muscleIndex);
                    max[j0] = HumanTrait.GetMuscleDefaultMax(muscleIndex);
                }
            }
            motionJoint.Dof3 = dof3;

            if (limit.useDefaultValues)
            {

                motionJoint.MaxRange = max;
                motionJoint.MinRange = min;
            }
            else
            {
                motionJoint.MaxRange = limit.max;
                motionJoint.MinRange = limit.min;
            }


            motionJoint.position = jointTransform.position;
            motionJoint.rotation = jointTransform.rotation;

            joints[boneIndex] = motionJoint;
        }

/*        for (int i = 0; i < joints.Length; i++)
        {
            for (int j0 = 0; j0 < 3; j0++)//OYM£ºdof
            {
                int muscleIndex = HumanTrait.MuscleFromBone(i, j0);
                if (muscleIndex != -1)
                {
                    joints[i].Dof3[j0] = currentPose.muscles[muscleIndex];
                }
            }
        }
        if (isUpdatePosition)
        {
            RigidTransform[] jointUpdateTransform =new RigidTransform[joints.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                //OYM£ºgetdata
                int currentIndex = i;
                MagicMotionJoint currentJoint = joints[currentIndex];
                if (!currentJoint.isVaild)
                {
                    continue;
                }

                Transform jointTransform = animator.GetBoneTransform((HumanBodyBones)currentIndex);
                RigidTransform currentTrans = new RigidTransform();

                //OYM£ºDof to euler angle
                float3 Dof3toEuler = 
                    math.lerp(0, currentJoint.MinRange, -math.clamp(currentJoint.Dof3, -2, 0))
                    + math.lerp(0, currentJoint.MaxRange, math.clamp(currentJoint.Dof3, 0, 2));

                Dof3toEuler = Dof3toEuler * new float3(0, -1, -1);
                quaternion euler = quaternion.Euler(math.radians(Dof3toEuler));
               // quaternion euler = quaternion.identity;

                quaternion parentRotation; float3 parentPosition;
                if (currentJoint.parentIndex == -1)
                {
                    parentPosition = animator.transform.position;
                     parentRotation = animator.transform.rotation; ;
                }
                else
                {
                    parentRotation = jointUpdateTransform[currentJoint.parentIndex].rot;
                    parentPosition = jointUpdateTransform[currentJoint.parentIndex].pos;

                }
                currentTrans.rot = math.mul(parentRotation, math.mul(currentJoint.localRotation, euler));
                currentTrans.pos = parentPosition + math.mul(parentRotation, currentJoint.localPosition);

                jointUpdateTransform[i] = currentTrans;
                jointTransform.position = jointUpdateTransform[i].pos;
                jointTransform.rotation = jointUpdateTransform[i].rot;
            }
        }*/
/*
        [System.Serializable]
        internal class AvatarMuscleEditor 
        {
        public enum HumanDofStart
        {
            BodyDofStart = 0,
            HeadDofStart = 9,
            LeftLegDofStart = 21,
            RightLegDofStart = 29,
            LeftArmDofStart = 37,
            RightArmDofStart = 46,
            LeftThumbDofStart = 55,
            LeftIndexDofStart = 59,
            LeftMiddleDofStart = 0x3F,
            LeftRingDofStart = 67,
            LeftLittleDofStart = 71,
            RightThumbDofStart = 75,
            RightIndexDofStart = 79,
            RightMiddleDofStart = 83,
            RightRingDofStart = 87,
            RightLittleDofStart = 91,
            LastDof = 95
        }
            class Styles
            {
                public GUIContent[] muscleBodyGroup =
                {
                EditorGUIUtility.TrTextContent("Body"),
                EditorGUIUtility.TrTextContent("Head"),
                EditorGUIUtility.TrTextContent("Left Arm"),
                EditorGUIUtility.TrTextContent("Left Fingers"),
                EditorGUIUtility.TrTextContent("Right Arm"),
                EditorGUIUtility.TrTextContent("Right Fingers"),
                EditorGUIUtility.TrTextContent("Left Leg"),
                EditorGUIUtility.TrTextContent("Right Leg")
            };

                public GUIContent[] muscleTypeGroup =
                {
                EditorGUIUtility.TrTextContent("Open Close"),
                EditorGUIUtility.TrTextContent("Left Right"),
                EditorGUIUtility.TrTextContent("Roll Left Right"),
                EditorGUIUtility.TrTextContent("In Out"),
                EditorGUIUtility.TrTextContent("Roll In Out"),
                EditorGUIUtility.TrTextContent("Finger Open Close"),
                EditorGUIUtility.TrTextContent("Finger In Out")
            };

                public GUIContent armTwist = EditorGUIUtility.TrTextContent("Upper Arm Twist");
                public GUIContent foreArmTwist = EditorGUIUtility.TrTextContent("Lower Arm Twist");
                public GUIContent upperLegTwist = EditorGUIUtility.TrTextContent("Upper Leg Twist");
                public GUIContent legTwist = EditorGUIUtility.TrTextContent("Lower Leg Twist");
                public GUIContent armStretch = EditorGUIUtility.TrTextContent("Arm Stretch");
                public GUIContent legStretch = EditorGUIUtility.TrTextContent("Leg Stretch");
                public GUIContent feetSpacing = EditorGUIUtility.TrTextContent("Feet Spacing");
                public GUIContent hasTranslationDoF = EditorGUIUtility.TrTextContent("Translation DoF");

                public GUIStyle box = "OL box noexpand";
                public GUIStyle title = "OL TITLE";

                public GUIStyle toolbar = "TE Toolbar";
                public GUIStyle toolbarDropDown = "TE ToolbarDropDown";

                public GUIContent muscle = EditorGUIUtility.TrTextContent("Muscles");
                public GUIContent resetMuscle = EditorGUIUtility.TrTextContent("Reset");
            }

            static Styles styles { get { if (s_Styles == null) s_Styles = new Styles(); return s_Styles; } }
            static Styles s_Styles;

            // This list containt the mecanim's musle id for each muscle group
            protected int[][] m_Muscles =
            {
            new int[] { (int)HumanDofStart.BodyDofStart + (int)BodyDof.SpineFrontBack,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.SpineLeftRight,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.SpineRollLeftRight,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.ChestFrontBack,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.ChestLeftRight,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.ChestRollLeftRight,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.UpperChestFrontBack,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.UpperChestLeftRight,
                        (int)HumanDofStart.BodyDofStart + (int)BodyDof.UpperChestRollLeftRight},

            new int[] { (int)HumanDofStart.HeadDofStart + (int)HeadDof.NeckFrontBack,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.NeckLeftRight,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.NeckRollLeftRight,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.HeadFrontBack,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.HeadLeftRight,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.HeadRollLeftRight,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.LeftEyeDownUp,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.LeftEyeInOut,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.RightEyeDownUp,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.RightEyeInOut,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.JawDownUp,
                        (int)HumanDofStart.HeadDofStart + (int)HeadDof.JawLeftRight},

            new int[] { (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ShoulderDownUp,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ShoulderFrontBack,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ArmDownUp,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ArmFrontBack,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ArmRollInOut,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ForeArmCloseOpen,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ForeArmRollInOut,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.HandDownUp,
                        (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.HandInOut},

            new int[]
            {
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.DistalCloseOpen
            },

            new int[] { (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ShoulderDownUp,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ShoulderFrontBack,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ArmDownUp,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ArmFrontBack,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ArmRollInOut,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ForeArmCloseOpen,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ForeArmRollInOut,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.HandDownUp,
                        (int)HumanDofStart.RightArmDofStart + (int)ArmDof.HandInOut},

            new int[]
            {
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.DistalCloseOpen
            },

            new int[]
            {
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.UpperLegFrontBack,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.UpperLegInOut,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.UpperLegRollInOut,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.LegCloseOpen,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.LegRollInOut,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.FootCloseOpen,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.FootInOut,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.ToesUpDown,
            },

            new int[]
            {
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.UpperLegFrontBack,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.UpperLegInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.UpperLegRollInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.LegCloseOpen,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.LegRollInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.FootCloseOpen,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.FootInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.ToesUpDown,
            }
        };

            protected int[][] m_MasterMuscle =
            {
            // Body open close
            new int[]
            {
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.SpineFrontBack,
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.ChestFrontBack,
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.UpperChestFrontBack,
                (int)HumanDofStart.HeadDofStart + (int)HeadDof.NeckFrontBack,
                (int)HumanDofStart.HeadDofStart + (int)HeadDof.HeadFrontBack,

                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.UpperLegFrontBack,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.LegCloseOpen,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.FootCloseOpen,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.UpperLegFrontBack,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.LegCloseOpen,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.FootCloseOpen,

                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ShoulderDownUp,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ArmDownUp,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ForeArmCloseOpen,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.HandDownUp,

                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ShoulderDownUp,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ArmDownUp,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ForeArmCloseOpen,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.HandDownUp
            },

            // Body Left Right
            new int[]
            {
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.SpineLeftRight,
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.ChestLeftRight,
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.UpperChestLeftRight,
                (int)HumanDofStart.HeadDofStart + (int)HeadDof.NeckLeftRight,
                (int)HumanDofStart.HeadDofStart + (int)HeadDof.HeadLeftRight,
            },

            // Roll Left Right
            new int[]
            {
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.SpineRollLeftRight,
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.ChestRollLeftRight,
                (int)HumanDofStart.BodyDofStart + (int)BodyDof.UpperChestRollLeftRight,
                (int)HumanDofStart.HeadDofStart + (int)HeadDof.NeckRollLeftRight,
                (int)HumanDofStart.HeadDofStart + (int)HeadDof.HeadRollLeftRight,
            },

            // In Out
            new int[]
            {
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.UpperLegInOut,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.FootInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.UpperLegInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.FootInOut,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ShoulderFrontBack,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ArmFrontBack,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.HandInOut,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ShoulderFrontBack,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ArmFrontBack,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.HandInOut
            },

            // Roll In Out
            new int[]
            {
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.UpperLegRollInOut,
                (int)HumanDofStart.LeftLegDofStart + (int)LegDof.LegRollInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.UpperLegRollInOut,
                (int)HumanDofStart.RightLegDofStart + (int)LegDof.LegRollInOut,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ArmRollInOut,
                (int)HumanDofStart.LeftArmDofStart + (int)ArmDof.ForeArmRollInOut,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ArmRollInOut,
                (int)HumanDofStart.RightArmDofStart + (int)ArmDof.ForeArmRollInOut
            },

            // Finger open close
            new int[]
            {
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.DistalCloseOpen,

                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.ProximalDownUp,
                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.IntermediateCloseOpen,
                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.DistalCloseOpen
            },

            // Finger In Out
            new int[]
            {
                (int)HumanDofStart.LeftThumbDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftIndexDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftMiddleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftRingDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.LeftLittleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightThumbDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightIndexDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightMiddleDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightRingDofStart + (int)FingerDof.ProximalInOut,
                (int)HumanDofStart.RightLittleDofStart + (int)FingerDof.ProximalInOut,
            }
        };


            bool[] m_MuscleBodyGroupToggle;
            bool[] m_MuscleToggle;

            int m_FocusedMuscle;

            [SerializeField]
            float[] m_MuscleValue = null;

            [SerializeField]
            float[] m_MuscleMasterValue = null;

            [SerializeField]
            protected float m_ArmTwistFactor;

            [SerializeField]
            protected float m_ForeArmTwistFactor;

            [SerializeField]
            protected float m_UpperLegTwistFactor;

            [SerializeField]
            protected float m_LegTwistFactor;

            [SerializeField]
            protected float m_ArmStretchFactor;

            [SerializeField]
            protected float m_LegStretchFactor;

            [SerializeField]
            protected float m_FeetSpacingFactor;

            [SerializeField]
            protected bool m_HasTranslationDoF;

            string[] m_MuscleName = null;
            int m_MuscleCount = 0;

            SerializedProperty[] m_MuscleMin = null;
            SerializedProperty[] m_MuscleMax = null;

            [SerializeField]
            float[] m_MuscleMinEdit = null;

            [SerializeField]
            float[] m_MuscleMaxEdit = null;

            SerializedProperty[] m_Modified = null;

            // These member are used when the avatar is part of an asset
            SerializedProperty m_ArmTwistProperty;
            SerializedProperty m_ForeArmTwistProperty;
            SerializedProperty m_UpperLegTwistProperty;
            SerializedProperty m_LegTwistProperty;

            SerializedProperty m_ArmStretchProperty;
            SerializedProperty m_LegStretchProperty;

            SerializedProperty m_FeetSpacingProperty;

            SerializedProperty m_HasTranslationDoFProperty;
            SerializedProperty m_HumanBoneArrayProperty;

            const string sMinX = "m_Limit.m_Min.x";
            const string sMinY = "m_Limit.m_Min.y";
            const string sMinZ = "m_Limit.m_Min.z";

            const string sMaxX = "m_Limit.m_Max.x";
            const string sMaxY = "m_Limit.m_Max.y";
            const string sMaxZ = "m_Limit.m_Max.z";
            const string sModified = "m_Limit.m_Modified";

            const string sArmTwist = "m_HumanDescription.m_ArmTwist";
            const string sForeArmTwist = "m_HumanDescription.m_ForeArmTwist";
            const string sUpperLegTwist = "m_HumanDescription.m_UpperLegTwist";
            const string sLegTwist = "m_HumanDescription.m_LegTwist";

            const string sArmStretch = "m_HumanDescription.m_ArmStretch";
            const string sLegStretch = "m_HumanDescription.m_LegStretch";

            const string sFeetSpacing = "m_HumanDescription.m_FeetSpacing";

            const string sHasTranslationDoF = "m_HumanDescription.m_HasTranslationDoF";

            const string sHumanBoneArray = "m_HumanDescription.m_Human";

            const float sMuscleMin = -180.0f;
            const float sMuscleMax = 180.0f;

            const float kPreviewWidth = 80;
            const float kNumberWidth = 38;
            const float kLineHeight = 18;

            private readonly Handles.BoneRenderer m_BoneRenderer = new Handles.BoneRenderer();

            static Rect GetSettingsRect(Rect wholeWidthRect)
            {
                wholeWidthRect.xMin += (kPreviewWidth + 3);
                wholeWidthRect.width -= 4;
                return wholeWidthRect;
            }

            static Rect GetSettingsRect()
            {
                return GetSettingsRect(GUILayoutUtility.GetRect(10, kLineHeight));
            }

            static Rect GetPreviewRect(Rect wholeWidthRect)
            {
                wholeWidthRect.width = kPreviewWidth - 9;
                wholeWidthRect.x += 5;
                wholeWidthRect.height = kLineHeight;
                return wholeWidthRect;
            }

            void HeaderGUI(string h1, string h2)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(h1, styles.title, GUILayout.Width(kPreviewWidth));
                GUILayout.Label(h2, styles.title, GUILayout.ExpandWidth(true));
                GUILayout.EndHorizontal();
            }

            static float PreviewSlider(Rect position, float val)
            {
                val = GUI.HorizontalSlider(GetPreviewRect(position), val, -1, 1);
                if (val > -0.1f && val < 0.1f)
                    val = 0;
                return val;
            }

            protected AvatarSetupTool.BoneWrapper[] m_Bones;

            internal void ResetValuesFromProperties()
            {
                m_ArmTwistFactor = m_ArmTwistProperty.floatValue;
                m_ForeArmTwistFactor = m_ForeArmTwistProperty.floatValue;
                m_UpperLegTwistFactor = m_UpperLegTwistProperty.floatValue;
                m_LegTwistFactor = m_LegTwistProperty.floatValue;
                m_ArmStretchFactor = m_ArmStretchProperty.floatValue;
                m_LegStretchFactor = m_LegStretchProperty.floatValue;
                m_FeetSpacingFactor = m_FeetSpacingProperty.floatValue;
                m_HasTranslationDoF = m_HasTranslationDoFProperty.boolValue;

                // limit is a special case, because they are added dynamicly by the editor
                // all the default value are wrong, we must explictly query mecanim to get the default value when
                // m_Modified is set to false.
                for (int i = 0; i < m_Bones.Length; i++)
                {
                    if (m_Modified[i] != null)
                    {
                        bool modified = m_Modified[i].boolValue;

                        int dx = HumanTrait.MuscleFromBone(i, 0);
                        int dy = HumanTrait.MuscleFromBone(i, 1);
                        int dz = HumanTrait.MuscleFromBone(i, 2);
                        if (dx != -1)
                        {
                            m_MuscleMinEdit[dx] = modified ? m_MuscleMin[dx].floatValue : HumanTrait.GetMuscleDefaultMin(dx);
                            m_MuscleMaxEdit[dx] = modified ? m_MuscleMax[dx].floatValue : HumanTrait.GetMuscleDefaultMax(dx);
                        }

                        if (dy != -1)
                        {
                            m_MuscleMinEdit[dy] = modified ? m_MuscleMin[dy].floatValue : HumanTrait.GetMuscleDefaultMin(dy);
                            m_MuscleMaxEdit[dy] = modified ? m_MuscleMax[dy].floatValue : HumanTrait.GetMuscleDefaultMax(dy);
                        }

                        if (dz != -1)
                        {
                            m_MuscleMinEdit[dz] = modified ? m_MuscleMin[dz].floatValue : HumanTrait.GetMuscleDefaultMin(dz);
                            m_MuscleMaxEdit[dz] = modified ? m_MuscleMax[dz].floatValue : HumanTrait.GetMuscleDefaultMax(dz);
                        }
                    }
                }
            }

            internal void InitializeProperties()
            {
                m_ArmTwistProperty = serializedObject.FindProperty(sArmTwist);
                m_ForeArmTwistProperty = serializedObject.FindProperty(sForeArmTwist);
                m_UpperLegTwistProperty = serializedObject.FindProperty(sUpperLegTwist);
                m_LegTwistProperty = serializedObject.FindProperty(sLegTwist);
                m_ArmStretchProperty = serializedObject.FindProperty(sArmStretch);
                m_LegStretchProperty = serializedObject.FindProperty(sLegStretch);
                m_FeetSpacingProperty = serializedObject.FindProperty(sFeetSpacing);
                m_HasTranslationDoFProperty = serializedObject.FindProperty(sHasTranslationDoF);
                m_HumanBoneArrayProperty = serializedObject.FindProperty(sHumanBoneArray);

                for (int i = 0; i < m_Bones.Length; i++)
                {
                    SerializedProperty bone = m_Bones[i].GetSerializedProperty(m_HumanBoneArrayProperty, false);
                    if (bone != null)
                    {
                        m_Modified[i] = bone.FindPropertyRelative(sModified);

                        int dx = HumanTrait.MuscleFromBone(i, 0);
                        int dy = HumanTrait.MuscleFromBone(i, 1);
                        int dz = HumanTrait.MuscleFromBone(i, 2);


                        if (dx != -1)
                        {
                            m_MuscleMin[dx] = bone.FindPropertyRelative(sMinX);
                            m_MuscleMax[dx] = bone.FindPropertyRelative(sMaxX);
                        }

                        if (dy != -1)
                        {
                            m_MuscleMin[dy] = bone.FindPropertyRelative(sMinY);
                            m_MuscleMax[dy] = bone.FindPropertyRelative(sMaxY);
                        }

                        if (dz != -1)
                        {
                            m_MuscleMin[dz] = bone.FindPropertyRelative(sMinZ);
                            m_MuscleMax[dz] = bone.FindPropertyRelative(sMaxZ);
                        }
                    }
                }
            }

            internal void Initialize()
            {
                // initialize the necessary serialized property
                if (m_HumanBoneArrayProperty == null)
                    m_HumanBoneArrayProperty = serializedObject.FindProperty(sHumanBoneArray);

                // Handle human bones
                if (m_Bones == null)
                    m_Bones = AvatarSetupTool.GetHumanBones(m_HumanBoneArrayProperty, modelBones);

                m_FocusedMuscle = -1;

                m_MuscleBodyGroupToggle = new bool[m_Muscles.Length];
                for (int i = 0; i < m_Muscles.Length; i++)
                {
                    m_MuscleBodyGroupToggle[i] = false;
                }

                m_MuscleName = HumanTrait.MuscleName;
                for (int i = 0; i < m_MuscleName.Length; i++)
                {
                    if (m_MuscleName[i].StartsWith("Right"))
                        m_MuscleName[i] = m_MuscleName[i].Substring(5).Trim();
                    if (m_MuscleName[i].StartsWith("Left"))
                        m_MuscleName[i] = m_MuscleName[i].Substring(4).Trim();
                }
                m_MuscleCount = HumanTrait.MuscleCount;

                m_MuscleToggle = new bool[m_MuscleCount];
                m_MuscleValue = new float[m_MuscleCount];
                m_MuscleMin = new SerializedProperty[m_MuscleCount];
                m_MuscleMax = new SerializedProperty[m_MuscleCount];

                m_MuscleMinEdit = new float[m_MuscleCount];
                m_MuscleMaxEdit = new float[m_MuscleCount];

                for (int i = 0; i < m_MuscleCount; i++)
                {
                    m_MuscleToggle[i] = false;
                    m_MuscleValue[i] = 0;
                    m_MuscleMin[i] = null;
                    m_MuscleMin[i] = null;
                }

                m_Modified = new SerializedProperty[m_Bones.Length];
                for (int i = 0; i < m_Bones.Length; i++)
                {
                    m_Modified[i] = null;
                }

                InitializeProperties();
                ResetValuesFromProperties();

                m_MuscleMasterValue = new float[m_MasterMuscle.Length];
                for (int i = 0; i < m_MasterMuscle.Length; i++)
                {
                    m_MuscleMasterValue[i] = 0;
                }
            }

            public override void Enable(AvatarEditor inspector)
            {
                base.Enable(inspector);

                Initialize();

                WritePose();
            }

            public override void OnInspectorGUI()
            {
                if (Event.current.type == EventType.ValidateCommand && Event.current.commandName == EventCommandNames.UndoRedoPerformed)
                {
                    WritePose();
                }

                if (avatarAsset == null)
                {
                    return;
                }

                using (new EditorGUI.DisabledScope(!avatarAsset.isHuman))
                {
                    EditorGUIUtility.labelWidth = 110;
                    EditorGUIUtility.fieldWidth = 40;

                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();

                    MuscleGroupGUI();
                    EditorGUILayout.Space();
                    MuscleGUI();
                    EditorGUILayout.Space();
                    PropertiesGUI();

                    GUILayout.EndVertical();
                    GUILayout.Space(1);
                    GUILayout.EndHorizontal();

                    DisplayMuscleButtons();

                    ApplyRevertGUI();
                }
            }

            protected void DisplayMuscleButtons()
            {
                GUILayout.BeginHorizontal("", styles.toolbar, GUILayout.ExpandWidth(true));
                {
                    Rect r;

                    // Muscle
                    r = GUILayoutUtility.GetRect(styles.muscle, styles.toolbarDropDown);
                    if (GUI.Button(r, styles.muscle, styles.toolbarDropDown))
                    {
                        GenericMenu menu = new GenericMenu();
                        menu.AddItem(styles.resetMuscle, false, ResetMuscleToDefault);
                        menu.DropDown(r);
                    }

                    GUILayout.FlexibleSpace();
                }
                GUILayout.EndHorizontal();
            }

            protected override void ResetValues()
            {
                serializedObject.Update();
                ResetValuesFromProperties();
            }

            protected void ResetMuscleToDefault()
            {
                Avatar avatar = null;
                // For live update, update instanciate avatar to adjust pose
                if (gameObject != null)
                {
                    Animator animator = gameObject.GetComponent(typeof(Animator)) as Animator;
                    avatar = animator.avatar;
                }

                for (int i = 0; i < m_MuscleCount; i++)
                {
                    float min = HumanTrait.GetMuscleDefaultMin(i);
                    float max = HumanTrait.GetMuscleDefaultMax(i);

                    if (m_MuscleMin[i] != null && m_MuscleMax[i] != null)
                    {
                        m_MuscleMin[i].floatValue = m_MuscleMinEdit[i] = min;
                        m_MuscleMax[i].floatValue = m_MuscleMaxEdit[i] = max;
                    }

                    int humanId = HumanTrait.BoneFromMuscle(i);
                    if (m_Modified[humanId] != null && humanId != -1)
                        m_Modified[humanId].boolValue = false;

                    if (avatar != null)
                        avatar.SetMuscleMinMax(i, min, max);
                }

                WritePose();
            }

            protected void UpdateAvatarParameter(HumanParameter parameterId, float value)
            {
                // For live update, update instanciate avatar to adjust pose
                if (gameObject != null)
                {
                    Animator animator = gameObject.GetComponent(typeof(Animator)) as Animator;
                    Avatar avatar = animator.avatar;
                    avatar.SetParameter((int)parameterId, value);
                }
            }

            protected bool UpdateMuscle(int muscleId, float min, float max)
            {
                Undo.RegisterCompleteObjectUndo(this, "Updated muscle range");
                m_MuscleMin[muscleId].floatValue = min;
                m_MuscleMax[muscleId].floatValue = max;

                int humanId = HumanTrait.BoneFromMuscle(muscleId);
                if (humanId != -1)
                {
                    if (!m_Modified[humanId].boolValue)
                    {
                        int mx = HumanTrait.MuscleFromBone(humanId, 0);
                        int my = HumanTrait.MuscleFromBone(humanId, 1);
                        int mz = HumanTrait.MuscleFromBone(humanId, 2);

                        if (mx != -1 && mx != muscleId)
                        {
                            m_MuscleMin[mx].floatValue = HumanTrait.GetMuscleDefaultMin(mx);
                            m_MuscleMax[mx].floatValue = HumanTrait.GetMuscleDefaultMax(mx);
                        }

                        if (my != -1 && my != muscleId)
                        {
                            m_MuscleMin[my].floatValue = HumanTrait.GetMuscleDefaultMin(my);
                            m_MuscleMax[my].floatValue = HumanTrait.GetMuscleDefaultMax(my);
                        }

                        if (mz != -1 && mz != muscleId)
                        {
                            m_MuscleMin[mz].floatValue = HumanTrait.GetMuscleDefaultMin(mz);
                            m_MuscleMax[mz].floatValue = HumanTrait.GetMuscleDefaultMax(mz);
                        }
                    }

                    m_Modified[humanId].boolValue = true;
                }

                // OnSceneGUI need focused muscle to know which one to draw
                m_FocusedMuscle = muscleId;

                // For live update, update instanciate avatar to adjust pose
                if (gameObject != null)
                {
                    Animator animator = gameObject.GetComponent(typeof(Animator)) as Animator;
                    Avatar avatar = animator.avatar;
                    avatar.SetMuscleMinMax(muscleId, min, max);
                }

                // Need to repaint scene to update muscle range handle
                SceneView.RepaintAll();

                return gameObject != null;
            }

            protected void MuscleGroupGUI()
            {
                bool recomputePose = false;

                HeaderGUI("Preview", "Muscle Group Preview");
                GUILayout.BeginVertical(styles.box);
                {
                    {
                        Rect r = GUILayoutUtility.GetRect(10, kLineHeight);
                        Rect settingsRect = GetSettingsRect(r);
                        Rect previewRect = GetPreviewRect(r);
                        if (GUI.Button(previewRect, "Reset All", EditorStyles.miniButton))
                        {
                            for (int i = 0; i < m_MuscleMasterValue.Length; i++)
                                m_MuscleMasterValue[i] = 0;
                            for (int i = 0; i < m_MuscleValue.Length; i++)
                                m_MuscleValue[i] = 0;
                            recomputePose = true;
                        }

                        GUI.Label(settingsRect, "Reset All Preview Values", EditorStyles.label);
                    }

                    for (int i = 0; i < m_MasterMuscle.Length; i++)
                    {
                        Rect r = GUILayoutUtility.GetRect(10, kLineHeight);
                        Rect settingsRect = GetSettingsRect(r);

                        float oldValue = m_MuscleMasterValue[i];
                        m_MuscleMasterValue[i] = PreviewSlider(r, m_MuscleMasterValue[i]);
                        if (m_MuscleMasterValue[i] != oldValue)
                        {
                            Undo.RegisterCompleteObjectUndo(this, "Muscle preview");
                            for (int j = 0; j < m_MasterMuscle[i].Length; j++)
                            {
                                if (m_MasterMuscle[i][j] != -1)
                                {
                                    m_MuscleValue[m_MasterMuscle[i][j]] = m_MuscleMasterValue[i];
                                }
                            }
                        }
                        // Muscle value changed and we do have a game object to update
                        recomputePose |= m_MuscleMasterValue[i] != oldValue && gameObject != null;

                        GUI.Label(settingsRect, styles.muscleTypeGroup[i], EditorStyles.label);
                    }
                }
                GUILayout.EndVertical();

                if (recomputePose)
                    WritePose();
            }

            protected void MuscleGUI()
            {
                bool recomputePose = false;

                HeaderGUI("Preview", "Per-Muscle Settings");
                GUILayout.BeginVertical(styles.box);
                {
                    Rect r, settingsRect;
                    const int indentPerLevel = 15;
                    for (int i = 0; i < m_MuscleBodyGroupToggle.Length; i++)
                    {
                        r = GUILayoutUtility.GetRect(10, kLineHeight);
                        settingsRect = GetSettingsRect(r);
                        m_MuscleBodyGroupToggle[i] = GUI.Toggle(settingsRect, m_MuscleBodyGroupToggle[i], styles.muscleBodyGroup[i], EditorStyles.foldout);
                        if (m_MuscleBodyGroupToggle[i])
                        {
                            for (int j = 0; j < m_Muscles[i].Length; j++)
                            {
                                int muscleId = m_Muscles[i][j];
                                // Some muscle can be optionnal like Toes, if this bone is not characterized you can't edit this muscle
                                if (muscleId != -1 && m_MuscleMin[muscleId] != null && m_MuscleMax[muscleId] != null)
                                {
                                    bool expanded = m_MuscleToggle[muscleId];

                                    r = GUILayoutUtility.GetRect(10, expanded ? kLineHeight * 2 : kLineHeight);
                                    settingsRect = GetSettingsRect(r);
                                    settingsRect.xMin += indentPerLevel;

                                    // Foldout
                                    settingsRect.height = kLineHeight;
                                    m_MuscleToggle[muscleId] = GUI.Toggle(settingsRect, m_MuscleToggle[muscleId], m_MuscleName[muscleId], EditorStyles.foldout);

                                    // Preview slider
                                    float value = PreviewSlider(r, m_MuscleValue[muscleId]);
                                    // OnSceneGUI need focused muscle to know which one to draw
                                    if (m_MuscleValue[muscleId] != value)
                                    {
                                        Undo.RegisterCompleteObjectUndo(this, "Muscle preview");
                                        m_FocusedMuscle = muscleId;
                                        m_MuscleValue[muscleId] = value;
                                        recomputePose |= (gameObject != null);
                                    }

                                    if (expanded)
                                    {
                                        bool muscleChanged = false;

                                        settingsRect.xMin += indentPerLevel;
                                        settingsRect.y += kLineHeight;

                                        Rect sliderRect = settingsRect;

                                        if (settingsRect.width > 160)
                                        {
                                            Rect numberRect = settingsRect;
                                            numberRect.width = kNumberWidth;

                                            EditorGUI.BeginChangeCheck();
                                            m_MuscleMinEdit[muscleId] = EditorGUI.FloatField(numberRect, m_MuscleMinEdit[muscleId]);
                                            muscleChanged |= EditorGUI.EndChangeCheck();

                                            numberRect.x = settingsRect.xMax - kNumberWidth;

                                            EditorGUI.BeginChangeCheck();
                                            m_MuscleMaxEdit[muscleId] = EditorGUI.FloatField(numberRect, m_MuscleMaxEdit[muscleId]);
                                            muscleChanged |= EditorGUI.EndChangeCheck();

                                            sliderRect.xMin += (kNumberWidth + 5);
                                            sliderRect.xMax -= (kNumberWidth + 5);
                                        }

                                        EditorGUI.BeginChangeCheck();
                                        EditorGUI.MinMaxSlider(sliderRect, ref m_MuscleMinEdit[muscleId], ref m_MuscleMaxEdit[muscleId], sMuscleMin, sMuscleMax);
                                        muscleChanged |= EditorGUI.EndChangeCheck();

                                        if (muscleChanged)
                                        {
                                            m_MuscleMinEdit[muscleId] = Mathf.Clamp(m_MuscleMinEdit[muscleId], sMuscleMin, 0);
                                            m_MuscleMaxEdit[muscleId] = Mathf.Clamp(m_MuscleMaxEdit[muscleId], 0, sMuscleMax);
                                            recomputePose |= UpdateMuscle(muscleId, m_MuscleMinEdit[muscleId], m_MuscleMaxEdit[muscleId]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                GUILayout.EndVertical();

                if (recomputePose)
                    WritePose();
            }

            protected void PropertiesGUI()
            {
                bool recomputePose = false;

                HeaderGUI("", "Additional Settings");
                GUILayout.BeginVertical(styles.box);
                {
                    m_ArmTwistFactor = EditorGUI.Slider(GetSettingsRect(), styles.armTwist, m_ArmTwistFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_ArmTwistProperty.floatValue != m_ArmTwistFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Upper arm twist");
                        m_ArmTwistProperty.floatValue = m_ArmTwistFactor;
                        UpdateAvatarParameter(HumanParameter.UpperArmTwist, m_ArmTwistFactor);
                        recomputePose = true;
                    }

                    m_ForeArmTwistFactor = EditorGUI.Slider(GetSettingsRect(), styles.foreArmTwist, m_ForeArmTwistFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_ForeArmTwistProperty.floatValue != m_ForeArmTwistFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Lower arm twist");
                        m_ForeArmTwistProperty.floatValue = m_ForeArmTwistFactor;
                        UpdateAvatarParameter(HumanParameter.LowerArmTwist, m_ForeArmTwistFactor);
                        recomputePose = true;
                    }

                    m_UpperLegTwistFactor = EditorGUI.Slider(GetSettingsRect(), styles.upperLegTwist, m_UpperLegTwistFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_UpperLegTwistProperty.floatValue != m_UpperLegTwistFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Upper leg twist");
                        m_UpperLegTwistProperty.floatValue = m_UpperLegTwistFactor;
                        UpdateAvatarParameter(HumanParameter.UpperLegTwist, m_UpperLegTwistFactor);
                        recomputePose = true;
                    }

                    m_LegTwistFactor = EditorGUI.Slider(GetSettingsRect(), styles.legTwist, m_LegTwistFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_LegTwistProperty.floatValue != m_LegTwistFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Lower leg twist");
                        m_LegTwistProperty.floatValue = m_LegTwistFactor;
                        UpdateAvatarParameter(HumanParameter.LowerLegTwist, m_LegTwistFactor);
                        recomputePose = true;
                    }

                    m_ArmStretchFactor = EditorGUI.Slider(GetSettingsRect(), styles.armStretch, m_ArmStretchFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_ArmStretchProperty.floatValue != m_ArmStretchFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Arm stretch");
                        m_ArmStretchProperty.floatValue = m_ArmStretchFactor;
                        UpdateAvatarParameter(HumanParameter.ArmStretch, m_ArmStretchFactor);
                        recomputePose = true;
                    }

                    m_LegStretchFactor = EditorGUI.Slider(GetSettingsRect(), styles.legStretch, m_LegStretchFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_LegStretchProperty.floatValue != m_LegStretchFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Leg stretch");
                        m_LegStretchProperty.floatValue = m_LegStretchFactor;
                        UpdateAvatarParameter(HumanParameter.LegStretch, m_LegStretchFactor);
                        recomputePose = true;
                    }

                    m_FeetSpacingFactor = EditorGUI.Slider(GetSettingsRect(), styles.feetSpacing, m_FeetSpacingFactor, 0, 1,
                        EditorStyles.miniTextField);
                    if (m_FeetSpacingProperty.floatValue != m_FeetSpacingFactor)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Feet spacing");
                        m_FeetSpacingProperty.floatValue = m_FeetSpacingFactor;
                        UpdateAvatarParameter(HumanParameter.FeetSpacing, m_FeetSpacingFactor);
                        recomputePose = true;
                    }

                    m_HasTranslationDoF = EditorGUI.Toggle(GetSettingsRect(), styles.hasTranslationDoF, m_HasTranslationDoF);
                    if (m_HasTranslationDoFProperty.boolValue != m_HasTranslationDoF)
                    {
                        Undo.RegisterCompleteObjectUndo(this, "Translation DoF");
                        m_HasTranslationDoFProperty.boolValue = m_HasTranslationDoF;
                    }
                }
                GUILayout.EndVertical();

                if (recomputePose)
                    WritePose();
            }

            protected void WritePose()
            {
                if (gameObject)
                {
                    Animator animator = gameObject.GetComponent(typeof(Animator)) as Animator;
                    if (animator != null)
                    {
                        Avatar avatar = animator.avatar;
                        if (avatar != null && avatar.isValid && avatar.isHuman)
                        {
                            AvatarUtility.SetHumanPose(animator, m_MuscleValue);
                            SceneView.RepaintAll();
                        }
                    }
                }
            }

            public void DrawMuscleHandle(Transform t, int humanId)
            {
                Animator animator = gameObject.GetComponent(typeof(Animator)) as Animator;
                Avatar avatar = animator.avatar;

                int mx = HumanTrait.MuscleFromBone(humanId, 0);
                int my = HumanTrait.MuscleFromBone(humanId, 1);
                int mz = HumanTrait.MuscleFromBone(humanId, 2);

                float axisLen = avatar.GetAxisLength(humanId);
                Quaternion preQ = avatar.GetPreRotation(humanId);
                Quaternion postQ = avatar.GetPostRotation(humanId);

                preQ = t.parent.rotation * preQ;
                postQ = t.rotation * postQ;

                Vector3 normal;
                Vector3 from;

                Color alpha = new Color(1, 1, 1, 0.5f);
                Quaternion zyRoll = avatar.GetZYRoll(humanId, Vector3.zero);
                Vector3 sign = avatar.GetLimitSign(humanId);

                // Draw axis
                normal = postQ * Vector3.right;
                Vector3 axisEnd = t.position + (normal * axisLen);
                Handles.color = Color.white;
                Handles.DrawLine(t.position, axisEnd);

                if (mx != -1)
                {
                    Quaternion zyPostQ = avatar.GetZYPostQ(humanId, t.parent.rotation, t.rotation);

                    float minx = m_MuscleMinEdit[mx];
                    float maxx = m_MuscleMaxEdit[mx];

                    normal = postQ * Vector3.right;
                    from = zyPostQ * Vector3.forward;

                    Handles.color = Color.black;
                    //Handles.DrawLine (t.position, t.position + (from * axisLen * 0.75f));

                    Vector3 xDoF = t.position + (normal * axisLen * 0.75f);

                    normal = postQ * Vector3.right * sign.x;
                    Quaternion q = Quaternion.AngleAxis(minx, normal);
                    from = q * from;

                    Handles.color = Color.yellow;
                    //Handles.DrawLine (t.position, t.position + (from * axisLen * 0.75f));

                    // Draw Muscle range
                    Handles.color = Handles.xAxisColor * alpha;
                    Handles.DrawSolidArc(xDoF, normal, from, maxx - minx, axisLen * 0.25f);

                    from = postQ * Vector3.forward;
                    Handles.color = Handles.centerColor;
                    Handles.DrawLine(xDoF, xDoF + (from * axisLen * 0.25f));
                }

                if (my != -1)
                {
                    float miny = m_MuscleMinEdit[my];
                    float maxy = m_MuscleMaxEdit[my];

                    normal = preQ * Vector3.up * sign.y;
                    from = preQ * zyRoll * Vector3.right;

                    Handles.color = Color.black;
                    //Handles.DrawLine (t.position, t.position + (from * axisLen * 0.75f));

                    Quaternion q = Quaternion.AngleAxis(miny, normal);
                    from = q * from;

                    Handles.color = Color.yellow;
                    //Handles.DrawLine (t.position, t.position + (from * axisLen * 0.75f));

                    // Draw Muscle range
                    Handles.color = Handles.yAxisColor * alpha;
                    Handles.DrawSolidArc(t.position, normal, from, maxy - miny, axisLen * 0.25f);
                }
                if (mz != -1)
                {
                    float minz = m_MuscleMinEdit[mz];
                    float maxz = m_MuscleMaxEdit[mz];

                    normal = preQ * Vector3.forward * sign.z;
                    from = preQ * zyRoll * Vector3.right;

                    Handles.color = Color.black;
                    //Handles.DrawLine (t.position, t.position + (from * axisLen * 0.75f));

                    Quaternion q = Quaternion.AngleAxis(minz, normal);
                    from = q * from;

                    Handles.color = Color.yellow;
                    //Handles.DrawLine (t.position, t.position + (from * axisLen * 0.75f));

                    // Draw Muscle range
                    Handles.color = Handles.zAxisColor * alpha;
                    Handles.DrawSolidArc(t.position, normal, from, maxz - minz, axisLen * 0.25f);
                }
            }

            public override void OnSceneGUI()
            {
                AvatarSkeletonDrawer.DrawSkeleton(root, modelBones, m_BoneRenderer);

                if (gameObject == null)
                    return;

                Animator animator = gameObject.GetComponent(typeof(Animator)) as Animator;
                if (m_FocusedMuscle == -1 || animator == null)
                    return;

                int humanId = HumanTrait.BoneFromMuscle(m_FocusedMuscle);
                if (humanId != -1)
                {
                    DrawMuscleHandle(m_Bones[humanId].bone, humanId);
                }
            }
        }
    */

/// <summary>
/// Remove empty element or sort something 
/// </summary>
/*        private void Regular()
        {
            List<MMJoint> vaildJoint = new List<MMJoint>();
            for (int i = 0; i < motionJoints.Length; i++)
            {
                if (motionJoints[i] != null)
                {
                    motionJoints[i].jointIndex = vaildJoint.Count;
                    vaildJoint.Add(motionJoints[i]);
                }
            }
            motionJoints = vaildJoint.ToArray();

            List<MMMuscle> vaildmotionMuscle = new List<MMMuscle>();
            for (int i = 0; i < motionMuscles.Length; i++)
            {
                if (motionMuscles[i] != null)
                {
                    motionMuscles[i].muscleIndex = vaildmotionMuscle.Count;
                    vaildmotionMuscle.Add(motionMuscles[i]);
                }
            }
            motionMuscles = vaildmotionMuscle.ToArray();
        }*/
/*            #region Load muscle's Range
            //OYM£ºload 3 import range if not useDefaultValues
            var humanbone = humandescription.human;
            for (int i = 0; i < humanbone.Length; i++)
            {
                var jointDescription = humanbone[i];
                int boneIndex = Array.IndexOf(HumanTrait.BoneName, jointDescription.humanName);
                if (!jointDescription.limit.useDefaultValues)
                {
                    var motionJoint = motionJoints[boneIndex];
                    motionJoint.minRange = jointDescription.limit.min;
                    motionJoint.maxRange = jointDescription.limit.max;
                }

            }
            #endregion*/

/*            currentPose.muscles = new float[muscleValue.Length];
            humanPoseHandler.SetHumanPose(ref currentPose);*/
#endregion

