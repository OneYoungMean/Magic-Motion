using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion.Mono
{
    /*
    public class AnimatorTreeNode
    {
        public Transform transform;
        public List<AnimatorTreeNode> childs;
        public AnimatorTreeNode parent;

        public MagicMotionJoint geneticJoint;
        public bool isVaild => transform != null;
        public AnimatorTreeNode(Transform target)
        {
            this.transform = target;
            childs = new List<AnimatorTreeNode>(); //ºÏ≤‚µΩ‘Ÿº”‘ÿ
        }
        public override string ToString()
        {
            return transform.ToString();
        }
    }
    public class MagicMotionJointTree : MonoBehaviour
    {
        #region StaticField
        public static readonly float3 BodyLowerLimit = new float3(-40, -40, -40);
        public static readonly float3 BodyUpperLimit = new float3(40, 40, 40);

        public static readonly float3 HeadLowerLimit = new float3(-40, -40, -40);
        public static readonly float3 HeadUpperLimit = new float3(40, 40, 40);

        public static readonly float3 EyeLowerLimit = new float3(-15, -20, 0);
        public static readonly float3 EyeUpperLimit = new float3(10, 20, 0);

        public static readonly float3 UpperLegLowerLimit = new float3(-60, -90, -60);
        public static readonly float3 UpperLegUpperLimit = new float3(60, 50, 60);

        public static readonly float3 LowerLegLowerLimit = new float3(-80, -90, -0);
        public static readonly float3 LowerLegUpperLimit = new float3(80, 90, 0);

        public static readonly float3 FootLowerLimit = new float3(-50, -0, -30);
        public static readonly float3 FootUpperLimit = new float3(50, 0, 30);

        public static readonly float3 ShoulderLowerLimit = new float3(-0, -15, -30);
        public static readonly float3 ShoulderUpperLimit = new float3(0, 15, 30);

*//*        public static readonly Dictionary<string,float3x2> HumanBodyBoneNames = new Dictionary<string, float3x2>()
        {
            { "Root", float3x2.zero},
           { "Hips",float3x2.zero},
           {"LeftUpperLeg",new float3x2(UpperLegLowerLimit,UpperLegUpperLimit)},
           {"RightUpperLeg",new float3x2(UpperLegLowerLimit,UpperLegUpperLimit)},

           {"LeftLowerLeg",new float3x2(LowerLegLowerLimit,LowerLegUpperLimit)},
           {"RightLowerLeg",new float3x2(LowerLegLowerLimit,LowerLegUpperLimit)},

           {"LeftFoot",new float3x2(FootLowerLimit,FootUpperLimit)},
           {"RightFoot",new float3x2(FootLowerLimit,FootUpperLimit)},

           {"Spine",new float3x2(BodyLowerLimit,BodyUpperLimit)},
           {"Chest",new float3x2(BodyLowerLimit,BodyUpperLimit)},

           {"Neck",new float3x2(HeadLowerLimit,HeadUpperLimit)},
           {"Head",new float3x2(HeadLowerLimit,HeadUpperLimit)},

           {"LeftShoulder",},
           {"RightShoulder",},

           {"LeftUpperArm",},
           {"RightUpperArm",},
           {"LeftLowerArm",},
           {"RightLowerArm",},
           {"LeftHand",},
           {"RightHand",},
           {"LeftToes",},
           {"RightToes",},
           {"LeftEye",},
           {"RightEye",},
           {"Jaw",},
           {"LeftThumbProximal",},
           {"LeftThumbIntermediate",},
           {"LeftThumbDistal",},
           {"LeftIndexProximal",},
           {"LeftIndexIntermediate",},
           {"LeftIndexDistal",},
           {"LeftMiddleProximal",},
           {"LeftMiddleIntermediate",},
           {"LeftMiddleDistal",},
           {"LeftRingProximal",},
           {"LeftRingIntermediate",},
           {"LeftRingDistal",},
           {"LeftLittleProximal",},
           {"LeftLittleIntermediate",},
           {"LeftLittleDistal",},
           {"RightThumbProximal",},
           {"RightThumbIntermediate",},
           {"RightThumbDistal",},
           {"RightIndexProximal",},
           {"RightIndexIntermediate",},
           {"RightIndexDistal",},
           {"RightMiddleProximal",},
           {"RightMiddleIntermediate",},
           {"RightMiddleDistal",},
           {"RightRingProximal",},
           {"RightRingIntermediate",},
           {"RightRingDistal",},
           {"RightLittleProximal",},
           {"RightLittleIntermediate",},
           {"RightLittleDistal",},
           {"UpperChest",},
    };*//*
        #endregion
        #region  Field&Property
        public bool isInitial;
        public Animator animator;
        public List<AnimatorTreeNode> allVaildNode;

        private List<MagicMotionJoint> allVaildJoint;
        #endregion

        #region UnityFunc
        #endregion

        #region LocalFunc
        #endregion

        #region StaticFunc
        #endregion









        //public AnimatorTreeNode LastBone; unexist;
        public void Start()
        {
            Initialize();
        }

        public void Initialize()
        {
            if (animator == null || !animator.isHuman)
            {
                Debug.LogError("Cant be a Human bone");
                return;
            }
            if (isInitial)
            {
                return;
            }

            GetAllHumanBone();
            CollectVaildHumanBone();
            BuildAnimatorTree();
            BuildGeneticJoint();
            isInitial = true;
        }

        void GetAllHumanBone()
        {
            HumanDescription description = animator.avatar.humanDescription;
            Root = new AnimatorTreeNode(animator.transform);
            Hips = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.Hips));
            LeftUpperLeg = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg));
            RightUpperLeg = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightUpperLeg));
            LeftLowerLeg = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg));
            RightLowerLeg = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightLowerLeg));
            LeftFoot = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftFoot));
            RightFoot = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightFoot));
            Spine = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.Spine));
            Chest = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.Chest));
            Neck = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.Neck));
            Head = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.Head));
            LeftShoulder = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftShoulder));
            RightShoulder = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightShoulder));
            LeftUpperArm = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftUpperArm));
            RightUpperArm = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightUpperArm));
            LeftLowerArm = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftLowerArm));
            RightLowerArm = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightLowerArm));
            LeftHand = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftHand));
            RightHand = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightHand));
            LeftToes = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftToes));
            RightToes = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightToes));
            LeftEye = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftEye));
            RightEye = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightEye));
            Jaw = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.Jaw));
            LeftThumbProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftThumbProximal));
            LeftThumbIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftThumbIntermediate));
            LeftThumbDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftThumbDistal));
            LeftIndexProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftIndexProximal));
            LeftIndexIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftIndexIntermediate));
            LeftIndexDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftIndexDistal));
            LeftMiddleProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftMiddleProximal));
            LeftMiddleIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftMiddleIntermediate));
            LeftMiddleDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftMiddleDistal));
            LeftRingProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftRingProximal));
            LeftRingIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftRingIntermediate));
            LeftRingDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftRingDistal));
            LeftLittleProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftLittleProximal));
            LeftLittleIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftLittleIntermediate));
            LeftLittleDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LeftLittleDistal));
            RightThumbProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightThumbProximal));
            RightThumbIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightThumbIntermediate));
            RightThumbDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightThumbDistal));
            RightIndexProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightIndexProximal));
            RightIndexIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightIndexIntermediate));
            RightIndexDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightIndexDistal));
            RightMiddleProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightMiddleProximal));
            RightMiddleIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightMiddleIntermediate));
            RightMiddleDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightMiddleDistal));
            RightRingProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightRingProximal));
            RightRingIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightRingIntermediate));
            RightRingDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightRingDistal));
            RightLittleProximal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightLittleProximal));
            RightLittleIntermediate = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightLittleIntermediate));
            RightLittleDistal = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.RightLittleDistal));
            UpperChest = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.UpperChest));
            //LastBone = new AnimatorTreeNode(animator.GetBoneTransform(HumanBodyBones.LastBone));
        }
        void CollectVaildHumanBone()
        {


            allVaildNode = new List<AnimatorTreeNode>();

            if (Root.isVaild) allVaildNode.Add(Root);
            if (Hips.isVaild) allVaildNode.Add(Hips);
            if (LeftUpperLeg.isVaild) allVaildNode.Add(LeftUpperLeg);
            if (RightUpperLeg.isVaild) allVaildNode.Add(RightUpperLeg);
            if (LeftLowerLeg.isVaild) allVaildNode.Add(LeftLowerLeg);
            if (RightLowerLeg.isVaild) allVaildNode.Add(RightLowerLeg);
            if (LeftFoot.isVaild) allVaildNode.Add(LeftFoot);
            if (RightFoot.isVaild) allVaildNode.Add(RightFoot);
            if (Spine.isVaild) allVaildNode.Add(Spine);
            if (Chest.isVaild) allVaildNode.Add(Chest);
            if (Neck.isVaild) allVaildNode.Add(Neck);
            if (Head.isVaild) allVaildNode.Add(Head);
            if (LeftShoulder.isVaild) allVaildNode.Add(LeftShoulder);
            if (RightShoulder.isVaild) allVaildNode.Add(RightShoulder);
            if (LeftUpperArm.isVaild) allVaildNode.Add(LeftUpperArm);
            if (RightUpperArm.isVaild) allVaildNode.Add(RightUpperArm);
            if (LeftLowerArm.isVaild) allVaildNode.Add(LeftLowerArm);
            if (RightLowerArm.isVaild) allVaildNode.Add(RightLowerArm);
            if (LeftHand.isVaild) allVaildNode.Add(LeftHand);
            if (RightHand.isVaild) allVaildNode.Add(RightHand);
            if (LeftToes.isVaild) allVaildNode.Add(LeftToes);
            if (RightToes.isVaild) allVaildNode.Add(RightToes);
            if (LeftEye.isVaild) allVaildNode.Add(LeftEye);
            if (RightEye.isVaild) allVaildNode.Add(RightEye);
            if (Jaw.isVaild) allVaildNode.Add(Jaw);
            if (LeftThumbProximal.isVaild) allVaildNode.Add(LeftThumbProximal);
            if (LeftThumbIntermediate.isVaild) allVaildNode.Add(LeftThumbIntermediate);
            if (LeftThumbDistal.isVaild) allVaildNode.Add(LeftThumbDistal);
            if (LeftIndexProximal.isVaild) allVaildNode.Add(LeftIndexProximal);
            if (LeftIndexIntermediate.isVaild) allVaildNode.Add(LeftIndexIntermediate);
            if (LeftIndexDistal.isVaild) allVaildNode.Add(LeftIndexDistal);
            if (LeftMiddleProximal.isVaild) allVaildNode.Add(LeftMiddleProximal);
            if (LeftMiddleIntermediate.isVaild) allVaildNode.Add(LeftMiddleIntermediate);
            if (LeftMiddleDistal.isVaild) allVaildNode.Add(LeftMiddleDistal);
            if (LeftRingProximal.isVaild) allVaildNode.Add(LeftRingProximal);
            if (LeftRingIntermediate.isVaild) allVaildNode.Add(LeftRingIntermediate);
            if (LeftRingDistal.isVaild) allVaildNode.Add(LeftRingDistal);
            if (LeftLittleProximal.isVaild) allVaildNode.Add(LeftLittleProximal);
            if (LeftLittleIntermediate.isVaild) allVaildNode.Add(LeftLittleIntermediate);
            if (LeftLittleDistal.isVaild) allVaildNode.Add(LeftLittleDistal);
            if (RightThumbProximal.isVaild) allVaildNode.Add(RightThumbProximal);
            if (RightThumbIntermediate.isVaild) allVaildNode.Add(RightThumbIntermediate);
            if (RightThumbDistal.isVaild) allVaildNode.Add(RightThumbDistal);
            if (RightIndexProximal.isVaild) allVaildNode.Add(RightIndexProximal);
            if (RightIndexIntermediate.isVaild) allVaildNode.Add(RightIndexIntermediate);
            if (RightIndexDistal.isVaild) allVaildNode.Add(RightIndexDistal);
            if (RightMiddleProximal.isVaild) allVaildNode.Add(RightMiddleProximal);
            if (RightMiddleIntermediate.isVaild) allVaildNode.Add(RightMiddleIntermediate);
            if (RightMiddleDistal.isVaild) allVaildNode.Add(RightMiddleDistal);
            if (RightRingProximal.isVaild) allVaildNode.Add(RightRingProximal);
            if (RightRingIntermediate.isVaild) allVaildNode.Add(RightRingIntermediate);
            if (RightRingDistal.isVaild) allVaildNode.Add(RightRingDistal);
            if (RightLittleProximal.isVaild) allVaildNode.Add(RightLittleProximal);
            if (RightLittleIntermediate.isVaild) allVaildNode.Add(RightLittleIntermediate);
            if (RightLittleDistal.isVaild) allVaildNode.Add(RightLittleDistal);
            if (UpperChest.isVaild) allVaildNode.Add(UpperChest);
            //if (LastBone.isVaild) allVaildNode.Add(LastBone);
        }

        void BuildAnimatorTree()
        {
            Dictionary<Transform, AnimatorTreeNode> searchDic = new Dictionary<Transform, AnimatorTreeNode>();
            for (int i = allVaildNode.Count - 1; i >= 0; i--)
            {
                searchDic.Add(allVaildNode[i].transform, allVaildNode[i]);
            }

            foreach (var item in searchDic)
            {
                AnimatorTreeNode childNode = item.Value;
                Transform childTarget = item.Key;

                Transform probableParent = childTarget.parent;
                while (probableParent != null)
                {
                    if (searchDic.TryGetValue(probableParent, out AnimatorTreeNode parentNode))
                    {
                        parentNode.childs.Add(childNode);
                        childNode.parent = parentNode;
                        break;
                    }
                    probableParent = probableParent.parent;
                }
            }
        }
        void BuildGeneticJoint()
        {
            //OYM:BFS
            Queue<AnimatorTreeNode> nodeQueue = new Queue<AnimatorTreeNode>();
            allVaildNode.Clear();//OYM:Resort;
            nodeQueue.Enqueue(Root);

            while (nodeQueue.Count != 0)
            {
                int index = allVaildNode.Count;

                AnimatorTreeNode curNode = nodeQueue.Dequeue();
                ref MagicMotionJoint currentJoint = ref curNode.geneticJoint;
                Vector3 currentPostion = curNode.transform.position;
                Quaternion currentRotation = curNode.transform.rotation;

                ref MagicMotionJoint parentJoint = ref curNode.parent.geneticJoint;
                parentJoint.childIndexStart = math.min(parentJoint.childIndexStart, index);
                parentJoint.childIndexEnd = math.max(parentJoint.childIndexEnd, index + 1);
                *//*                if (curNode.parent == null)//OYM: setting rootJoint
                    {
                        currentJoint.localPosition = currentPostion;
                        currentJoint.localRotation = currentRotation;
                        currentJoint.MinRange = -0f;
                        currentJoint.MaxRange = 0f;
                        currentJoint.parentIndex = -1;
                        currentJoint.childIndexStart = int.MaxValue;
                        currentJoint.childIndexEnd = int.MinValue;
                    }
                    else //OYM:setting parentChildRange
                    {

                    }*//*

                //OYM:initialchildJoint
                for (int i = 0; i < curNode.childs.Count; i++)
                {
                    var childNode = curNode.childs[i];
                    ref MagicMotionJoint childJoint = ref childNode.geneticJoint;
                    Vector3 childPostion = childNode.transform.position;
                    Quaternion childRotation = childNode.transform.rotation;

                    childJoint.localPosition = Quaternion.Inverse(currentRotation) * (childPostion - currentPostion);
                    childJoint.localRotation = childRotation * Quaternion.Inverse(currentRotation);
                    childJoint.MinRange = -30;
                    childJoint.MaxRange = 30;
                    childJoint.parentIndex = index;
                    childJoint.childIndexStart = int.MaxValue;
                    childJoint.childIndexEnd = int.MinValue;

                    nodeQueue.Enqueue(childNode);
                }

                allVaildNode.Add(curNode);//OYM: re-add in list
            }
        }

        public MagicMotionJoint[] ToGeneticJointArray()
        {
            if (!isInitial)
            {
                return null;
            }
            else
            {
                return allVaildNode.Select(x => x.geneticJoint).ToArray();
            }


        }

        internal Transform[] ToTransformArray()
        {
            if (!isInitial)
            {
                return null;
            }
            else
            {
                return allVaildNode.Select(x => x.transform).ToArray();
            }
        }


    }
    */
}
