using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public enum BioJointType
    {
        Rotational, Translational
    }

    public class BioJoint : MonoBehaviour
    {
        public BioSegment segment;//OYM:挂载的Segment
        public BioJointType jointType = BioJointType.Rotational;
        public BioMotion bioMotion;//OYM:包含了运动的倾向
        private float3 anchor;
        private float3 orientation;
        private float3 defaultPosition;
        private quaternion defaultRotation;

        //OYM:两个意义不明的变量
        private float3 AnimationPosition, AnimatedDefaultPosition;//OYM:动作真实数据与差值过的数据
        private quaternion AnimationRotation, AnimatedDefaultRotation;//OYM:同上

        private quaternion rotationMat;
        private float3 LSA;
        private float3 ADPADRSA;
        private float3 lastPosition;
        private quaternion lastrotation;
        private float3 scale;

        private float AnimationWeight => segment.character.animationWeight;
        private MotionType motionType => segment.character.motionType;
        // Start is called before the first frame update
        void Start()
        {
            lastPosition = transform.localPosition;
            lastrotation = transform.localRotation;
        }

        // Update is called once per frame
        void Update()
        {

        }
        public void Initialize(BioSegment segment)
        {
            this.segment = segment;

            bioMotion = segment.transform.gameObject.AddComponent<BioMotion>();
            bioMotion.Create(this);
            defaultPosition = transform.localPosition;
            defaultRotation = transform.localRotation;

            Vector3 forward = Vector3.zero;
            if (segment.childs.Count == 1)
            {
                forward = segment.childs[0].transform.localPosition;
            }
            else if (segment.parent != null)
            {
                forward = Quaternion.Inverse(segment.transform.localRotation) * segment.transform.localPosition;//OYM;应该是childposion
            }

 /*           if (forward.magnitude != 0f)
            {
                SetOrientation(Quaternion.LookRotation(forward, Vector3.up).eulerAngles);
            }//这部分存疑*/

            Start();
        }

        public void PrecaptureAnimation()
        {
            transform.hasChanged = false;
        }

        public void PostcaptureAnimation()
        {
            if (transform.hasChanged)
            {
                AnimationPosition = transform.localPosition;
                AnimationRotation = transform.localRotation;
            }
            else
            {
                AnimationPosition = defaultPosition;
                AnimationRotation = defaultRotation;
            }
        }
        public void UpdateData()
        {
            AnimatedDefaultPosition = math.clamp(AnimationWeight, defaultPosition , AnimationPosition);
            AnimatedDefaultRotation = math.slerp( defaultRotation, AnimationRotation, AnimationWeight);
            rotationMat = AnimatedDefaultRotation;

            LSA = anchor * transform.localScale;
            ADPADRSA = AnimatedDefaultPosition + math.mul(AnimatedDefaultRotation, LSA);
            scale = transform.lossyScale;
        }

        public void ProcessMotion()
        {
            float3 localPosition=float3.zero;
            quaternion localRotation = quaternion.identity;

            float3 currentMotion = bioMotion.ProcessMotion(motionType);
            if (jointType==BioJointType.Rotational)
            {
                ComputeLocalRotation(currentMotion, LSA, ADPADRSA, AnimatedDefaultRotation, out localPosition, out  localRotation);

            }
            else
            {
                currentMotion /= scale;
                ComputeLocalTranslational(currentMotion, bioMotion.axisMat, AnimatedDefaultPosition, AnimatedDefaultRotation, out localPosition, out localRotation);
            }
            //Apply local transformation
            if (Application.isPlaying)
            {
                //Assigning transformation
                transform.localPosition = math.lerp(localPosition,   lastPosition, segment.character.smoothing);
                transform.localRotation = Quaternion.Slerp(localRotation, lastrotation, segment.character.smoothing);

                //Blending animation
                transform.localPosition = math.lerp(transform.localPosition, AnimationPosition, segment.character.animationBlend);
                transform.localRotation = Quaternion.Slerp(transform.localRotation, AnimationRotation, segment.character.animationBlend);
            }
            else
            {
                transform.localPosition = localPosition;
                transform.localRotation = localRotation;
            }

            //Remember transformation
            lastPosition = transform.localPosition;
            lastrotation = transform.localRotation;

            transform.hasChanged = false;
        }
        public Vector3 GetAnchorInWorldSpace()
        {
            return transform.position + transform.rotation * Vector3.Scale(transform.lossyScale, anchor);
        }

        public void SetAnchor(Vector3 anchor)
        {
            this.anchor = anchor;
        }

        public Vector3 GetAnchor()
        {
            return anchor;
        }

        public void SetOrientation(Vector3 orientation)
        {
            this.orientation = orientation;
            bioMotion.axisMat = float3x3.Euler(orientation);
        }

        public Vector3 GetOrientation()
        {
            return orientation;
        }
        public int GetDof()
        {
            return math.csum((int3)bioMotion.isEnableValue);
        }
        public void SetDefaultFrame(Vector3 localPosition, Quaternion localRotation)
        {
            defaultPosition = localPosition;
            defaultRotation = localRotation;
        }
        public void RestoreDefaultFrame()
        {
            transform.localPosition = defaultPosition;
            transform.localRotation = defaultRotation;
        }
        public void ComputeLocalTransformation(float3 currentMotion, out float3 localPosition, out quaternion localRotation)
        {
            if (jointType == BioJointType.Translational)
            {
                ComputeLocalTranslational(currentMotion, bioMotion.axisMat, AnimatedDefaultPosition, AnimatedDefaultRotation, out localPosition, out localRotation);
            }
            else
            {
                ComputeLocalRotation(currentMotion, LSA, ADPADRSA, AnimatedDefaultRotation, out localPosition, out localRotation);
            }
        }
        private static void ComputeLocalTranslational(float3 currentMotion, float3x3 axisMat ,float3 AnimatedDefaultPosition,quaternion AnimatedDefaultRotation, out float3 localPosition, out quaternion localRotation)
        {
            currentMotion = math.mul(axisMat, currentMotion);
            localPosition = AnimatedDefaultPosition + math.mul(AnimatedDefaultRotation, currentMotion);
            localRotation = AnimatedDefaultRotation;
        }

        private static void ComputeLocalRotation(float3 currentMotion, float3 LSA,float3 ADPADRSA, quaternion AnimatedDefaultRotation, out float3 localPosition, out quaternion localRotation)
        {
            localRotation = math.mul(AnimatedDefaultRotation, quaternion.Euler(currentMotion));
            localPosition = ADPADRSA + math.mul(localRotation, LSA);
        }

        internal void Erase()
        {
            RestoreDefaultFrame();
            segment.transform.hideFlags = HideFlags.None;
            Utility.Destroy(this);
        }
    }
}