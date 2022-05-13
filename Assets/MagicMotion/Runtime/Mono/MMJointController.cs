﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MMJointController:MonoBehaviour
    {

        #region  Field&Property
        /// <summary>
        /// generate joint,
        /// </summary>
        public Transform rootTransform;
        /// <summary>
        /// generate joint,
        /// </summary>
        public MMJoint[] motionJoints;
        /// <summary>
        /// generate muscle
        /// </summary>
        public MMMuscle[] motionMuscles;
        /// <summary>
        /// generate constraint
        /// </summary>
        public List<MMConstraint> motionConstraints;
        /// <summary>
        ///  optimize kernel
        /// </summary>
        public MagicMotionKernel kernel;
        /// <summary>
        /// isInitialize
        /// </summary>
        private bool isInitialize;
/*        [SerializeField]
        private int editorIndex = 0;*/
        [SerializeField]
        private AnimationCurve lossCurve = new AnimationCurve();
/*        [SerializeField]*/
/*        private AnimationCurve muscleCurve = new AnimationCurve();*/
/*        [SerializeField]*/
/*        private AnimationCurve gradientCurve = new AnimationCurve();*/
        private bool isReadyUpdate;

        private float3 worldPosition;
        private quaternion worldRotation;
        private ConstraintData[] constraintDatas;
        #endregion

        #region UnityFunc
        public void Start()
        {
            Initialize();
            RegisterData(); 
        }



        private void Update()
        {
            if (!isInitialize)
            {
                return;
            }
            UpdateData();

            kernel.Optimize(Time.deltaTime, worldPosition, worldRotation);

            //OYM：这部分我一直没想好怎么写会好一点
            //OYM：异步赋值再访问感觉怪怪的.
            if (isReadyUpdate)
            {
                UpdateMotion();
                UpdateCruve();
            }

            isReadyUpdate = false;
        }

        private void OnDestroy()
        {
            kernel?.Dispose();
        }

        #endregion

        #region LocalFunc

        /// <summary>
        /// Update motion for muscle on editor or mian thread
        /// </summary>
        public void UpdateMotion()
        {
            //OYM：rebuild sklenon
            for (int i = 0; i < motionJoints.Length; i++)
            {
                MMJoint currentJoint = motionJoints[i];
                if (currentJoint == null || !currentJoint.enabled)
                {
                    continue;
                }

                Transform jointTransform = currentJoint.transform;

                //OYM：Dof to euler angle
                float3 Dof3 = new float3(
                    currentJoint.muscles[0] == null ? 0 : currentJoint.muscles[0].value,
                    currentJoint.muscles[1] == null ? 0 : currentJoint.muscles[1].value,
                    currentJoint.muscles[2] == null ? 0 : currentJoint.muscles[2].value);

                if (math.all(Dof3 == 0))
                {
                    continue;
                }
                float3 Dof3toRadian = math.radians(
                        math.lerp(0, currentJoint.minRange, -math.clamp(Dof3, -1, 0))
                    + math.lerp(0, currentJoint.maxRange, math.clamp(Dof3, 0, 1))
                    );

                quaternion eulerAngle = quaternion.identity;
                if (Dof3[0] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[0], Dof3toRadian[0]), eulerAngle);
                }
                if (Dof3[1] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[1], Dof3toRadian[1]), eulerAngle);
                }
                if (Dof3[2] != 0)
                {
                    eulerAngle = math.mul(quaternion.AxisAngle(currentJoint.dof3Axis[2], Dof3toRadian[2]), eulerAngle);
                }

                if (currentJoint.parent == null)
                {
                    /*                    parentPosition = currentJoint.transform.position;
                                        parentRotation = currentJoint.transform.rotation;
                    */
                    jointTransform.localRotation = math.mul(currentJoint.initiallocalRotation, eulerAngle);
                }
                else
                {
                    quaternion parentRotation = currentJoint.parent.transform.rotation;
                    float3 parentPosition = currentJoint.parent.transform.position;
                    jointTransform.position = parentPosition + math.mul(parentRotation, currentJoint.initiallocalPosition);
                    jointTransform.rotation = math.mul(parentRotation, math.mul(currentJoint.initiallocalRotation, eulerAngle));
                }
            }

        }
        public void Initialize()
        {
            if (isInitialize)
            {
                return;
            }
            CheckValue();
            AddRootJointInArray();
            BuildJointRelation();
            GetMuscleAndConstraintData();
            if (Application.isPlaying)
            {
                isInitialize = true;
            }
        }

        private void SetMuscle(float[] muscleValues)
        {
            Debug.Assert(muscleValues.Length == motionMuscles.Length);
            for (int i = 0; i < motionMuscles. Length; i++)
            {
                motionMuscles[i].value = muscleValues[i];
            }
            isReadyUpdate = true;
        }

        private void CheckValue()
        {
            if (motionJoints == null || motionJoints.Length == 0)
            {
                throw new NullReferenceException("Joint List Length is zero or empty");
            }
        }

        private void AddRootJointInArray()
        {
            if (motionJoints[0].transform != rootTransform)
            {
                var rootJoint = rootTransform.GetComponent<MMJoint>() ?? rootTransform.gameObject.AddComponent<MMJoint>();
                var newArray = new MMJoint[motionJoints.Length + 1];
                Array.Copy(motionJoints, 0, newArray, 1, motionJoints.Length);
                newArray[0] = rootJoint;
                motionJoints = newArray;
            }
            motionJoints[0].jointIndex = 0;
            motionJoints[0].parent = null;
        }

        private void BuildJointRelation()
        {
            #region Clac localPosition and localRotation

            for (int i = 0; i < motionJoints.Length; i++)
            {
                var currentJoint = motionJoints[i];
                if (motionJoints[i] == null)
                {
                    continue;
                }
                currentJoint.jointIndex = i;
                currentJoint.controller = this;
                for (Transform parentTransform = currentJoint.transform.parent;
                    parentTransform != null;
                    parentTransform = parentTransform.parent)
                {
                    MMJoint parentJoint = motionJoints.FirstOrDefault(x => x.transform == parentTransform);
                    if (parentJoint != null)
                    {
                        currentJoint.parent = parentJoint;
                        break;
                    }
                }

                if (currentJoint.parent == null)
                {
                    currentJoint.initiallocalRotation = currentJoint.transform.localRotation;
                    currentJoint.initiallocalPosition = currentJoint.transform.localPosition;
                    currentJoint.length = currentJoint.cumulativeLength = 0;
                }
                else
                {
                    currentJoint.initiallocalRotation = math.mul(math.inverse(currentJoint.parent.transform.rotation), currentJoint.transform.rotation);
                    currentJoint.initiallocalPosition = math.mul(math.inverse(currentJoint.parent.transform.rotation), (float3)(currentJoint.transform.position - currentJoint.parent.transform.position));

                    currentJoint.length = math.length(currentJoint.initiallocalPosition);
                    currentJoint.cumulativeLength = currentJoint.parent.length + currentJoint.length;
                }


                motionJoints[i] = currentJoint;
            }
            #endregion
        }
        private void GetMuscleAndConstraintData()
        {
            if (motionJoints==null)
            {
                throw new InvalidOperationException("motionJoint List is Empty");
            }
            List<MMConstraint> constraints= new List<MMConstraint> ();
            for (int i = 0; i < motionJoints.Length; i++)
            {
                var joint= motionJoints[i];
                for (int ii = 0; ii < joint.constraints?.Length; ii++)
                {
                    if (joint.constraints[ii]!=null)
                    {
                        constraints.Add(motionJoints[i].constraints[ii]);
                    }
                }
            }
            this.motionConstraints = constraints;

            List<MMMuscle> muscles = new List<MMMuscle> ();
            for (int i = 0; i < motionJoints.Length; i++)
            {
                var joint = motionJoints[i];
                if (motionJoints.FirstOrDefault(x => x.parent == joint) == null &&
    (joint.constraints == null || joint.constraints[(int)MMConstraintType.LookAt] == null)
    )
                {
                    continue;
                }
                for (int ii = 0; ii < joint.muscles?.Length; ii++)
                {
                    if (joint.muscles[ii] == null) continue;
                    muscles.Add(joint.muscles[ii]);
                }
            }

            motionMuscles = muscles.ToArray() ;
        }

        /// Input data to kernel
        /// </summary>
        /// <param name="kernel"></param>
        private void RegisterData()
        {
            if (kernel != null)
            {
                kernel.Dispose();
            }
            kernel = new MagicMotionKernel((SearchLevel)4,21,3);

            MuscleData[] muscleDatas = new MuscleData[motionMuscles.Length];
            for (int i = 0; i < motionMuscles.Length; i++)
            {
                muscleDatas[i] = motionMuscles[i].GetNativeData();
            }
            kernel.SetMuscleSata(muscleDatas);


            JointData[] jointData = new JointData[motionJoints.Length];

            for (int i = 0; i < motionJoints.Length; i++)
            {
                jointData[i] = motionJoints[i].GetNativeJointData();
            }
            kernel.SetJointData(jointData);

            constraintDatas = new ConstraintData[motionJoints.Length];

            for (int i = 0; i < motionJoints.Length; i++)
            {
                var constraint= motionJoints[i].GetNativeConstraintData();
                constraintDatas[i] = constraint;
            }
            kernel.SetConstraintData(constraintDatas);
            kernel.SetOutDataFunc(SetMuscle);
            kernel.Initialize();
        }
        private void UpdateData()
        {
            worldPosition = rootTransform.position;
            worldRotation = rootTransform.parent==null ? quaternion.identity : rootTransform.parent.rotation;

            for (int i = 0; i < constraintDatas.Length; i++)
            {
                constraintDatas[i] = motionJoints[i].GetNativeConstraintData();
            }
        }
        private  void UpdateCruve()
        {
/*            gradientCurve.keys = kernel.GetGradientsKey(editorIndex);
            muscleCurve.keys = kernel.GetmusclesKey(editorIndex);*/
            lossCurve.keys = kernel.GetLossKey();
        }
        #endregion

        #region StaticFunc
        #endregion
    }
}