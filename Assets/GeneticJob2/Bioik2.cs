using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public class BIOIK2 : MonoBehaviour
    {
        #region  Field&Property


        [SerializeField] private bool UseThreading = true;

        [SerializeField] private int Generations = 2;
        [SerializeField] private int PopulationSize = 50;
        [SerializeField] private int Elites = 2;

        public float smoothing = 0.5f;
        public float animationWeight = 0;
        public float animationBlend = 0;

        internal MotionType motionType;

        public float maximumVelocity = 3;
        public float maximumAcceleration = 3;

        public List<BioSegment> segments = new List<BioSegment>();
        public BioSegment root = null;
        public BioEvolution evolution = null;
        public NativeArray<float3> solutions;
        // Start is called before the first frame update

        private bool Destoryed = false;

        public BioSegment selectedSegment = null;
        public Vector2 scroll = Vector2.zero;

        #endregion

        #region UnityFunc

        private void Awake()
        {
            Refresh();
        }
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            PrecaptureAnimation(root);
        }

        private void LateUpdate()
        {
            PostcaptureAnimation(root);

            UpdateData(root);

            for (int i = 0; i < solutions.Length; i++)
            {
                solutions[i] = evolution.GetModel().motionPtrs[i].Motion.GetTargetValue(true);
            }
            solutions = evolution.Optimise(Generations);
            for (int i = 0; i < solutions.Length; i++)
            {
                BioMotion motion = evolution.GetModel().motionPtrs[i].Motion;
                motion.SetTargetValue(solutions[i], true);
            }

            ProcessMotion(root);
        }

        public void SetGenerations(int generations)
        {
            Generations = generations;
        }

        private void UpdateData(BioSegment segment)
        {
            if (segment.joint != null)
            {
                if (segment.joint.enabled)
                {
                    segment.joint.UpdateData();
                }
            }
            for (int i = 0; i < segment.objectives.Length; i++)
            {
                if (segment.objectives[i].enabled)
                {
                    segment.objectives[i].UpdateData();
                }
            }
            for (int i = 0; i < segment.childs.Count; i++)
            {
                UpdateData(segment.childs[i]);
            }
        }

        private void OnDestroy()
        {
            Destoryed = true;
            solutions.Dispose();
            DeInitialise();
            Utility.Cleanup(transform);
        }
        private void OnEnable()
        {
            Initialise();
        }
        void OnDisable()
        {
            DeInitialise();
        }

        #endregion

        #region LocalFunc
        private void PostcaptureAnimation(BioSegment segment)
        {
            if (segment.joint != null)
            {
                if (segment.joint.enabled)
                {
                    segment.joint.PostcaptureAnimation();
                }
            }
            for (int i = 0; i < segment.childs.Count; i++)
            {
                PostcaptureAnimation(segment.childs[i]);
            }
        }

        private void PrecaptureAnimation(BioSegment segment)
        {
            if (segment.joint != null)
            {
                if (segment.joint.enabled)
                {
                    segment.joint.PrecaptureAnimation();
                }
            }
            for (int i = 0; i < segment.childs.Count; i++)
            {
                PrecaptureAnimation(segment.childs[i]);
            }
        }
        private void DeInitialise()
        {
            if (evolution != null)
            {
                evolution = null;
            }
        }

        private void Initialise()
        {
            if (evolution == null)
            {
                evolution = new BioEvolution(new BioModel(this), PopulationSize, Elites, UseThreading);
            }
        }

        internal BioSegment FindSegment(Transform target
            )
        {
            if (target != null)
            {
                return segments.FirstOrDefault(x => x.transform == target);
            }
            else
            {
                return null;
            }
        }

        internal void Refresh(bool isEvolution = true)
        {
            if (Destoryed)
            {
                return;
            }

            for (int i = 0; i < segments.Count; i++)
            {
                if (segments[i] == null)
                {
                    segments.RemoveAt(i);
                    i--;
                }
            }
            Refresh(transform);
            root = FindSegment(transform);

            if (isEvolution && Application.isPlaying)
            {
                DeInitialise();
                Initialise();
                solutions = new NativeArray<float3>(evolution.GetModel().GetDof3(),Allocator.Persistent);
            }
        }

        private void Refresh(Transform targetTrans)
        {
            BioSegment segment = FindSegment(targetTrans);

            if (segment == null)
            {
                segment = targetTrans.gameObject.AddComponent<BioSegment>().Create(this);
                segments.Add(segment);
            }
            segment.character = this;
            segment.RenewRelation();

            for (int i = 0; i < targetTrans.childCount; i++)
            {
                Refresh(targetTrans.GetChild(i));
            }
        }

        private void ProcessMotion(BioSegment segment)
        {
            if (segment.joint != null)
            {
                if (segment.joint.enabled)
                {
                    segment.joint.ProcessMotion();
                }
            }
            for (int i = 0; i < segment.childs.Count; i++)
            {
                ProcessMotion(segment.childs[i]);
            }
        }

        #endregion
        #region StaticFunc
        #endregion
    }
}

