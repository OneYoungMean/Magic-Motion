using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace BIOIK2
{
    public class Bioik2 : MonoBehaviour
    {
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

        public List< BioSegment> segments = new List<BioSegment>();
        public BioSegment root = null;
        public BioEvolution evolution = null;
        // Start is called before the first frame update

        private bool Destoryed = false;

        public BioSegment selectedSegment = null;
        public Vector2 scroll = Vector2.zero;

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

        }

        private void OnDestroy()
        {
            Destoryed = true;
            DeInitialise();
        }

        private void DeInitialise()
        {
            if (evolution!=null)
            {
                evolution.Kill();
            }
        }
        private void OnEnable()
        {
            Initialise();
        }

        private void Initialise()
        {
            if (evolution==null)
            {
                evolution = new BioEvolution(new BioModel(this), PopulationSize, Elites, UseThreading);
            }
        }

        internal BioSegment FindSegment(Transform target
            )
        {
            if (target!=null)
            {
                return segments.FirstOrDefault(x => x.transform == target);
            }
            else
            {
                return null;
            }
        }

        internal void Refresh()
        {
            if (Destoryed)
            {
                return;
            }

            for (int i = 0; i < segments.Count; i++)
            {
                if (segments[i]==null)
                {
                    segments.RemoveAt(i);
                    i--;
                }
            }
            Refresh(transform);
        }

        private void Refresh(Transform targetTrans)
        {
            BioSegment segment = FindSegment(targetTrans);

            if (segment==null)
            {
                segment = targetTrans.gameObject.AddComponent<BioSegment>().Create(this);
                segments.Add(segment);
            }
            segment.RenewRelation();

            for (int i = 0; i < targetTrans.childCount; i++)
            {
                Refresh(targetTrans.GetChild(i));
            }
        }

        
    }
}

