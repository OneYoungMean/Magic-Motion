using System;
using System.Collections;
using System.Collections.Generic;
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

        public List<BioSegment> segments = new List<BioSegment>();
        public BioSegment root = null;
        public BioEvolution evolution = null;
        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        internal void Refresh()
        {
            throw new NotImplementedException();
        }
    }
}

