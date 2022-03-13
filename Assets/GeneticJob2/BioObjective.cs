using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public abstract class BioObjective : MonoBehaviour
    {
        public BioSegment segment;
        public double weight = 1;
        // Start is called before the first frame update
        void Start()
        {

        }

        internal void Erase()
        {
            Utility.Destroy(this);
        }

        // Update is called once per frame
        void Update()
        {

        }

        public void Create(BioSegment segment)
        {
            this.segment = segment;
        }

        public abstract void UpdateData();

        public abstract float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration);

        public abstract bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration);

        public abstract float ComputeValue(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration);
    }
}

