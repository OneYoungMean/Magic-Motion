using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    /// <summary>
    /// Make the node move the shortest distance to the next target
    /// </summary>
    public class SmallestAngle : BioObjective
    {
        public override bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            return true;
        }

        public override float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            float loss = 0.0f;
            for (int i = 0; i < configuration.Length; i++)
            {
                float temp = configuration[i].y;
                loss +=  temp;
            }
            loss = loss / configuration.Length;
            return weight * loss;
        }

        public override ObjectiveType GetObjectiveType()
        {
            return ObjectiveType.Displacement;
        }
        public override void UpdateData()
        {

        }
    }
}

