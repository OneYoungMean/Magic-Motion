using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BioIK2
{
    /// <summary>
    /// Make the node move the shortest distance to the next target
    /// </summary>
    public class Displacement : BioObjective
    {
        public override bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, NativeArray<float3> configuration)
        {
            return true;
        }

        public override float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, NativeArray<float3> configuration)
        {
            float loss = 0.0f;
            for (int i = 0; i < configuration.Length; i++)
            {
                float diff =math.csum( math.abs((segment.character.evolution.GetSolution()[i] - configuration[i]) / (segment.character.evolution.GetUpperBounds()[i] - segment.character.evolution.GetLowerBounds()[i])));
                loss += diff;
            }
            loss /= configuration.Length;
            return weight * loss * loss;
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

