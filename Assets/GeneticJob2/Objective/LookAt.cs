using System.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public class LookAt : BioObjective
    {
        [SerializeField] private Transform Target;
        [SerializeField] private Vector3 targetPositon;
        [SerializeField] private Vector3 ViewingDirection = Vector3.forward;
        [SerializeField] private float MaximumError = 0.1f;

        public override bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            return ComputeValue(worldPosition,worldRotation,node,configuration) < math.radians(MaximumError);
        }

        public override float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            float loss = ComputeValue(worldPosition, worldRotation, node, configuration);
            return loss* loss* weight;
        }

        public override ObjectiveType GetObjectiveType()
        {
            return ObjectiveType.LookAt;
        }

        public override void UpdateData()
        {
            if (segment.character.evolution == null)
            {
                return;
            }
            if (Target != null)
            {
                targetPositon= Target.position;

            }
        }

        public  float ComputeValue(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            float3 targetForward = math.mul(worldRotation, ViewingDirection);
            float3 targetDirection =   (float3)targetPositon- worldPosition;
            float cosA = math.dot(targetForward, targetDirection) / (math.length(targetForward) * math.length(targetDirection));
            cosA= math.clamp(cosA, - 1, 1);
            return math.acos(cosA);
        }
    }
}