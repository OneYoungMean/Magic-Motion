
using System.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public class Orientation : BioObjective
    {
        [SerializeField] private Transform Target;
        [SerializeField] private float3 targetRotationEuler;
        private quaternion targetRotation;
        [SerializeField] private float MaximumRotationError = 0.1f;

        public override bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            float d = math.dot(worldRotation, targetRotation);
            if (d < 0.0f)
            {
                d = -d;
                if (d > 1.0f)
                {
                    d = 1.0f;
                }
            }
            else if (d > 1.0f)
            {
                d = 1.0f;
            }
            return 2.0f * math.acos(d) <=math.radians( MaximumRotationError);
        }

        public override float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, float3[] configuration)
        {
            float d = math.dot(worldRotation, targetRotation);
            if (d < 0.0f)
            {
                d = -d;
                if (d > 1.0f)
                {
                    d = 1.0f;
                }
            }
            else if (d > 1.0f)
            {
                d = 1.0f;
            }
            float loss = 2.0f * math.acos(d);
            return weight * loss * loss;
        }

        public override ObjectiveType GetObjectiveType()
        {
            return ObjectiveType.Orientation;
        }

        public override void UpdateData()
        {
            if (segment.character.evolution == null)
            {
                return;
            }
            targetRotation = Target!=null? (quaternion)Target.rotation:quaternion.Euler(targetRotationEuler);
        }


        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}