using BioIK2;
using System.Collections;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BioIK2
{
    public class Position : BioObjective
    {
        [SerializeField] public Transform Target;
        [SerializeField] public float3 targetPosition;
        [SerializeField] public float MaximumError = 0.001f;

        private float ChainLength;
        private float Rescaling;//OYM：乘以了一个pi，貌似是用来保持和角度尺度一致的

        public override bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, NativeArray<float3> configuration)
        {
            //  return math.length(targetPosition - worldPosition)<= MaximumError;
            return false;
        }

        public override float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, NativeArray<float3> configuration)
        {
            return weight * Rescaling * math.lengthsq(targetPosition - worldPosition);
        }

        public override ObjectiveType GetObjectiveType()
        {
            return ObjectiveType.Position;
        }

        public override void UpdateData()
        {
            if (segment.character.evolution == null)
            {
                return;
            }
            ChainLength = 0.0f;
            Transform[] chain = segment.character.evolution.GetModel().FindObjectivePtr(this).Node
                .chain;
            for (int i = 0; i < chain.Length - 1; i++)
            {
                ChainLength += Vector3.Distance(chain[i].position, chain[i + 1].position);
            }
            Rescaling = (math.PI* math.PI) / (ChainLength * ChainLength);
            //Root = chain[0].position;
            targetPosition = Target != null ? Target.position : segment.character.evolution.GetModel().FindObjectivePtr(this).Node.transform.position;
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