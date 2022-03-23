using BIOIK2;
using System.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    //OYM：这个我认为应该是将脚投影到平面上什么的
    //OYM：我认为这个结果有问题，所以暂时没有实现
    //OYM：我觉得应该是引导向量垂直于当前的平面向量，并且在穿过碰撞体的时候试图修复。
    //OYM：并且它应当在执行完position与rotation校准之后执行
    //OYM：否则冲突将会难以抑制
    public unsafe class Projection : BioObjective
    {
        [SerializeField] private Transform Target;
        [SerializeField] private float3 targetPosition;
        [SerializeField] private quaternion targetRotation;
        [SerializeField] private float MaximumError = 0.001f;
        [SerializeField] private float3 Normal = Vector3.up;
        [SerializeField] private float Length;
        [SerializeField] private float Sensitivity = 0.75f;

        private float3 lastPosition;
        private Quaternion lastRotation;

        //private Vector3 Position;
        //private Quaternion Rotation;
        private float chainLength;
        private float rescaling;

        private RaycastHit[] Hits;
        public override bool CheckConvergence(float3 worldPosition, quaternion worldRotation, BioNode node, float3* configuration)
        {
            return true;
        }

        public override float ComputeLoss(float3 worldPosition, quaternion worldRotation, BioNode node, float3* configuration)
        {
            return 0;
        }

        public override ObjectiveType GetObjectiveType()
        {
           return ObjectiveType.Projection;
        }

        public override void UpdateData()
        {
            
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