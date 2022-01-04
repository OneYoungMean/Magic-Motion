using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public class BioNode
    {
        public BioModel model;
        public BioNode parent;
        public List<BioNode> childs = new List< BioNode>();

        public Transform transform;
        public BioJoint joint;
        public Transform[] chain;

        public float3 worldPosition;
        public quaternion worldRotation;
        public float3 worldScale;

        public float3 localPosition;
        public quaternion localRotation;

        public bool3 enabled;

        public int3 index;

        public float3 value;

        public BioNode(BioModel model, BioNode parent, BioSegment segment)
        {
            this.model = model;
            this.parent = parent;
            if (parent!=null)
            {
                this.parent.AddChild(this);
            }
            transform = segment.transform;
            joint = segment.Joint;
            List<Transform> reverseChain = new List<Transform>();
            reverseChain.Add(transform);
            BioNode p = parent;
            while (p != null)
            {
                reverseChain.Add(p.transform);//OYM:逆向链条?
                p = p.parent;
            }
            reverseChain.Reverse();
            chain = reverseChain.ToArray();

        }

        public void AddChild(BioNode child)
        {
            childs.Add( child);
        }
    }
}