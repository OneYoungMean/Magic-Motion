using System;
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

        public float3 enabledValue;

        public int index;

        public float3 currentValue;

        public bool[] ObjectiveImpacts; //表示整个运动树中哪些目标指标受到影响的布尔值
        public BioNode(BioModel model, BioNode parent, BioSegment segment)
        {
            this.model = model;
            this.parent = parent;
            if (parent!=null)
            {
                this.parent.AddChild(this);
            }
            transform = segment.transform;
            joint = segment.joint;
            enabledValue = joint == null ? 0 : 1;
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
        public void Refresh()
        {
            if (joint==null)
            {
                localPosition = transform.localPosition;
                localRotation = transform.localRotation;
            }
            else
            {
                currentValue = joint.bioMotion.GetTargetValue(true);
                joint.ComputeLocalTransformation(currentValue, out localPosition, out localRotation);


            }
            worldScale = transform.lossyScale;
            ComputeWorldTransformation();

            //Feed Forward
            foreach (BioNode child in childs)
            {
                child.Refresh();
            }

        }

        private void ComputeWorldTransformation()
        {
            quaternion rotation;
            float3 position;
            if (parent ==null)
            {
                worldPosition = model.positionOffset;
                 rotation = model.rotationOffset;
                 position = model.scaleOffset * localPosition;
            }
            else
            {
                worldPosition = parent.worldPosition;
                 rotation = parent.worldRotation;
                 position = parent.worldScale * localPosition;
            }
            worldPosition +=math.mul( rotation , position);
            worldRotation = math.mul(rotation, localRotation);
        }

        internal void CopyFrom(BioNode other)
        {
            other.worldPosition = worldPosition;
            other.worldRotation = worldRotation;
            other.worldScale = worldScale;
            other.localPosition = localPosition;
            other.localRotation = localRotation;

            other.currentValue = currentValue;
        }

        internal void FeedForwardConfiguration(float3[] configuration, bool updateWorld = false)
        {
            bool updateLocal = math.any((enabledValue == 1)& configuration[index] != currentValue);
            if (updateLocal)
            {
                currentValue = configuration[index];
                joint.ComputeLocalTransformation(currentValue, out localPosition,out localRotation);
                updateWorld = true;
            }
            if (updateWorld)
            {
                ComputeWorldTransformation();
            }
            foreach (BioNode child in childs)
            {
                child.FeedForwardConfiguration(configuration, updateWorld);
            }
        }

        internal void SimulateModification(float3[] configuration)
        {
            float3[] positions = model.tempPositions;
            quaternion[] rotations = model.tempRotation;
            for (int i = 0; i < model.objectivePtrs.Count; i++)
            {
                var node = model.objectivePtrs[i].Node;
                if (ObjectiveImpacts[i])
                {
                    float3 inputData = math.lerp(currentValue, configuration[index], enabledValue);
                    joint.ComputeLocalTransformation(inputData, out float3 localPosition, out quaternion localRotation);
                    float3 _tempPosition, _tempDirection;
                    quaternion _tempRotation;
                    if (parent == null)
                    {
                        positions[i] = model.positionOffset;
                        _tempRotation = model.rotationOffset;
                        _tempPosition = model.scaleOffset * localPosition;
                    }
                    else
                    {
                        positions[i] = parent.worldPosition;
                        _tempRotation = parent.worldRotation;
                        _tempPosition = parent.worldScale * localPosition;
                    }

                    var _tempWorldRotation = math.mul(math.mul(_tempRotation, localRotation), math.inverse(worldRotation));
                    _tempDirection = node.worldPosition - worldPosition;
                    _tempPosition = math.mul(_tempRotation, _tempPosition) + math.mul(_tempWorldRotation, _tempDirection);
                    positions[i] += _tempPosition;
                    rotations[i] = math.mul(_tempWorldRotation, node.worldRotation);
                    //OYM：这里一长串本质上就是在求当前节点的坐标和旋转
                    model.SimulatedLosses[i] = model.objectivePtrs[i].Objective.ComputeLoss(positions[i], rotations[i], node, configuration);
                }
                else
                {
                    positions[i] = node.worldPosition;
                    rotations[i] = node.worldRotation;
                    model.SimulatedLosses[i] = model.Losses[i];
                }
            }
        }
    }

    public class ObjectivePtr
    {
        public BioObjective Objective;
        public BioNode Node;
        public int Index;
        public ObjectivePtr(BioObjective objective, BioNode node, int index)
        {
            Objective = objective;
            Node = node;
            Index = index;
        }

    }

    public class MotionPtr
    {
        public BioMotion Motion;
        public BioNode Node;
        public int Index;
        public MotionPtr(BioMotion motion, BioNode node, int index)
        {
            Motion = motion;
            Node = node;
            Index = index;
        }
    }
}