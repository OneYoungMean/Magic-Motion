﻿using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace BIOIK2
{
    public class BioModel:ICloneable
    {
        private Bioik2 Character;

        private BioSegment root;

        public float3 positionOffset;
        public quaternion rotationOffset;
        public float3 scaleOffset;

        public List< BioNode> nodes = new List<BioNode>();
        public List<ObjectivePtr> objectivePtrs = new List<ObjectivePtr>();
        public List<MotionPtr> motionPtrs = new List<MotionPtr>();

        private float3[] Configuration;
        private float3[] Gradient;
        private float[] Losses;
        private float[] SimulatedLosses;

        private float3[] tempPositions;
        private quaternion[] tempRotation;



        private int Dof3;

        public BioModel(Bioik2 character)
        {
            this.Character = character;
            root = character.FindSegment(Character.transform);//OYM:找到root节点
            //此处应当有一个报错
            AddNode(root);

            var objectIiveCollect = new List<BioObjective>();
            CollectObjectives(root, objectIiveCollect);
            for (int i = 0; i < objectIiveCollect.Count; i++)
            {
                List<BioSegment> chain = GetChain(root, objectIiveCollect[i].segment);
                for (int j0 = 0; j0 < chain.Count; j0++)
                {
                    AddNode(chain[j0]);
                }
            }
            Dof3 = motionPtrs.Count;//多嘴一句,motionptrs的count应该是objectivePtrs的count的三倍,这里我做了一些合并,现在他们一样多

            for (int i = 0; i < nodes.Count; i++)
            {
                nodes[i].ObjectiveImpacts = new bool[objectivePtrs.Count];
            }
            tempPositions = new float3[objectivePtrs.Count];
            tempRotation = new quaternion[objectivePtrs.Count];
            Configuration = new float3[objectivePtrs.Count];
            Gradient = new float3[objectivePtrs.Count];
            Losses = new float[objectivePtrs.Count];
            SimulatedLosses = new float[objectivePtrs.Count];

            for (int i = 0; i < objectivePtrs.Count; i++)
            {
                BioNode node = objectivePtrs[i].Node;
                while (node!=null)
                {
                    node.ObjectiveImpacts[i] = true;
                    node = node.parent;
                }
            }

            Refresh();
        }
        public void CopyFrom(BioModel model)
        {
            positionOffset=model.positionOffset;
            rotationOffset=model.rotationOffset;
            scaleOffset=model.scaleOffset;

            Array.Copy(model.Configuration, Configuration, Dof3);
            Array.Copy(model.Gradient, Gradient, Dof3);

            Array.Copy(model.tempPositions, tempPositions, objectivePtrs.Count);
            Array.Copy(model.tempRotation, tempRotation, objectivePtrs.Count);
            Array.Copy(model.Losses, Losses, objectivePtrs.Count);
            Array.Copy(model.SimulatedLosses, SimulatedLosses, objectivePtrs.Count);

            for (int i = 0; i < nodes.Count; i++)
            {
                nodes[i].worldPosition=model.nodes[i].worldPosition;
                nodes[i].worldRotation = model.nodes[i].worldRotation;
                nodes[i].worldScale = model.nodes[i].worldScale;
                nodes[i].localPosition = model.nodes[i].localPosition;
                nodes[i].localRotation = model.nodes[i].localRotation;
                nodes[i].value = model.nodes[i].value;
            }
        }
        
        internal float ComputeLoss(float3[] configuration)
        {
            FK(configuration);
            float loss = 0.0f;
            for (int i = 0; i < objectivePtrs.Count; i++)
            {
                BioNode node = objectivePtrs[i].Node;
                Losses[i] = objectivePtrs[i].Objective.ComputeLoss(node.worldPosition, node.worldRotation, node, configuration);
                loss += Losses[i];
            }
            return (float)System.Math.Sqrt(loss / (float)objectivePtrs.Count);
        }
        public bool CheckConvergence(float3[] configuration)
        {
            FK(configuration);
            for (int i = 0; i < objectivePtrs.Count; i++)
            {
                BioNode node = objectivePtrs[i].Node;
                if (!objectivePtrs[i].Objective.CheckConvergence(node.worldPosition, node.worldRotation, node, configuration))
                {
                    return false;
                }
            }
            return true;
        }
        private void FK(float3[] configuration)
        {
            Array.Copy(configuration, Configuration, Configuration.Length);
            nodes[0].FeedForwardConfiguration(configuration);
        }
            public void Refresh()
        {
            for (int i = 0; i < Configuration.Length; i++)
            {
                Configuration[i] = motionPtrs[i].Motion.GetTargetValue(true);
            }

            if (root.transform.root == Character.transform)
            {
                positionOffset = 0;
                rotationOffset = quaternion.identity;
                scaleOffset = 1;
            }
            else
            {
                Transform parent = root.transform.parent;
                positionOffset = parent.position;
                rotationOffset = parent.rotation;
                scaleOffset = parent.lossyScale;
            }

            nodes[0].Refresh();
        }

        private void AddNode(BioSegment targetSegment)
        {
            if (FindNode(targetSegment.transform) == null)
            {
                BioNode targetNode = new BioNode(this, FindNode(targetSegment.transform.parent), targetSegment);
                if (targetNode.joint!=null)
                {
                    if (targetNode.joint.GetDof()==0||!targetNode.joint.enabled)
                    {
                        targetNode.joint =null;
                    }
                    else
                    {
                        MotionPtr motionPtr = new MotionPtr(targetNode.joint.bioMotion, targetNode, motionPtrs.Count); //OYM:

                        motionPtrs.Add(motionPtr);
                        targetNode.enabled = motionPtr.Motion.isEnable;
                        targetNode.index = motionPtr.Index;
                    }
                }
                for (int i = 0; i < targetSegment.objectives.Length; i++)
                {
                    var targetObject = targetSegment.objectives[i]; 
                    if (targetObject.enabled)
                    {
                        objectivePtrs.Add(new ObjectivePtr(targetObject, targetNode, objectivePtrs.Count));

                    }
                }
                nodes.Add(targetNode);
            }
        }


        private BioNode FindNode(Transform target)
        {
            if (target != null)
            {
                return nodes.Find(x => x.transform == target);
            }
            else
            {
                return null;
            }
        }

        private  void CollectObjectives(BioSegment segment, List<BioObjective> objectiveList)
        {
            //这个....既不是广搜也不是深搜
            for (int i = 0; i < segment.objectives?.Length; i++)
            {
                if (segment.objectives[i].enabled)
                {
                    objectiveList.Add(segment.objectives[i]);
                }
                
            }
            for (int i = 0; i < segment.childs.Count; i++)
            {
                CollectObjectives(segment.childs[i], objectiveList);
            }
        }
        internal List<BioSegment> GetChain(BioSegment root, BioSegment end)
        {
            List<BioSegment> chain = new List<BioSegment>();
            BioSegment target = end;
            while (target != root)
            {
                chain.Add(target);
                target = target.parent;
            }
            chain.Reverse();
            return chain;
        }

        public int GetDof3()
        {
            return Dof3;

        }

        public Bioik2 GetCharacter()
        {
            return Character;
        }

        public object Clone()
        {
            BioModel model = new BioModel(Character);

            model.positionOffset = positionOffset;
            model.rotationOffset = rotationOffset;
            model. scaleOffset = scaleOffset;

            Array.Copy(Configuration, model.Configuration, Dof3);
            Array.Copy(Gradient, model.Gradient, Dof3);
            Array.Copy(tempPositions, model.tempPositions, objectivePtrs.Count);
            Array.Copy(tempRotation, model.tempRotation, objectivePtrs.Count);
            Array.Copy(Losses, Losses, model.objectivePtrs.Count);
            Array.Copy(SimulatedLosses, model.SimulatedLosses, objectivePtrs.Count);

            for (int i = 0; i < nodes.Count; i++)
            {
                model.nodes[i].CopyFrom(nodes[i]);
            }

            return model;
        }
    }
}