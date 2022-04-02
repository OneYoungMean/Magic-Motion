
using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;

namespace BioIK2
{
    public unsafe class BioModel:IDisposable
    {
        private BIOIK2 Character;

        private BioSegment root;

        public float3 positionOffset;
        public quaternion rotationOffset;
        public float3 scaleOffset;

        public List< BioNode> nodes = new List<BioNode>();
        public List<ObjectivePtr> objectivePtrs = new List<ObjectivePtr>();
        public List<MotionPtr> motionPtrs = new List<MotionPtr>();

        public NativeArray<float> Configuration;
        public NativeArray<float> Gradient;
        public NativeArray<float> Losses;
        public NativeArray<float> SimulatedLosses;

        public NativeArray<float3> tempPositions;
        public NativeArray<quaternion> tempRotations;

        public NativeArray<float3> tempConfiguration;


        private int Dof3;

        public BioModel(BIOIK2 character)
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
            tempPositions = new NativeArray<float3>(objectivePtrs.Count,Allocator.Persistent);
            tempRotations = new NativeArray<quaternion>(objectivePtrs.Count, Allocator.Persistent);


            Losses = new NativeArray<float>(objectivePtrs.Count * 3, Allocator.Persistent);
            SimulatedLosses = new NativeArray<float>(objectivePtrs.Count * 3, Allocator.Persistent);

            Configuration = new NativeArray<float>(motionPtrs.Count*3,Allocator.Persistent);
            Gradient = new NativeArray<float>(motionPtrs.Count*3, Allocator.Persistent);
            tempConfiguration = new NativeArray<float3>(motionPtrs.Count, Allocator.Persistent);


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

        public MotionPtr FindMotionPtr(BioMotion motion)
        {
            for (int i = 0; i < motionPtrs.Count; i++)
            {
                if (motionPtrs[i].Motion == motion)
                {
                    return motionPtrs[i];
                }
            }
            return null;
        }

        internal ObjectivePtr FindObjectivePtr(BioObjective objective)
        {
            for (int i = 0; i < objectivePtrs.Count; i++)
            {
                if (objectivePtrs[i].Objective == objective)
                {
                    return objectivePtrs[i];
                }
            }
            return null;
        }

        public void CopyFrom(BioModel model)
        {
            positionOffset=model.positionOffset;
            rotationOffset=model.rotationOffset;
            scaleOffset=model.scaleOffset;

            Configuration.CopyFrom(model.Configuration);
            Gradient.CopyFrom(model.Gradient);
            tempPositions.CopyFrom(model.tempPositions);
            tempRotations.CopyFrom(model.tempRotations);
            Losses.CopyFrom(model.Losses);
            SimulatedLosses.CopyFrom(model.SimulatedLosses);

            for (int i = 0; i < nodes.Count; i++)
            {
                nodes[i].worldPosition=model.nodes[i].worldPosition;
                nodes[i].worldRotation = model.nodes[i].worldRotation;
                nodes[i].worldScale = model.nodes[i].worldScale;
                nodes[i].localPosition = model.nodes[i].localPosition;
                nodes[i].localRotation = model.nodes[i].localRotation;
                nodes[i].currentValue = model.nodes[i].currentValue;
            }
        }

        internal NativeArray<float> ComputeGradient(NativeArray<float> configuration, float resolution)
        {
            float oldLoss = ComputeLoss(configuration);
            for (int j = 0; j <Dof3*3; j++)
            {
                Configuration[j] += resolution;
                UnsafeUtility.MemCpy(tempConfiguration.GetUnsafePtr(), Configuration.GetUnsafePtr(), tempConfiguration.Length * UnsafeUtility.SizeOf<float3>());
                motionPtrs[j/3].Node.SimulateModification(tempConfiguration);
                Configuration[j] -= resolution;
                float newLoss = 0.0f;
                for (int i = 0; i < objectivePtrs.Count; i++)
                {
                    newLoss += SimulatedLosses[i];
                }
                newLoss = (float)System.Math.Sqrt(newLoss / (float)objectivePtrs.Count);
                Gradient[j] = (float)((newLoss - oldLoss) / resolution);
            }
            return Gradient;
        }

        internal float ComputeLoss(NativeArray<float> configuration)
        {
            UnsafeUtility.MemCpy(tempConfiguration.GetUnsafePtr(), configuration.GetUnsafePtr(), tempConfiguration.Length * UnsafeUtility.SizeOf<float3>());

            return ComputeLoss(tempConfiguration);
        }
        internal float ComputeLoss(NativeArray<float3> configuration)
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
        public bool CheckConvergence(NativeArray<float3> configuration)
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
        private void FK(NativeArray<float3> configuration)
        {
            UnsafeUtility.MemCpy (Configuration.GetUnsafePtr(), configuration.GetUnsafePtr(), Configuration.Length*UnsafeUtility.SizeOf<float>());
            nodes[0].FeedForwardConfiguration(configuration);
        }

        public void Refresh()
        {
            for (int i = 0; i < motionPtrs.Count; i++)
            {
                float3 value = motionPtrs[i].Motion.GetTargetValue(true);
                Configuration[i*3] = value.x;
                Configuration[i*3 + 1] = value.y;
                Configuration[i*3 + 2] = value.z;
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
                        targetNode.enabledValue = motionPtr.Motion.isEnableValue;
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

        public BIOIK2 GetCharacter()
        {
            return Character;
        }

        public void Dispose()
        {
            Configuration.Dispose();
            Gradient.Dispose();
            Losses.Dispose();
            SimulatedLosses.Dispose();
            tempPositions.Dispose();
            tempRotations.Dispose();
            tempConfiguration.Dispose();

        }
    }
}