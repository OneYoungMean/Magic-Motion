using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using System;
using Random = Unity.Mathematics.Random;
using UnityEngine;
using static GeneticJob.GenerticJobs;
using UnityEngine.Jobs;

namespace GeneticJob
{

    public unsafe class GeneticIKKernal :IDisposable
    {
        const int innerLoopCount = 32;
        public TransformAccessArray transformAccessArray;
        public NativeArray<GeneticJoint> geneticJoint;//OYM:just only read;
        public NativeArray<RigidTransform> targetTransformData;
        public NativeArray<float> bestFitness;

        public NativeArray<float4> targetPosition;
        public NativeArray<ulong> genesRawData;
        //public NativeArray<Random> randoms;

        public GeneticJoint* geneticJointPtr;
        public float4* targetPositionPtr;
        public RigidTransform* targetTransformDataPtr;
        public float* bestFitnessPtr;
        //public Random* randomsPtr;

        public JobHandle Hjob;

        public int jointCount;
        public void SetData(Transform[] transforms, GeneticJoint[] joints)
        {
            Dispose();

            jointCount = joints.Length;
            transformAccessArray = new TransformAccessArray(transforms);

            geneticJoint = new NativeArray<GeneticJoint>(joints, Allocator.Persistent);
            targetPosition = new NativeArray<float4>(jointCount, Allocator.Persistent);
            targetTransformData= new NativeArray<RigidTransform>(jointCount, Allocator.Persistent);
            bestFitness = new NativeArray<float>(1, Allocator.Persistent);
            //randoms = new NativeArray<Random>(randomsTemp, Allocator.Persistent);

            geneticJointPtr =(GeneticJoint *) geneticJoint.GetUnsafePtr();
            targetPositionPtr = (float4*)targetPosition.GetUnsafePtr();
            targetTransformDataPtr = (RigidTransform*)targetTransformData.GetUnsafePtr();
            bestFitnessPtr = (float*)bestFitness.GetUnsafePtr();
            //randomsPtr = (Random*)randoms.GetUnsafePtr();


        }

        public void Schedule(bool isAsync=false,int iteration =1000, int BatchCount =1024, int EliteGeneCount = 8,int RandomGeneCount=64)
        {
            if (!Hjob.IsCompleted)
            {
                Debug.Log("Cannot Schedule: Uncomplete work");
                return;
            }
            if (!genesRawData.IsCreated|| genesRawData.Length!= BatchCount * EliteGeneCount * jointCount)
            {
                if (genesRawData.IsCreated )
                {
                    genesRawData.Dispose();
                }

                genesRawData = new NativeArray<ulong>(BatchCount * EliteGeneCount *GeneDynamic1.GetRawLength(jointCount) , Allocator.Persistent,NativeArrayOptions.UninitializedMemory);
            }
            ulong* rawEliteDataPtr = (ulong*)genesRawData.GetUnsafePtr();

            PrepareGeneticDataJob prepareGeneticDataJob = new PrepareGeneticDataJob()
            {
                targetPosition = targetPositionPtr
            };

            GeneticAlgorizomComputeJob geneticProcessJob = new GeneticAlgorizomComputeJob()
            {
                geneticJointPtr = geneticJointPtr,
                targetPositionPtr = targetPositionPtr,

                rawEliteDataPtr = rawEliteDataPtr,

                EliteGeneCount = EliteGeneCount,
                MutationGeneCount = EliteGeneCount,
                CrossoverGeneCount = EliteGeneCount* EliteGeneCount,
                RandomGeneCount = RandomGeneCount,

                geneSnippetCount=jointCount,
                //jointCount= jointCount,
                iteration =iteration,
            };

            PoseprocessGeneticComputeJob processGeneticComputeJob = new PoseprocessGeneticComputeJob()
            {
                rawEliteDataPtr= rawEliteDataPtr,
                targetWorldTramsformPtr= targetTransformDataPtr,
                geneticJointPtr= geneticJointPtr,
                bestFitnessPtr =bestFitnessPtr,

                allEliteGeneCount = EliteGeneCount* BatchCount,
                geneSnippetCount = jointCount
            };

            ReWriteResultToTransformJob reWriteResultToTransformJob = new ReWriteResultToTransformJob
            {
                targetWorldTramsformPtr = targetTransformDataPtr
            };

            if (isAsync)
            {
                Hjob=prepareGeneticDataJob.ScheduleReadOnly(transformAccessArray,innerLoopCount);
                Hjob=geneticProcessJob.Schedule(BatchCount, innerLoopCount, Hjob);
               // Hjob = processGeneticComputeJob.Schedule(Hjob);
               // Hjob = reWriteResultToTransformJob.Schedule(transformAccessArray, Hjob);
            }
            else
            {
                prepareGeneticDataJob.RunReadOnly(transformAccessArray);
                geneticProcessJob.Run(BatchCount);
                //processGeneticComputeJob.Run();
                //reWriteResultToTransformJob.Schedule(transformAccessArray);
            }
        }

        public void Dispose()
        {
            if (geneticJoint.IsCreated)
            {
                geneticJoint.Dispose();
            }
            if (targetPosition.IsCreated)
            {
                targetPosition.Dispose();
            }
            if (transformAccessArray.isCreated)
            {
                transformAccessArray.Dispose();
            }
            if (genesRawData.IsCreated)
            {
                genesRawData.Dispose();
            }
            if (targetTransformData.IsCreated)
            {
                targetTransformData.Dispose();
            }
            if (bestFitness.IsCreated)
            {
                bestFitness.Dispose();
            }
        }
    }
}