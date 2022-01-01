using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using System;
using Random = Unity.Mathematics.Random;
using System.Runtime.CompilerServices;
using UnityEngine.Jobs;
using Unity.Burst;

namespace GeneticJob
{
    using static GeneticMath;

public static unsafe class GenerticJobs
{
        [BurstCompile]
        public struct PrepareGeneticDataJob : IJobParallelForTransform 
        {
            [NativeDisableUnsafePtrRestriction]
            public float4* targetPosition;
            public void Execute(int index, TransformAccess transform)
            {
                float4 position =new float4( transform.position,1);
                targetPosition[index] = position;
              }
        }
        
        [BurstCompile]
        public struct RandomizeAllGene : IJobParallelFor
        {
            [NativeDisableUnsafePtrRestriction]
            public ulong* targetPosition;
            
            public void Execute(int index)
            {
            }
        }

        #region GeneticMainlyComputeJob
        /// <summary>
        /// Struct Main 
        /// </summary>
        [BurstCompile]
        public partial struct GeneticAlgorizomComputeJob : IJobParallelFor
        {

            const float uintMax = uint.MaxValue;

            /// <summary>
            ///  the skeleton controller point
            ///  has at least 44 rotation control joint and 1 position control joint 
            /// </summary>
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public GeneticJoint* geneticJointPtr;//OYM:length==jointCount
            /// <summary>
            ///  the target float4
            ///  the xyz is the target position
            ///  if the w>0 ,means that origin should move to position else means move back
            /// </summary>
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public float4* targetPositionPtr;//OYM:length==jointCount
            /*
            /// <summary>
            ///  rawRandom Value, change this frequent between different thread will create large lock and unlock operation, so copy it and keep it readonly
            /// </summary>         
            public Random* rawRandomsPtr;//OYM:length==jointCount ,every random for every rawData in Gene
            */

            /// <summary>
            /// Gene RawData,used to save Elite Gene;
            /// </summary>
            [NativeDisableUnsafePtrRestriction]
            public ulong* rawEliteDataPtr;//OYM:length==EliteGeneCount*batchCount
            /// <summary>
            /// EliteGene's count;
            /// </summary>
            public int EliteGeneCount;
            /// <summary>
            /// Mutation means use single EliteGene to create new gene
            /// </summary>
            public int MutationGeneCount;
            /// <summary>
            /// Crossover means use double EliteGene to create new gene
            /// </summary>
            public int CrossoverGeneCount;
            /// <summary>
            /// RandomGene means use Random Function to create new gene
            /// </summary>
            public int RandomGeneCount;

            /// <summary>
            /// means how many gene snippets in gene.
            /// is NOT a raw gene data length because a raw gene data contain a fitness data and gene snippetdata 
            /// </summary>
            public int geneSnippetCount;
            /// <summary>
            /// ==number of jointCount
            /// </summary>
            //public int jointCount;

            /// <summary>
            /// Means how many iteration are Genetic evolution ;
            /// </summary>
            public int iteration;

            /// <summary>
            ///  random seed
            /// </summary>
            /// <param name="index"></param>
            public int randomSeed;
            public void Execute(int index)
            {
                int individalCount = EliteGeneCount + CrossoverGeneCount+ MutationGeneCount + RandomGeneCount;
                int geneRawLength = GeneDynamic1.GetRawLength(geneSnippetCount);
                ulong* currentRawDataPtr = Move(rawEliteDataPtr, index * EliteGeneCount* geneRawLength);


                CreateTempData(individalCount, geneSnippetCount,
                out NativeArray<GeneDynamic1> rawDataReadWriteBuffer,
                out NativeArray<SortToken> sortTokenArray,
                out NativeArray<Random> randomArray,
                out NativeArray<ulong> tempRawData,
                out ulong* tempRawDataPtr);
                Random* randomArrayPtr = (Random*)randomArray.GetUnsafePtr();
                SortToken* sortTokenArrayPtr = (SortToken*)sortTokenArray.GetUnsafePtr();
                GeneDynamic1* rawDataReadWriteBufferPtr = (GeneDynamic1*)rawDataReadWriteBuffer.GetUnsafePtr();
                {
                    //CopyEliteData(currentRawDataPtr, tempRawDataPtr, EliteGeneCount, geneRawLength);

                    for (int i = 0; i < iteration; i++)
                    {
                        #region Calculate
                        int localRandomSeed = i * individalCount+ randomSeed;

                        ReGenerateIndivualData(rawDataReadWriteBufferPtr, EliteGeneCount, MutationGeneCount, CrossoverGeneCount, RandomGeneCount, geneSnippetCount,localRandomSeed);

                        ComputeFitness(rawDataReadWriteBufferPtr, geneticJointPtr, targetPositionPtr, sortTokenArrayPtr, individalCount, geneSnippetCount);

                        FastGetElite(rawDataReadWriteBufferPtr, sortTokenArrayPtr, individalCount, EliteGeneCount, geneSnippetCount);

                        #endregion
                    }

                    CopyEliteData(tempRawDataPtr, currentRawDataPtr, EliteGeneCount, geneRawLength);
                }
                rawDataReadWriteBuffer.Dispose();
                randomArray.Dispose();
                sortTokenArray.Dispose();
                tempRawData.Dispose();
            }
        }

        /// <summary>
        /// Genetic algrothim part 1, including get fitness , mutation and crossover function .Mostly you should only change this part. 
        /// </summary>
        public partial struct GeneticAlgorizomComputeJob
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void ComputeFitness(GeneDynamic1* tempGeneBufferPtr, GeneticJoint* geneticJointPtr, float4* targetPositionPtr, SortToken* sortTokenBufferPtr, int individalCount, int jointCount)
            {
                NativeArray<RigidTransform> transformTempArray = new NativeArray<RigidTransform>(jointCount, Allocator.Temp, NativeArrayOptions.ClearMemory);//OYM:存疑,如果这里不能被展开优化的话,则可以考虑放在for循环外边
                RigidTransform* transformTempArrayPtr = (RigidTransform*)transformTempArray.GetUnsafePtr();
                {
                    for (int geneIndex = 0; geneIndex < individalCount; geneIndex++)
                {
                    GeneDynamic1 currentGene = tempGeneBufferPtr[geneIndex];

                        //OYM:这里其实有一个对齐与不对齐的问题
                        //OYM:正常情况而言,gene应该是与joint一一对应,一个long对应一个joint
                        //OYM:但是有些joint实际并没有子节点,所以对于它们的gene其实是不会被用上的
                        //OYM:这里衍生出了两种写法,一种是保持对齐,另一种是减少joint,提供指向性
                        //OYM:两种都不难,先用第一种,第二种待会测一下.

                        float4 globalfitness = 0;
                        for (int jointIndex = 0; jointIndex < jointCount; jointIndex++)//OYM:潜在的burst bug，如果这里被for循环展开优化，可能会出现顺序丢失导致值为0的问题
                        {
                            float4 localFitness = 0;

                            GeneticJoint currentJoint = geneticJointPtr[jointIndex];
                            RigidTransform currentTransform = new RigidTransform();

                            ulong rawData = currentGene[jointIndex];
                            float4 transcriptionData = GeneticMath.UlongTofloat4(rawData);

                            switch (currentJoint.JointMode)
                            {
                                case JointMode.Position:
                                    {
                                        //OYM:正常而言应该是先去寻找父节点的位置,再来计算这个
                                        //OYM:但是父节点存在和位置计算很迷(说实话我自己也没想好这个root节点到底该怎么处理pos,一直置零吗?)
                                        float3 moveFloat3 = GeneticMath.NomalizeTofloat3(transcriptionData, 2, currentJoint.MinRange, currentJoint.MaxRange);
                                        currentTransform.pos = moveFloat3;
                                        currentTransform.rot = quaternion.identity;
                                    }
                                    break;
                                case JointMode.Rotation:
                                    {
                                        //OYM:get parent data
                                        int parentIndex = currentJoint.parentIndex;
                                        RigidTransform parentTransform = transformTempArrayPtr[parentIndex];
                                        //OYM:get current gene rotate
                                        float3 euler = GeneticMath.NomalizeTofloat3(transcriptionData, 1, currentJoint.MinRange, currentJoint.MaxRange);
                                        quaternion rotateEuler = quaternion.Euler(euler);
                                        //OYM:clac and set transform
                                        currentTransform.pos = parentTransform.pos+ math.mul(parentTransform.rot, currentJoint.localPosition);
                                        currentTransform.rot = math.mul(rotateEuler, math.mul(currentJoint.localRotation, parentTransform.rot));
                                        float4 targetPositon = targetPositionPtr[jointIndex];
                                        localFitness.x = math.length(currentTransform.pos - targetPositon.xyz) * targetPositon.w*1000;
                                        localFitness.y = math.csum(math.abs(euler));
                                    }
                                    break;
                                default:
                                    break;
                            }

                            //OYM:get current transform fitness
                            transformTempArrayPtr[jointIndex] = currentTransform;
                            globalfitness += localFitness;
                        }


                        var sortToken = new SortToken()
                        {
                            index = geneIndex,
                            value =- math.length(globalfitness)/jointCount,
                        };
                        sortTokenBufferPtr[geneIndex] = sortToken;

                    }
                }
                transformTempArray.Dispose();
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void MutationGene(GeneDynamic1 origin, GeneDynamic1 destination, int geneSnippetCount, long v)
            {
                for (int j0 = 0; j0 < geneSnippetCount; j0++)
                {
                    destination[j0] = Mutation_FilpBitFast(origin[j0]);
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void CrossoverGene(GeneDynamic1 origin1, GeneDynamic1 origin2, GeneDynamic1 destination, int geneSnippetCount, uint randomseed)
            {
                Random random =  Random.CreateFromIndex(randomseed+ (uint)destination.First);
                Crossover_CycleCrossArray(origin1.GetSnippetGenePtr(), origin2.GetSnippetGenePtr(), destination.GetSnippetGenePtr(), geneSnippetCount, random);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void RandomizeGene(GeneDynamic1 target, int geneSnippetCount, uint v)
            {
                ulong* snippetDataPtr = target.GetSnippetGenePtr();
                Randomize_RandomGene(snippetDataPtr, new Random((uint)snippetDataPtr+v), geneSnippetCount);
            }
        }

        /// <summary>
        /// Genetic algorithm part 2, mostly you won't  change this part
        /// </summary>
        public partial struct GeneticAlgorizomComputeJob
        {
            /// <summary>
            ///  Create temp data
            ///  WARNING: you should DISPOSE your native collection on jobs after you don't need that.
            /// </summary>
            /// <param name="individalCount"></param>
            /// <param name="geneRawLength"></param>
            /// <param name="tempGeneBufferPtr"></param>
            /// <param name="tempsGenes"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void CreateTempData(int individalCount, int geneSnippetCount,
                out NativeArray<GeneDynamic1> rawDataReadWriteBuffer,
                out NativeArray<SortToken> sortTokenArray,
                out NativeArray<Random> randomArray,
                out NativeArray<ulong> rawData,
                out ulong* rawDataPtr)
            {
                int geneRawLength = GeneDynamic1.GetRawLength(geneSnippetCount);

                rawData = new NativeArray<ulong>(geneRawLength * individalCount, Allocator.Temp, NativeArrayOptions.ClearMemory);
                rawDataPtr = (ulong*)rawData.GetUnsafePtr();

                rawDataReadWriteBuffer = new NativeArray<GeneDynamic1>(individalCount, Allocator.Temp, NativeArrayOptions.ClearMemory);
                sortTokenArray = new NativeArray<SortToken>(individalCount, Allocator.Temp, NativeArrayOptions.ClearMemory);
                randomArray = new NativeArray<Random>(individalCount, Allocator.Temp, NativeArrayOptions.ClearMemory);


                for (int i = 0; i < individalCount; i++)
                {
                    rawDataReadWriteBuffer[i] = new GeneDynamic1(Move(rawDataPtr, i * geneRawLength), geneSnippetCount);
                    sortTokenArray[i] = new SortToken() { index = i, value = float.NaN };
                    randomArray[i] = rawDataReadWriteBuffer[i].random;
                }

            }
            /// <summary>
            ///  copy the least elite result to this data,it can reduce the iteration count
            /// </summary>
            /// <param name="rawEliteDataPtr"></param>
            /// <param name="tempRawDataPtr"></param>
            /// <param name="eliteGeneCount"></param>
            /// <param name="geneRawLength"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void CopyEliteData(ulong* rawEliteDataPtr, ulong* tempRawDataPtr, int eliteGeneCount, int geneRawLength)
            {
                UnsafeUtility.MemCpy(tempRawDataPtr, rawEliteDataPtr, eliteGeneCount * geneRawLength * ULONG_SIZE);
            }

            /// <summary>
            /// move elite gene to array head;
            /// </summary>
            /// <param name="rawDataReadWriteArrayPtr"></param>
            /// <param name="sortTokenBufferPtr"></param>
            /// <param name="individalCount"></param>
            /// <param name="eliteCount"></param>
            /// <param name="geneSnippetCount"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void FastGetElite(GeneDynamic1* rawDataReadWriteArrayPtr, SortToken* sortTokenBufferPtr, int individalCount, int eliteCount, int geneSnippetCount)
            {
                FastMergeSortTokenWithElite(sortTokenBufferPtr, individalCount, eliteCount);
                SortSortTokenWithIndex(sortTokenBufferPtr, eliteCount);
                CopyEliteGeneToHead(rawDataReadWriteArrayPtr, sortTokenBufferPtr, eliteCount, GeneDynamic1.GetRawLength(geneSnippetCount));
            }

            /// <summary>
            ///  Move the elite sorttoken to array head;
            ///  It looks like from fastSort,but would not complete sort! it just can get the num of biggest fitness of eliteCount and move forward to array;
            ///  WatchMore: https://leetcode-cn.com/problems/zui-xiao-de-kge-shu-lcof/
            /// </summary>
            /// <param name="sortTokenBufferPtr"></param>
            /// <param name="length"></param>
            /// <param name="eliteCount"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void FastMergeSortTokenWithElite(SortToken* sortTokenBufferPtr, int length, int eliteCount)
            {
                int leftRange = 0;
                int rightRange = length - 1;

                while (true)
                {
                    int left = leftRange;
                    int right = rightRange;

                    SortToken target = sortTokenBufferPtr[leftRange];
                    while (left < right)
                    {                       
                        while (left < right && sortTokenBufferPtr[right] <= target) right--;
                        while (left < right && sortTokenBufferPtr[left] >= target) left++;
                        if (left < right && sortTokenBufferPtr[left] < sortTokenBufferPtr[right])
                        {
                            Swap(sortTokenBufferPtr, left, right);
                        }

                    }
                    if (left != leftRange)
                    {
                        Swap(sortTokenBufferPtr, left, leftRange);
                    }

                    if (left < eliteCount - 1)
                    {
                        leftRange = left + 1;
                    }
                    else if (left > eliteCount - 1)
                    {
                        rightRange = left - 1;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            /// <summary>
            ///  Sort sorttoken as its index
            ///  used to copy raw data
            /// </summary>
            /// <param name="sortTokenBufferPtr"></param>
            /// <param name="eliteCount"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void SortSortTokenWithIndex(SortToken* sortTokenBufferPtr, int eliteCount)
            {
                for (int i = 0; i < eliteCount; i++)
                {
                    for (int j0 = i+1; j0 < eliteCount; j0++)
                    {
                        SortToken origin = sortTokenBufferPtr[i];
                        SortToken target = sortTokenBufferPtr[j0];
                        if (origin.index> target.index)
                        {
                            Swap(sortTokenBufferPtr, i, j0);
                        }
                    }
                }
            }
            /// <summary>
            /// Write fitness to gene and copy it to rawData head;
            /// </summary>
            /// <param name="rawDataReadWriteArrayPtr"></param>
            /// <param name="sortTokenBufferPtr"></param>
            /// <param name="eliteCount"></param>
            /// <param name="geneRawLength"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void CopyEliteGeneToHead(GeneDynamic1* rawDataReadWriteArrayPtr,SortToken* sortTokenBufferPtr, int eliteCount, int geneRawLength)
            {
                for (int i = 0; i < eliteCount; i++)
                {
                    SortToken sortToken = sortTokenBufferPtr[i];
                    GeneDynamic1 copyFrom = rawDataReadWriteArrayPtr[sortToken.index];
                    GeneDynamic1 copyTo = rawDataReadWriteArrayPtr[i];

                    copyFrom.fitness = sortToken.value;//OYM:rewrite fitness to elite gene;

                    MemCpy(copyFrom, copyTo, geneRawLength);
                }
            }
            /// <summary>
            /// Generate all individual gene
            /// </summary>
            /// <param name="tempGeneBufferPtr"></param>
            /// <param name="eliteGeneCount"></param>
            /// <param name="mutationGeneCount"></param>
            /// <param name="crossoverGeneCount"></param>
            /// <param name="randomGeneCount"></param>
            /// <param name="geneSnippetCount"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void ReGenerateIndivualData(GeneDynamic1* tempGeneBufferPtr, int eliteGeneCount, int mutationGeneCount, int crossoverGeneCount, int randomGeneCount,int geneSnippetCount, int randomSeed)
            {
                int begin = 0;
/*                for (int i = 0; i < eliteGeneCount; i++)
                {
                    //OYM:do nothing
                }*/
                begin += eliteGeneCount;
                for (int i = begin; i < begin+ mutationGeneCount; i++)
                {

                    //OYM:mutation
                    GeneDynamic1 target = tempGeneBufferPtr[i];
                    int originIndex = i % eliteGeneCount;
                    GeneDynamic1 originElite = tempGeneBufferPtr[originIndex];

                    MutationGene(originElite, target,  geneSnippetCount, randomSeed+i);

                    for (int j0 = 0; j0 < i; j0++)
                    {
                        if (tempGeneBufferPtr[i].Equals(tempGeneBufferPtr[j0]))
                        {

                        }
                    }
                }
                begin += mutationGeneCount;
                for (int i = begin; i < begin + crossoverGeneCount; i++)
                {
                    //OYM:crossover
                    GeneDynamic1 target = tempGeneBufferPtr[i];
                    int originIndex1 = i/eliteGeneCount;
                    int originIndex2 = i % eliteGeneCount;

                    GeneDynamic1 originElite1 = tempGeneBufferPtr[originIndex1];
                    GeneDynamic1 originElite2 = tempGeneBufferPtr[originIndex2+ eliteGeneCount];
                    CrossoverGene(originElite1, originElite2, target, geneSnippetCount, (uint)(randomSeed + i));

                    for (int j0 = 0; j0 < i; j0++)
                    {
                        if (tempGeneBufferPtr[i].Equals(tempGeneBufferPtr[j0]))
                        {

                        }
                    }
                }
                begin += crossoverGeneCount;
                for (int i = begin; i < begin+ randomGeneCount; i++)
                {
                    GeneDynamic1 target = tempGeneBufferPtr[i];
                    RandomizeGene(target, geneSnippetCount,(uint)( randomSeed + i));

                    for (int j0 = 0; j0 < i; j0++)
                    {
                        if (tempGeneBufferPtr[i].Equals(tempGeneBufferPtr[j0]))
                        {

                        }
                    }
                }


            }


        }

        /// <summary>
        /// Other function like move ptr or copy memory
        /// </summary>
        public partial struct GeneticAlgorizomComputeJob
        {
            //OYM:Be careful when you want to move your ptr
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static uint* Move(uint* targetPtr, int length)
            {
                return targetPtr + length;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static SortToken* Move(SortToken* targetPtr, int length)
            {
                return targetPtr + length;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ulong* Move(ulong* targetPtr, int length)
            {
                return targetPtr + length;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static RigidTransform* Move(RigidTransform* targetPtr, int length)
            {
                return targetPtr + length;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static GeneDynamic1* Move(GeneDynamic1* targetPtr, int length) //OYM:在burst里面用泛型掉性能。。。。唉
            {
                return targetPtr + length;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Swap(SortToken* array, int a, int b) //OYM:在burst里面用泛型掉性能。。。。唉
            {
                SortToken temp = array[a];
                array[a] = array[b];
                array[b] = temp;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void MemCpy(GeneDynamic1 origin, GeneDynamic1 destination, int geneRawLength)
            {
                UnsafeUtility.MemCpy(destination.GetUnsafePtr(), origin.GetUnsafePtr(), geneRawLength * ULONG_SIZE);
            }

            private static void ComputeWorldTransform(GeneDynamic1 currentGene, GeneticJoint* geneticJointPtr, RigidTransform* transformTempArrayPtr, int jointCount)
            {
                for (int jointIndex = 0; jointIndex < jointCount; jointIndex++)//OYM:潜在的burst bug，如果这里被for循环展开优化，可能会出现顺序丢失导致值为0的问题
                {
                    float4 localFitness = 0;

                    GeneticJoint currentJoint = geneticJointPtr[jointIndex];
                    RigidTransform currentTransform = new RigidTransform();

                    ulong rawData = currentGene[jointIndex];
                    float4 transcriptionData = GeneticMath.UlongTofloat4(rawData);

                    switch (currentJoint.JointMode)
                    {
                        case JointMode.Position:
                            {
                                //OYM:正常而言应该是先去寻找父节点的位置,再来计算这个
                                //OYM:但是父节点存在和位置计算很迷(说实话我自己也没想好这个root节点到底该怎么处理pos,一直置零吗?)
                                float3 moveFloat3 = GeneticMath.NomalizeTofloat3(transcriptionData, 2, currentJoint.MinRange, currentJoint.MaxRange);
                                currentTransform.pos = moveFloat3;
                                currentTransform.rot = quaternion.identity;
                            }
                            break;
                        case JointMode.Rotation:
                            {
                                //OYM:get parent data
                                int parentIndex = currentJoint.parentIndex;
                                RigidTransform parentTransform = transformTempArrayPtr[parentIndex];
                                //OYM:get current gene rotate
                                float3 euler = GeneticMath.NomalizeTofloat3(transcriptionData, 1, currentJoint.MinRange, currentJoint.MaxRange);
                                quaternion rotateEuler = quaternion.Euler(euler);
                                //OYM:clac and set transform
                                currentTransform.pos = math.mul(parentTransform.rot, currentJoint.localPosition);
                                currentTransform.rot = math.mul(rotateEuler, math.mul(currentJoint.localRotation, parentTransform.rot));
                            }
                            break;
                        default:
                            break;
                    }
                    transformTempArrayPtr[jointIndex] = currentTransform;
                }
            }
        }

        #endregion
        /// <summary>
        /// Statistics the best gene and transcirpt to transform data;
        /// </summary>
        [BurstCompile]
        public struct PoseprocessGeneticComputeJob : IJob
        {
            /// <summary>
            /// Gene rawData
            /// </summary>
            [NativeDisableUnsafePtrRestriction]
            public ulong* rawEliteDataPtr;
            /// <summary>
            /// the the joint transform of the best fitness
            /// </summary>
            [WriteOnly,NativeDisableUnsafePtrRestriction]
            public RigidTransform* targetWorldTramsformPtr;
            /// <summary>
            ///  the skeleton controller point
            ///  has at least 44 rotation control joint and 1 position control joint 
            /// </summary>
            [ReadOnly, NativeDisableUnsafePtrRestriction]
            public GeneticJoint* geneticJointPtr;
            /// <summary>
            /// best fitness ,used to check how does genetic algorizom happend internal.
            /// </summary>
            [WriteOnly, NativeDisableUnsafePtrRestriction]
            public float* bestFitnessPtr;
            /// <summary>
            /// the raw elite gene's count;
            /// </summary>
            public int allEliteGeneCount;
            /// <summary>
            /// means how many gene snippets in gene.
            /// is NOT a raw gene data length because a raw gene data contain a fitness data and gene snippetdata 
            /// </summary>
            public int geneSnippetCount;
            /// <summary>
            ///  Random Seed
            /// </summary>
            public Random random;

            public void Execute()
            {
                GetBestFitnessGene(rawEliteDataPtr, allEliteGeneCount, geneSnippetCount, out GeneDynamic1 bestGene);
                ComputeWorldTransform(bestGene, geneticJointPtr, targetWorldTramsformPtr, geneSnippetCount);
            }
            /// <summary>
            ///  Create temp data
            ///  WARNING: you should DISPOSE your native collection on jobs after you don't need that.
            /// </summary>
            /// <param name="individalCount"></param>
            /// <param name="geneRawLength"></param>
            /// <param name="tempGeneBufferPtr"></param>
            /// <param name="tempsGenes"></param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void GetBestFitnessGene(ulong* rawEliteDataPtr, int allEliteGeneCount,int geneSnippetCount,
                out GeneDynamic1 bestGene)
            {
                float bestFitness = float.MinValue;
                int rawGeneLength = GeneDynamic1.GetRawLength(geneSnippetCount);
                int stride = rawGeneLength * ULONG_SIZE;

                int bestIndex = -1;
                for (int i = 0; i < allEliteGeneCount; i++)
                {
                    float fitness = UnsafeUtility.ReadArrayElementWithStride<float>(rawEliteDataPtr, i, stride);
                    if (bestFitness<fitness)
                    {
                        bestIndex = i;
                    }
                }

                ulong* bestGenePtr = Move(rawEliteDataPtr, bestIndex * rawGeneLength);
                bestGene = new GeneDynamic1(bestGenePtr, geneSnippetCount);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void ComputeWorldTransform(GeneDynamic1 currentGene, GeneticJoint* geneticJointPtr, RigidTransform* transformTempArrayPtr, int jointCount)
            {
                for (int jointIndex = 0; jointIndex < jointCount; jointIndex++)//OYM:潜在的burst bug，如果这里被for循环展开优化，可能会出现顺序丢失导致值为0的问题
                {
                    float4 localFitness = 0;

                    GeneticJoint currentJoint = geneticJointPtr[jointIndex];
                    RigidTransform currentTransform = new RigidTransform();

                    ulong rawData = currentGene[jointIndex];
                    float4 transcriptionData = GeneticMath.UlongTofloat4(rawData);

                    switch (currentJoint.JointMode)
                    {
                        case JointMode.Position:
                            {
                                //OYM:正常而言应该是先去寻找父节点的位置,再来计算这个
                                //OYM:但是父节点存在和位置计算很迷(说实话我自己也没想好这个root节点到底该怎么处理pos,一直置零吗?)
                                float3 moveFloat3 = GeneticMath.NomalizeTofloat3(transcriptionData, 2, currentJoint.MinRange, currentJoint.MaxRange);
                                currentTransform.pos = moveFloat3;
                                currentTransform.rot = quaternion.identity;
                            }
                            break;
                        case JointMode.Rotation:
                            {
                                //OYM:get parent data
                                int parentIndex = currentJoint.parentIndex;
                                RigidTransform parentTransform = transformTempArrayPtr[parentIndex];
                                //OYM:get current gene rotate
                                float3 euler = GeneticMath.NomalizeTofloat3(transcriptionData, 1, currentJoint.MinRange, currentJoint.MaxRange);
                                quaternion rotateEuler = quaternion.Euler(euler);
                                //OYM:clac and set transform
                                currentTransform.pos = math.mul(parentTransform.rot, currentJoint.localPosition);
                                currentTransform.rot = math.mul(rotateEuler, math.mul(currentJoint.localRotation, parentTransform.rot));
                            }
                            break;
                        default:
                            break;
                    }
                    transformTempArrayPtr[jointIndex] = currentTransform;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ulong* Move(ulong* targetPtr, int length)
            {
                return targetPtr + length;
            }
        }

        /// <summary>
        /// Re-write transform data to transform 
        /// </summary>
        [BurstCompile]
        public struct ReWriteResultToTransformJob : IJobParallelForTransform
        {
            [NativeDisableUnsafePtrRestriction]
            public RigidTransform* targetWorldTramsformPtr;

            public void Execute(int index, TransformAccess transform)
            {
                transform.position = targetWorldTramsformPtr[index].pos;
                transform.rotation = targetWorldTramsformPtr[index].rot;
            }
        }
    }
}