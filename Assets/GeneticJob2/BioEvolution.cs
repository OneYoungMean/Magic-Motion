
using System.Collections.Generic;
using System.Linq;
using System;
using System.Threading;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace BioIK2
{
    using static Utility;
    public unsafe class BioEvolution:IDisposable
    {
        private BioModel bioModel;
        private int populationSize;
        private int elites;
        private int Dimensionality;
        private NativeArray< float3> lowerBounds;
        private NativeArray< float3> upperBounds;
        public  NativeArray< float3> solutions;

        private Individual[] population;

        private Individual[] offSpring;

        private List<Individual> pool = new List<Individual>();

        private int poolCount;

        private float[] probabilities;

        private float3 geneTemp;

        private float weight;

        private NativeArray<float3> constrained;

        private bool useThreading;

        private bool evolving = false;

        private bool[] improved = null;

        private BioModel[] models = null;

        private BIOIK.BFGS_F[] optimisers = null;
        private BroydenFletcherGoldfarbShanno[] optimisers2 = null;
        private float Fitness;
        private Random random;
        //OYM: threading;

        private bool threadsRunning = false;
        public BioEvolution(BioModel bioModel, int populationSize, int elites, bool useThreading)
        {
            this.bioModel = bioModel;
            this.populationSize = populationSize;
            this.elites = elites;
            this.useThreading = useThreading;
            this.Dimensionality = bioModel.GetDof3();

            population=new Individual[populationSize]; 
            offSpring = new Individual[populationSize];
            for (int i = 0; i < populationSize; i++)
            {
                population[i] = new Individual(Dimensionality);
                offSpring[i] = new Individual(Dimensionality);
            }

            lowerBounds = new NativeArray<float3>(Dimensionality,Allocator.Persistent);
            upperBounds = new NativeArray<float3>(Dimensionality, Allocator.Persistent);
            constrained = new NativeArray<float3>(Dimensionality, Allocator.Persistent);
            probabilities = new float[populationSize];
            solutions = new NativeArray<float3>(Dimensionality, Allocator.Persistent);

            models =new BioModel[elites];
            optimisers = new BIOIK.BFGS_F[elites];
            optimisers2 = new BroydenFletcherGoldfarbShanno[elites];
            improved = new bool[elites];
            for (int i = 0; i < elites; i++)
            {
                int index = i;
                models[index] = new BioModel(bioModel.GetCharacter());
/*                optimisers[index] = new BIOIK.BFGS_F(Dimensionality*3, x => models[index].ComputeLoss(x.ToFloat3Array()), y => models[index].ComputeGradient(y, 1e-5f));*/
                optimisers2[index] = new BroydenFletcherGoldfarbShanno(Dimensionality * 3, x => models[index].ComputeLoss(x), y => models[index].ComputeGradient(y, 1e-5f));
            }


            random = new Random(1); 
        }

        ~BioEvolution()
        {
            Dispose();
        }

        internal BioModel GetModel()
        {
            return bioModel;
        }
        public NativeArray<float3> GetSolution()
        {
            return solutions;
        }

        public NativeArray<float3> GetLowerBounds()
        {
            return lowerBounds;
        }

        public NativeArray<float3> GetUpperBounds()
        {
            return upperBounds;
        }

        internal NativeArray<float3> Optimise(int generations)
        {
            bioModel.Refresh();

            for (int i = 0; i < Dimensionality; i++)
            {
                lowerBounds[i] = bioModel.motionPtrs[i].Motion.GetLowerLimit();
                upperBounds[i] = bioModel.motionPtrs[i].Motion.GetUpperLimit();
                constrained[i] = bioModel.motionPtrs[i].Motion.constraintValue;
            }
            Fitness = bioModel.ComputeLoss(solutions);

            if (!bioModel.CheckConvergence(solutions))
            {
                Initialise(solutions);
                for (int i = 0; i < elites; i++)
                {
                    models[i].CopyFrom(bioModel);

                   UnsafeUtility.MemCpy( optimisers2[i].UpperBounds .GetUnsafePtr(),upperBounds.GetUnsafePtr(), upperBounds.Length*UnsafeUtility.SizeOf<float3>());
                    UnsafeUtility.MemCpy(optimisers2[i].LowerBounds.GetUnsafePtr(), lowerBounds.GetUnsafePtr(), lowerBounds.Length * UnsafeUtility.SizeOf<float3>());
                }
                for (int i = 0; i < generations; i++)
                {
                    //for(int i=0; i<25; i++) { //Performance testing
                    Evolve();
                }
            }
            return solutions;
        }


        private void Evolve()
        {
            pool.Clear();
            pool.AddRange(population);
            poolCount = populationSize;

            System.DateTime timestamp = Utility.GetTimestamp();
            for (int i = elites; i < populationSize; i++)
            {
                if (poolCount > 0)
                {
                    Individual parentA = Select(pool);
                    Individual parentB = Select(pool);
                    Individual prototype = Select(pool);

                    //Recombination and Adoption
                    Reproduce(offSpring[i], parentA, parentB, prototype);

                    //Pre-Selection Niching
                    if (offSpring[i].fitness < parentA.fitness)
                    {
                        pool.Remove(parentA);
                        poolCount -= 1;
                    }
                    if (offSpring[i].fitness < parentB.fitness)
                    {
                        pool.Remove(parentB);
                        poolCount -= 1;
                    }
                }
                else
                {
                    //Fill the population
                    Reroll(offSpring[i]);
                }
            }
            float duration = Utility.GetElapsedTime(timestamp);
            for (int i = 0; i < elites; i++)
            {
                SurviveSequential(i, duration);
            }
            for (int i = 0; i < elites; i++)
            {
                if (!improved[i])
                {
                    Reroll(offSpring[i]); //OYM：无法优化了，创建一个新的基因
                }
            }
            Swap(ref population, ref offSpring);
            SortByFitness();

            if (!TryUpdateSolution() && !HasAnyEliteImproved())
            {
                Initialise(solutions);
            }
            else
            {
                ComputeExtinctions();
            }
        }

        private bool HasAnyEliteImproved()
        {
            for (int i = 0; i < elites; i++)
            {
                if (improved[i])
                {
                    return true;
                }
            }
            return false;
        }

        private void Swap(ref Individual[] a, ref Individual[] b)
        {
            Individual[] tmp = a;
            a = b;
            b = tmp;
        }

        private void SurviveSequential(int index, float timeout)
        {
            //Copy elitist survivor
            Individual survivor = population[index];
            Individual elite = offSpring[index];

            elite.genes.CopyFrom( survivor.genes);
            Array.Copy(survivor.momentum, elite.momentum, Dimensionality);
            float fitness = models[index].ComputeLoss(elite.genes);

            //OYM：两者之间收敛速度差不多
            //OYM：但是后者可读性吊打前者
            //OYM：我要做一下nativeArray，修一下GC
            /*            optimisers[index].Minimise(elite.genes.ToFloatArray(), (float)timeout);

                        if (optimisers[index].Value < fitness)
                        {
                            var optimGene = optimisers[index].Solution.ToFloat3Array();
                            for (int i = 0; i < Dimensionality; i++)
                            {
                                elite.momentum[i] = optimGene[i] - elite.genes[i];
                                elite.genes[i] = optimGene[i];
                            }
                            elite.fitness = optimisers[index].Value;
                            improved[index] = true;
                        }
                        else
                        {
                            elite.fitness = fitness;
                            improved[index] = false;
                        }*/
            optimisers2[index].Minimize(elite.genes);

            if (optimisers2[index].Value < fitness)
            {
                var newSolution = optimisers2[index].Solution;
                for (int i = 0; i < Dimensionality; i++)
                {
                    float3 target = new float3(newSolution[i * 3], newSolution[i * 3 + 1], newSolution[i * 3 + 2]);
                    elite.momentum[i] =  - elite.genes[i];
                    elite.genes[i] = target;
                }
                elite.fitness = optimisers2[index].Value;
                improved[index] = true;
            }
            else
            {
                elite.fitness = fitness;
                improved[index] = false;
            }
        }

            //OYM：前处理
            private void Reproduce(Individual offSpring, Individual parentA, Individual parentB, Individual prototype)
        {
            float mutationProbability = GetMutationProbability(parentA, parentB);
            float mutationStrength = GetMutationStrength(parentA, parentB);

            //OYM：Recombination
            for (int i = 0; i < Dimensionality; i++)
            {
                weight = random.NextFloat();
                float3 momentum = random.NextFloat3() *parentA.momentum[i]+ random.NextFloat3()* parentB.momentum[i];

                offSpring.genes[i] += math.lerp(parentA.genes[i] , parentB.genes[i], weight) + momentum;

                //Store
                geneTemp = offSpring.genes[i];

                //OYM：mutation
                if (random.NextFloat() < mutationProbability)
                {
                    float3 constraintedValue = (float3)constrained[i];
                    offSpring.genes[i] += math.lerp(lowerBounds[i], upperBounds[i], random.NextFloat3()) * constraintedValue * mutationStrength;
                }

                //OYM：Adoption
                weight = random.NextFloat();
                float3 avgGeneLoss = 0.5f * (parentA.genes[i] + parentB.genes[i])- offSpring.genes[i];
                float3 protoGeneLoss =  prototype.genes[i] - offSpring.genes[i];
                offSpring.genes[i] += math.lerp(avgGeneLoss, protoGeneLoss, weight);
                //Project

                offSpring.genes[i]=math.clamp(offSpring.genes[i],lowerBounds[i], upperBounds[i]);
            }
            //OYM：
            offSpring.fitness = bioModel.ComputeLoss(offSpring.genes);
        }

        private float GetMutationProbability(Individual parentA, Individual parentB)
        {
            float extinction = 0.5f * (parentA.extinction + parentB.extinction);
            float inverse = 1.0f / (float)Dimensionality;
            return extinction * (1.0f - inverse) + inverse;
        }
        private float GetMutationStrength(Individual parentA, Individual parentB)
        {
            return 0.5f * (parentA.extinction + parentB.extinction);
        }
        //OYM：选择权重
        private Individual Select(List<Individual> pool)
        {
            float rankSum = (poolCount * (poolCount + 1)) / 2.0f;//OYM：等差数列求和
            for (int i = 0; i < poolCount; i++)
            {
                probabilities[i] = (float)(poolCount - i) / rankSum; //OYM：归一化之后的值
            }
            return pool[GetRandomWeightedIndex(probabilities,poolCount)];
        }
        /// <summary>
        /// 依次递减直到rVal为零为止
        /// 话说这东西不应该有个通项之类的吗。。。。
        /// </summary>
        /// <param name="probabilities"></param>
        /// <param name="poolCount"></param>
        /// <returns></returns>
        private int GetRandomWeightedIndex(float[] probabilities, int poolCount)
        {
            float weightSum = 0.0f;
            for (int i = 0; i < poolCount; i++)
            {
                weightSum += probabilities[i]; //OYM：这里应该是1对吧
            }
            float rVal =random.NextFloat() * weightSum;
            for (int i = 0; i < poolCount; i++)
            {
                rVal -= probabilities[i];
                if (rVal <= 0.0f)
                {
                    return i;
                }
            }
            return poolCount - 1;
        }

        private void Initialise(NativeArray<float3> seeds)
        {
            for (int i = 0; i < Dimensionality; i++)
            {
                population[0].genes[i] = seeds[i];
                population[0].momentum[i] = 0.0f;
            }
            population[0].fitness = bioModel.ComputeLoss(population[0].genes);

            for (int i = 1; i < populationSize; i++)
            {
                Reroll(population[i]);
            }

            SortByFitness();

            ComputeExtinctions();

            TryUpdateSolution();
        }

        private void SortByFitness()
        {
            System.Array.Sort(population, (a, b) => a.fitness.CompareTo(b.fitness));
        }
        private void ComputeExtinctions()
        {
            float min = population[0].fitness;
            float max = population[populationSize - 1].fitness;
            for (int i = 0; i < populationSize; i++)
            {
                float grading = (float)i / ((float)populationSize - 1);
                population[i].extinction = (population[i].fitness + min * (grading - 1.0f)) / max;
            }
        }
        private bool TryUpdateSolution()
        {
            float candidateFitness = population[0].fitness;
            if (candidateFitness < Fitness)
            {
                solutions.CopyFrom(population[0].genes) ;
                Debug.Log(Fitness + "~" + candidateFitness);
                Fitness = candidateFitness;

                return true;
            }
            else
            {
                return false;
            }
        }
        private void Reroll(Individual individual)
        {
            for (int i = 0; i < Dimensionality; i++)
            {
                float3 constrianedValue = (float3)constrained[i];
                individual.genes[i] = random.NextFloat3(lowerBounds[i], upperBounds[i]);
            }
            individual.fitness = bioModel.ComputeLoss(individual.genes);
        }

        public void Dispose()
        {
            lowerBounds.Dispose();
            upperBounds.Dispose();
            solutions.Dispose();
            constrained.Dispose();
        }
    }
}