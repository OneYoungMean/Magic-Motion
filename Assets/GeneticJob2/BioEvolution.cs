using BIOIK;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Unity.Mathematics;

namespace BIOIK2
{
    public class BioEvolution
    {
        private BioModel bioModel;
        private int populationSize;
        private int elites;
        private int Dimensionality;
        private float3[] lowerBounds;
        private float3[] upperBounds;
        public float3[] solutions;

        private Individual[] population;

        private Individual[] offSpring;

        private List<Individual> pool = new List<Individual>();

        private int poolCount;

        private float[] probabilities;

        private double geneTemp;

        private double weight;

        private bool3[] constrained;

        private bool useThreading;

        private bool evolving = false;

        private bool[] improved = null;

        private BioModel[] models = null;

        private BFGS_F[] optimisers = null;

        private float3[] Solution;                               
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
            random = new Random(1); 
        }

        internal void Kill()
        {
        }

        internal BioModel GetModel()
        {
            return bioModel;
        }

        internal float3[] Optimise(int generations, float3[] seeds)
        {
            bioModel.Refresh();

            for (int i = 0; i < Dimensionality; i++)
            {
                lowerBounds[i] = bioModel.motionPtrs[i].Motion.GetLowerLimit(true);
                upperBounds[i] = bioModel.motionPtrs[i].Motion.GetUpperLimit(true);
                constrained[i] = bioModel.motionPtrs[i].Motion.constraint;
                solutions[i] = seeds[i];
            }
            Fitness = bioModel.ComputeLoss(Solution);

            if (!bioModel.CheckConvergence(Solution))
            {
                Initialise(seeds);
                for (int i = 0; i < elites; i++)
                {
                    models[i].CopyFrom(bioModel);
                    optimisers[i].LowerBounds = lowerBounds.SelectMany(x=>new float[] { x.x,x.y,x.z}).ToArray();
                    optimisers[i].UpperBounds = upperBounds.SelectMany(x => new float[] { x.x, x.y, x.z }).ToArray();
                }
                for (int i = 0; i < generations; i++)
                {
                    //for(int i=0; i<25; i++) { //Performance testing
                    Evolve();
                }
            }
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
                    Reproduce(Offspring[i], parentA, parentB, prototype);

                    //Pre-Selection Niching
                    if (Offspring[i].Fitness < parentA.Fitness)
                    {
                        Pool.Remove(parentA);
                        PoolCount -= 1;
                    }
                    if (Offspring[i].Fitness < parentB.Fitness)
                    {
                        Pool.Remove(parentB);
                        PoolCount -= 1;
                    }
                }
                else
                {
                    //Fill the population
                    Reroll(Offspring[i]);
                }
            }
        }

        private Individual Select(List<Individual> pool)
        {
            float rankSum = (float)(poolCount * (poolCount + 1)) / 2.0f;
            for (int i = 0; i < poolCount; i++)
            {
                probabilities[i] = (float)(PoolCount - i) / rankSum;
            }
            return pool[GetRandomWeightedIndex(Probabilities, PoolCount)];
        }

        private void Initialise(float3[] seeds)
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
                for (int i = 0; i < Dimensionality; i++)
                {
                    Solution[i] = population[0].genes[i];
                }
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
                individual.genes[i] = random.NextFloat3(lowerBounds[i], upperBounds[i])* constrianedValue + Solution[i]*(1- constrianedValue);
            }
            individual.fitness = bioModel.ComputeLoss(individual.genes);
        }
    }
}