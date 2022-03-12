using System;
using System.Collections.Generic;
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

        private Individual[] population;

        private Individual[] offSpring;

        private List<Individual> pool = new List<Individual>();

        private int poolCount;

        private double geneTemp;

        private double weight;

        private bool[] constrained;

        private bool useThreading;

        private bool evolving = false;

        private bool[] improved = null;

        private BioModel[] models = null;

        private BFGS_F[] optimisers = null;

        //OYM: threading;

        private bool threadsRunning = false;
        public BioEvolution(BioModel bioModel, int populationSize, int elites, bool useThreading)
        {
            this.bioModel = bioModel;
            this.populationSize = populationSize;
            this.elites = elites;
            this.useThreading = useThreading;
        }

        internal void Kill()
        {
            throw new NotImplementedException();
        }
    }
}