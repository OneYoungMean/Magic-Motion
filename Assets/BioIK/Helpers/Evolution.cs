using UnityEngine;
using System.Threading;
using System.Collections.Generic;

namespace BIOIK
{

    //----------------------------------------------------------------------------------------------------
    //====================================================================================================
    //Memetic Evolutionary Optimisation
    //====================================================================================================
    //----------------------------------------------------------------------------------------------------
    public class Evolution {		
		private Model Model;                 		    		//Reference to the kinematic model
		private int PopulationSize;                 			//Number of individuals (population size)
		private int Elites;                     				//Number of elite individuals
		private int Dimensionality;                  			//Search space dimensionality

		private double[] LowerBounds;               			//Constraints for the lower bounds
		private double[] UpperBounds; 		                    //Constraints for the upper bounds

		private Individual[] Population;                    	//Array for current individuals
		private Individual[] Offspring;							//Array for offspring individuals

		private List<Individual> Pool = new List<Individual>();	//Selection pool for recombination
		private int PoolCount;									//Current size of the selection pool
		private double[] Probabilities;							//Current probabilities for selection
		private double Gene;									//Simple storage variable #1
		private double Weight;									//Simple storage variable #2
		private bool[] Constrained;

        //Variables for elitism exploitation
        private bool UseThreading;
        private bool Evolving = false;
        private bool[] Improved = null;
        private Model[] Models = null;
        private BFGS[] Optimisers = null;

        //Threading
		private bool ThreadsRunning = false;
        private ManualResetEvent[] Handles = null;
        private Thread[] Threads = null;
	    private bool[] Work = null;
        
        //Variables for optimisation queries
		private double[] Solution;                       		//Evolutionary solution
		private double Fitness;                  				//Evolutionary fitness

        private bool Killed = false;

		//Initialises the algorithm
		public Evolution(Model model, int populationSize, int elites, bool useThreading) {
			Model = model;
			PopulationSize = populationSize;
			Elites = elites;
			Dimensionality = model.GetDoF();
            //UseThreading = useThreading;
            UseThreading = false;

			Population = new Individual[PopulationSize];
			Offspring = new Individual[PopulationSize];
			for(int i=0; i<PopulationSize; i++) {
				Population[i] = new Individual(Dimensionality);
				Offspring[i] = new Individual(Dimensionality);
			}

			LowerBounds = new double[Dimensionality];
			UpperBounds = new double[Dimensionality];
			Constrained = new bool[Dimensionality];
			Probabilities = new double[PopulationSize];
			Solution = new double[Dimensionality];

            Models = new Model[Elites];
            Optimisers = new BFGS[Elites];
            Improved = new bool[Elites];
            for(int i=0; i<Elites; i++) {
                int index = i;
                Models[index] = new Model(Model.GetCharacter());
                Optimisers[index] = new BFGS(Dimensionality, x => Models[index].ComputeLoss(x), y => Models[index].ComputeGradient(y, 1e-5));
            }

            if(UseThreading) {
                //Start Threads
                ThreadsRunning = true;
                Work = new bool[Elites];
                Handles = new ManualResetEvent[Elites];
                Threads = new Thread[Elites];
                for(int i=0; i<Elites; i++) {
                    int index = i;
                    Work[index] = false;
                    Handles[index] = new ManualResetEvent(true);
                    Threads[index] = new Thread(x => SurviveThread(index));
                    //Threads[index].Start();
                }
            } else {
                ThreadsRunning = false;
            }
		}

        ~Evolution() {
            Kill();
        }

        public void Kill() {
            if(Killed) {
                return;
            }
            Killed = true;
            if(UseThreading) {
                //Stop Threads
                ThreadsRunning = false;
                for(int i=0; i<Elites; i++) {
                    if(Threads[i].IsAlive) {
                        Handles[i].Set();
                        Threads[i].Join();
                    }
                }
            }
        }

		public double[] Optimise(int generations, double[] seed) {
            if(UseThreading) {
                for(int i=0; i<Elites; i++) {
                    if(!Threads[i].IsAlive) {
                        Threads[i].Start();
                    }
                }
            }

			Model.Refresh();
            
			for(int i=0; i<Dimensionality; i++) {
				LowerBounds[i] = Model.MotionPtrs[i].Motion.GetLowerLimit(true);
				UpperBounds[i] = Model.MotionPtrs[i].Motion.GetUpperLimit(true);
                Constrained[i] = Model.MotionPtrs[i].Motion.Constrained;
				Solution[i] = seed[i];
			}
			Fitness = Model.ComputeLoss(Solution);

			if(!Model.CheckConvergence(Solution)) {
				Initialise(seed);
				for(int i=0; i<Elites; i++) {
					Models[i].CopyFrom(Model);
					Optimisers[i].LowerBounds = LowerBounds;
					Optimisers[i].UpperBounds = UpperBounds;
				}
				for(int i=0; i<generations; i++) {
					//for(int i=0; i<25; i++) { //Performance testing
					Evolve();
				}
			}

			return Solution;
		}

		//Initialises the population with the seed
		private void Initialise(double[] seed) {
			for(int i=0; i<Dimensionality; i++) {
				Population[0].Genes[i] = seed[i];
				Population[0].Momentum[i] = 0.0;
			}
			Population[0].Fitness = Model.ComputeLoss(Population[0].Genes);

			for(int i=1; i<PopulationSize; i++) {
				Reroll(Population[i]);
			}

			SortByFitness();

            ComputeExtinctions();

			TryUpdateSolution();
		}

		//One evolutionary cycle
		private void Evolve() {
			//Create mating pool
			Pool.Clear();
			Pool.AddRange(Population);
			PoolCount = PopulationSize;

            if(UseThreading) {
                //Evolve offspring
                Evolving = true;

                //Exploit elites in threads
                for(int i=0; i<Elites; i++) {
                    Handles[i].Set();
                }

                //Evolve non elites sequentially
                for(int i=Elites; i<PopulationSize; i++) {
                    if(PoolCount > 0) {
                        Individual parentA = Select(Pool);
                        Individual parentB = Select(Pool);
                        Individual prototype = Select(Pool);

                        //Recombination and Adoption
                        Reproduce(Offspring[i], parentA, parentB, prototype);

                        //Pre-Selection Niching
                        if(Offspring[i].Fitness < parentA.Fitness) {
                            Pool.Remove(parentA);
                            PoolCount -= 1;
                        }
                        if(Offspring[i].Fitness < parentB.Fitness) {
                            Pool.Remove(parentB);
                            PoolCount -= 1;
                        }
                    } else {
                        //Fill the population
                        Reroll(Offspring[i]);
                    }
                }

                //Finish evolving
                Evolving = false;

                //Wait for threads to finish
                //System.DateTime timer = Utility.GetTimestamp();
                while(HasWork()) {
                    //Wait one cycle
                    /*
                    if(Utility.GetElapsedTime(timer) > 0.5f) {
                        Debug.Log("!!! !!! !!! FIXING CRASH !!! !!! !!!");
                        for(int i=0; i<Work.Length; i++) {
                            Debug.Log("Work: " + i + " : " + Work[i]);
                        }
                        break;
                    }
                    */
                }

            } else {
                //Evolve offspring
                System.DateTime timestamp = Utility.GetTimestamp();
                for(int i=Elites; i<PopulationSize; i++) {
                    if(PoolCount > 0) {
                        Individual parentA = Select(Pool);
                        Individual parentB = Select(Pool);
                        Individual prototype = Select(Pool);

                        //Recombination and Adoption
                        Reproduce(Offspring[i], parentA, parentB, prototype);

                        //Pre-Selection Niching
                        if(Offspring[i].Fitness < parentA.Fitness) {
                            Pool.Remove(parentA);
                            PoolCount -= 1;
                        }
                        if(Offspring[i].Fitness < parentB.Fitness) {
                            Pool.Remove(parentB);
                            PoolCount -= 1;
                        }
                    } else {
                        //Fill the population
                        Reroll(Offspring[i]);
                    }
                }
                double duration = Utility.GetElapsedTime(timestamp);

                //Exploit elites sequentially
                for(int i=0; i<Elites; i++) {
                    SurviveSequential(i, duration);
                }
            }

			//Reroll elite if exploitation was not successful
			for(int i=0; i<Elites; i++) {
				if(!Improved[i]) {
					Reroll(Offspring[i]);
				}
			}

			//Swap population and offspring
			Swap(ref Population, ref Offspring);

			//Finalise
			SortByFitness();

			//Check improvement and wipeout criterion
			if(!TryUpdateSolution() && !HasAnyEliteImproved()) {
				Initialise(Solution);
			} else {
                ComputeExtinctions();
            }
		}

		//Returns the mutation probability from two parents
		private double GetMutationProbability(Individual parentA, Individual parentB) {
			double extinction = 0.5 * (parentA.Extinction + parentB.Extinction);
			double inverse = 1.0/(double)Dimensionality;
			return extinction * (1.0-inverse) + inverse;
		}

		//Returns the mutation strength from two parents
		private double GetMutationStrength(Individual parentA, Individual parentB) {
			return 0.5 * (parentA.Extinction + parentB.Extinction);
		}

		//Computes the extinction factors for all individuals
		private void ComputeExtinctions() {
			double min = Population[0].Fitness;
			double max = Population[PopulationSize-1].Fitness;
			for(int i=0; i<PopulationSize; i++) {
				double grading = (double)i/((double)PopulationSize-1);
				Population[i].Extinction = (Population[i].Fitness + min*(grading-1.0)) / max;
			}
		}

        private bool HasWork() {
            for(int i=0; i<Elites; i++) {
                if(Work[i]) {
                    return true;
                }
            }
            return false;
        }

		//Returns whether any elite could be improved by the exploitation
		private bool HasAnyEliteImproved() {
			for(int i=0; i<Elites; i++) {
				if(Improved[i]){
					return true;
				}
			}
			return false;
		}

		//Tries to improve the evolutionary solution by the population, and returns whether it was successful
		private bool TryUpdateSolution() {
			double candidateFitness = Population[0].Fitness;
			if(candidateFitness < Fitness) {
				for(int i=0; i<Dimensionality; i++) {
					Solution[i] = Population[0].Genes[i];
				}
				Fitness = candidateFitness;
				return true;
			} else {
				return false;
			}
		}

        private void SurviveThread(int index) {
			while(ThreadsRunning) {
                Work[index] = true;

                //Copy elitist survivor
                Individual survivor = Population[index];
                Individual elite = Offspring[index];
                for(int i=0; i<Dimensionality; i++) {
                    elite.Genes[i] = survivor.Genes[i];
                    elite.Momentum[i] = survivor.Momentum[i];
                }

                //Exploit
                double fitness = Models[index].ComputeLoss(elite.Genes);
                Optimisers[index].Minimise(elite.Genes, ref Evolving);
                if(Optimisers[index].Value < fitness) {
                    for(int i=0; i<Dimensionality; i++) {
                        elite.Momentum[i] = Optimisers[index].Solution[i] - elite.Genes[i];
                        elite.Genes[i] = Optimisers[index].Solution[i];
                    }
                    elite.Fitness = Optimisers[index].Value;
                    Improved[index] = true;
                } else {
                    elite.Fitness = fitness;
                    Improved[index] = false;
                }
                
                Handles[index].Reset();
                
                //Finish
                Work[index] = false;

                Handles[index].WaitOne();
            }
        }

		private void SurviveSequential(int index, double timeout) {
            //Copy elitist survivor
            Individual survivor = Population[index];
            Individual elite = Offspring[index];
            for(int i=0; i<Dimensionality; i++) {
                elite.Genes[i] = survivor.Genes[i];
                elite.Momentum[i] = survivor.Momentum[i];
            }

            //Exploit
            double fitness = Models[index].ComputeLoss(elite.Genes);
            Optimisers[index].Minimise(elite.Genes, timeout);
            if(Optimisers[index].Value < fitness) {
                for(int i=0; i<Dimensionality; i++) {
                    elite.Momentum[i] = Optimisers[index].Solution[i] - elite.Genes[i];
                    elite.Genes[i] = Optimisers[index].Solution[i];
                }
                elite.Fitness = Optimisers[index].Value;        
                Improved[index] = true;
            } else {
                elite.Fitness = fitness;
                Improved[index] = false;
            }
		}

		//Evolves a new individual
		private void Reproduce(Individual offspring, Individual parentA, Individual parentB, Individual prototype) {
            double mutationProbability = GetMutationProbability(parentA, parentB);
			double mutationStrength = GetMutationStrength(parentA, parentB);

			for(int i=0; i<Dimensionality; i++) {
				//Recombination
				Weight = Random.value;
				double momentum = Random.value * parentA.Momentum[i] + Random.value * parentB.Momentum[i];
				offspring.Genes[i] = Weight*parentA.Genes[i] + (1.0-Weight)*parentB.Genes[i] + momentum;

				//Store
				Gene = offspring.Genes[i];

				//Mutation
                if(Constrained[i]) {
                    if(Random.value < mutationProbability) {
                        offspring.Genes[i] += (Random.value * (UpperBounds[i] - LowerBounds[i]) + LowerBounds[i]) * mutationStrength;
                    }
                }

				//Adoption
				Weight = Random.value;
				offspring.Genes[i] += 
					Weight * Random.value * (0.5 * (parentA.Genes[i] + parentB.Genes[i]) - offspring.Genes[i])
					+ (1.0-Weight) * Random.value * (prototype.Genes[i] - offspring.Genes[i]);

				//Project
                if(Constrained[i]) {
                    if(offspring.Genes[i] < LowerBounds[i]) {
                        offspring.Genes[i] = LowerBounds[i];
                    }
                    if(offspring.Genes[i] > UpperBounds[i]) {
                        offspring.Genes[i] = UpperBounds[i];
                    }
                }

				//Momentum
				offspring.Momentum[i] = Random.value * momentum + (offspring.Genes[i] - Gene);
			}

			//Fitness
			offspring.Fitness = Model.ComputeLoss(offspring.Genes);
		}

		//Generates a random individual
		private void Reroll(Individual individual) {
			for(int i=0; i<Dimensionality; i++) {
				if(Constrained[i]) {
					individual.Genes[i] = (double)Random.Range((float)LowerBounds[i], (float)UpperBounds[i]);
					individual.Momentum[i] = 0.0;
				} else {
					individual.Genes[i] = Solution[i];
					individual.Momentum[i] = 0.0;
				}
			}
			individual.Fitness = Model.ComputeLoss(individual.Genes);
		}

		//Rank-based selection of an individual
		private Individual Select(List<Individual> pool) {
			double rankSum = (double)(PoolCount*(PoolCount+1)) / 2.0;
			for(int i=0; i<PoolCount; i++) {
				Probabilities[i] = (double)(PoolCount-i)/rankSum;
			}
			return pool[GetRandomWeightedIndex(Probabilities, PoolCount)];
		}
		
		//Returns a random index with respect to the probability weights
		private int GetRandomWeightedIndex(double[] probabilities, int count) {
			double weightSum = 0.0;
			for(int i=0; i<count; i++) {
				weightSum += probabilities[i];
			}
			double rVal = Random.value * weightSum;
			for(int i=0; i<count; i++) {
				rVal -= probabilities[i];
				if(rVal <= 0.0) {
					return i;
				}
			}
			return count-1;
		}

        /*
		//Projects a single gene to stay within search space
		private double Project(double gene, double min, double max, bool constrained) {
			if(max - min == 0.0) {
				return 0.0;
			}
			if(type == JointType.Revolute || type == JointType.Prismatic) {
				//Bounce
				while(gene < min || gene > max) {
					if(gene > max) {
						gene = max + max - gene;
					}
					if(gene < min) {
						gene = min + min - gene;
					}
				}
			}
			if(type == JointType.Continuous) {
				//Overflow
				while(gene < -PI || gene > PI) {
					if(gene > PI) {
						gene -= PI + PI;
					}
					if(gene < -PI) {
						gene += PI + PI;
					}
				}
			}
			if(type == JointType.Floating) {
				//nothing
			}
            return gene;
		}
        */

		//Sorts the population by their fitness values (descending)
		private void SortByFitness() {
			System.Array.Sort(Population,
				delegate(Individual a, Individual b) {
					return a.Fitness.CompareTo(b.Fitness);
				}
			);
		}

		//In-place swap by references
		private void Swap(ref Individual[] a, ref Individual[] b) {
			Individual[] tmp = a;
			a = b;
			b = tmp;
		}

        public Model GetModel() {
            return Model;
        }

        public double[] GetSolution() {
            return Solution;
        }

        public double GetFitness() {
            return Fitness;
        }

        public double[] GetLowerBounds() {
            return LowerBounds;
        }

        public double[] GetUpperBounds() {
            return UpperBounds;
        }

		public double[,] GetGeneLandscape() {
			double[,] values = new double[PopulationSize, Dimensionality];
			for(int i=0; i<PopulationSize; i++) {
				for(int j=0; j<Dimensionality; j++) {
					values[i,j] = Population[i].Genes[j];
				}
			}
			return values;
		}

		public double[] GetFitnessLandscape() {
			double[] values = new double[PopulationSize];
			for(int i=0; i<PopulationSize; i++) {
				values[i] = Population[i].Fitness;
			}
			return values;
		}

		//Data class for the individuals
		public class Individual {
			public double[] Genes;
			public double[] Momentum;
			public double Fitness;
            public double Extinction;

			public Individual(int dimensionality) {
				Genes = new double[dimensionality];
				Momentum = new double[dimensionality];
				Fitness = 0.0;
                Extinction = 0.0;
			}
		}
	}

    ///   Limited-memory Broyden–Fletcher–Goldfarb–Shanno (L-BFGS) optimization method.

    ///   The L-BFGS algorithm is a member of the broad family of quasi-Newton optimization
    ///   methods. L-BFGS stands for 'Limited memory BFGS'. Indeed, L-BFGS uses a limited
    ///   memory variation of the Broyden–Fletcher–Goldfarb–Shanno (BFGS) update to approximate
    ///   the inverse Hessian matrix (denoted by Hk). Unlike the original BFGS method which
    ///   stores a dense  approximation, L-BFGS stores only a few vectors that represent the
    ///   approximation implicitly. Due to its moderate memory requirement, L-BFGS method is
    ///   particularly well suited for optimization problems with a large number of variables.
    ///   L-BFGS never explicitly forms or stores Hk. Instead, it maintains a history of the past
    ///   m updates of the position x and gradient g, where generally the history
    ///   m can be short, often less than 10. These updates are used to implicitly do operations
    ///   requiring the Hk-vector product.

    ///   The framework implementation of this method is based on the original FORTRAN source code
    ///   by Jorge Nocedal (see references below). The original FORTRAN source code of L-BFGS (for
    ///   unconstrained problems) is available at http://www.netlib.org/opt/lbfgs_um.shar and had
    ///   been made available under the public domain.
    /// 
    ///   References:
    ///        http://www.netlib.org/opt/lbfgs_um.shar
    ///        Jorge Nocedal. Limited memory BFGS method for large scale optimization (Fortran source code). 1990.
    ///        Available in http://www.netlib.org/opt/lbfgs_um.shar
    ///        Jorge Nocedal. Updating Quasi-Newton Matrices with Limited Storage. Mathematics of Computation,
    ///        Vol. 35, No. 151, pp. 773--782, 1980.
    ///        Dong C. Liu, Jorge Nocedal. On the limited memory BFGS method for large scale optimization.
    


}