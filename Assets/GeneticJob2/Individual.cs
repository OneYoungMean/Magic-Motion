using Unity.Mathematics;

namespace BIOIK2
{
    internal class Individual
    {
        public float3[] genes;
        public float3[] momentum;
        public float fitness;
        public float extinction;

        public Individual(int dimension)
        {
            genes = new float3[dimension];
            momentum = new float3[dimension];
            fitness = 0;
            extinction = 0;
        }

    }
}