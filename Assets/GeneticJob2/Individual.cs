using Unity.Collections;
using Unity.Mathematics;

namespace BIOIK2
{
    internal class Individual
    {
        public NativeArray <float3> genes;
        public float3[] momentum;//OYM：梯度
        public float fitness;
        public float extinction;


        public Individual(int dimension)
        {
            genes = new NativeArray<float3>(dimension,Allocator.Persistent);
            momentum = new float3[dimension];
            fitness = 0;
            extinction = 0;
        }

        public void Dispose()
        {
            genes.Dispose();
        }
    }
}