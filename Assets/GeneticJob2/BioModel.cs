using Unity.Mathematics;

namespace BIOIK2
{
    public class BioModel
    {
        private Bioik2 Character;

        private BioSegment root;

        private float3 positionOffset;
        private quaternion rotationOffset;
        private float3 scaleOffset;

        public BioNode[] nodes;
    }
}