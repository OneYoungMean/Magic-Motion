using System.Collections.Generic;
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

        public  List<BioNode> nodes=new List<BioNode>();
        public List<ObjectivePtr> objectivePtrs=new List<ObjectivePtr>();
        public List<MotionPtr> motionPtrs = new List<MotionPtr>();

        internal float3 originPosition;
        internal quaternion originRotation;
        internal float3 originScale;

        private float[] Configuration;
        private float[] Gradient;
        private float[] Losses;
    }
}