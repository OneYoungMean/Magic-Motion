using UnityEngine;

namespace BIOIK2CSharp
{
    class BIOIK2
    {

    }

    public struct Frame
    {
        public Vector3 pos;
        public Quaternion rot;
    }
    public struct IKParams
    {
        public int mode;
        public bool enable_counter;
        public int threads;
        public uint randomSeed;
        public float dpos;
        public float drot;
        public float dtwist;
    }
    public struct PoseGoal
    {
        bool position_only_ik;
        float rotation_scale;
    }
    public struct CenterJointsGoal
    {
        float weight;
    }
    public struct AvoidJointLimitsGoal
    {
        float weight;
    }
    public struct MinimalDisplacementGoal
    {
        float weight;
    }
}
