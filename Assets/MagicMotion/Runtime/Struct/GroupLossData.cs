using Unity.Mathematics;
using System;
namespace MagicMotion
{
    /// <summary>
    /// Transform to ConstraintData
    /// </summary>

    public struct GroupLossData:IComparable<GroupLossData>
    {
        public int index;
        public double loss;

        public int CompareTo(GroupLossData other)
        {
            return loss.CompareTo(other.loss);
        }

        public override string ToString()
        {
            return index.ToString()+"-"+loss.ToString();
        }
    }
}