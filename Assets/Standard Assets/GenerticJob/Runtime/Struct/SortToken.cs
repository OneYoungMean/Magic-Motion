using System;

namespace GeneticJob
{
    public struct SortToken : IComparable<SortToken>
    {
       public int index;
       public float value;

        public int CompareTo(SortToken other)
        {
            if (value > other.value)
            {
                return 1;
            }
            else if (value < other.value)
            {
                return -1;
            }
            else return 0;
        }

        public static bool operator <(SortToken lhs, SortToken rhs)
        {
            return lhs.value < rhs.value;
        }
        public static bool operator >(SortToken lhs, SortToken rhs)
        {
            return lhs.value > rhs.value;
        }
        public static bool operator <=(SortToken lhs, SortToken rhs)
        {
            return lhs.value <= rhs.value;
        }
        public static bool operator >=(SortToken lhs, SortToken rhs)
        {
            return lhs.value >= rhs.value;
        }

        public override string ToString()
        {
            return $"index: {index},value: {value}";
        }
    }

}
