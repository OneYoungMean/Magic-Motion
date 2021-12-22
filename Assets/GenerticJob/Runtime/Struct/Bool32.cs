using System;
namespace GeneticJob
{
    public struct Bool32
    {
        public uint uintValue;

        public bool this[int index]
        {
            get
            {
#if UNITY_EDITOR
                if (index < 0 || index >= 32)
                {
                    throw new InvalidOperationException();
                }
#endif
                return (uintValue & 1u << index) == 0;
            }


            set
            {

#if UNITY_EDITOR
                if (index < 0 || index >= 32)
                {
                    throw new InvalidOperationException();
                }
#endif

                if (value)
                {
                    uintValue = uintValue | 1u << index;
                }
                else
                {
                    uintValue = ~(~uintValue | 1u << index);
                }
            }
        }

        public override string ToString()
        {
            return GeneticMath.UintToBinary(uintValue);
        }

    }

}
