using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion
{
    #region Gene
    /// <summary>
    /// The Gene,containing 64x8byte(ulong),like array of ulong;
    /// 0-56 is joint data ,57 is empty but occupy data. 
    /// 63 is fitness data
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public unsafe struct Gene_Safe : System.IEquatable<Gene_Safe>, IComparable<Gene_Safe>, IFormattable
    {
        /// <summary>
        /// array length
        /// </summary>
        public const int Length = 64;
        public const int ULONG_SIZE = 8;
        public const int SIZE_OF_STRUCT = 512;

        public ulong First => start;
        public ulong Last => end;

        public double Fitness
        {
            get
            {
                return ((double*)GetUnsafePtr())[Length];
            }
            set
            {
                ((double*)GetUnsafePtr())[Length] = value;
            }
        }



        [FieldOffset(0)]
        private ulong start;
        [FieldOffset(SIZE_OF_STRUCT - ULONG_SIZE)]
        private ulong end;

        #region DataStore
        /*        ulong a0;
                ulong a1;
                ulong a2;
                ulong a3;
                ulong a4;
                ulong a5;
                ulong a6;
                ulong a7;
                ulong a8;
                ulong a9;
                ulong b0;
                ulong b1;
                ulong b2;
                ulong b3;
                ulong b4;
                ulong b5;
                ulong b6;
                ulong b7;
                ulong b8;
                ulong b9;
                ulong c0;
                ulong c1;
                ulong c2;
                ulong c3;
                ulong c4;
                ulong c5;
                ulong c6;
                ulong c7;
                ulong c8;
                ulong c9;
                ulong d0;
                ulong d1;
                ulong d2;
                ulong d3;
                ulong d4;
                ulong d5;
                ulong d6;
                ulong d7;
                ulong d8;
                ulong d9;
                ulong e0;
                ulong e1;
                ulong e2;
                ulong e3;
                ulong e4;
                ulong e5;
                ulong e6;
                ulong e7;
                ulong e8;
                ulong e9;
                ulong f0;
                ulong f1;
                ulong f2;
                ulong f3;
                ulong f4;
                ulong f5;
                ulong f6;
                ulong f7;
                ulong f8;
                ulong f9;
                ulong g0;
                ulong g1;
                ulong g2;
                ulong g3;*/
        #endregion
        unsafe public ulong this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((ulong)index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                fixed (ulong* array = &start) { return array[index]; }
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                fixed (ulong* array = &start) { array[index] = value; }
            }
        }

        public void* GetUnsafePtr()
        {
            fixed (void* ptr = &start) return ptr;
        }
        public bool Equals(Gene_Safe other)
        {
            return UnsafeUtility.MemCmp(other.GetUnsafePtr(), GetUnsafePtr(), SIZE_OF_STRUCT) == 0;
        }

        public string ToString(string format, IFormatProvider formatProvider)
        {
            return $"Length ={Length}, Fitness ={Fitness},First={First},Last={Last}";
        }

        public int CompareTo(Gene_Safe other)
        {
            if (Fitness > other.Fitness)
            {
                return 1;
            }
            else if (Fitness < other.Fitness)
            {
                return -1;
            }
            else return 0;
        }


    }
    #endregion
}
