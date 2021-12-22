using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace GeneticJob
{
    using static GeneticMath;
    using Random = Unity.Mathematics.Random;

    /// <summary>
    /// Fixed size Gene,fast and useful if you only have static genetic question.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct GeneFixed
    {
        public const int Length = 32;
        [FieldOffset(0)] public float fitness;
        [FieldOffset(FLOAT_SIZE)] public Unity.Mathematics.Random random;
        [FieldOffset(FLOAT_SIZE+UINT_SIZE)] public uint First;
        [FieldOffset(FLOAT_SIZE + UINT_SIZE + UINT_SIZE * Length)] public uint Last;

        unsafe public uint this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                fixed (uint* array = &First) { return array[index]; }
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                fixed (uint* array = &First) { array[index] = value; }
            }
        }

        public int GetSize()
        {
            return FLOAT_SIZE + UINT_SIZE + UINT_SIZE * Length;
        }
    }

    /// <summary>
    /// Unfixed size Gene,a little bit more difficult ,but useful as the fixed
    /// uint rawdata
    /// </summary>
    public unsafe struct GeneDynamic : System.IEquatable<GeneDynamic>,IFormattable
    {
        /// <summary>
        /// Rawdata ptr
        /// </summary>
        private uint* rawGeneData;
        /// <summary>
        /// gene snippet count
        /// </summary>
        public readonly int Length;
        /// <summary>
        ///  fitness for gene
        /// </summary>
        public float fitness => *(float*)rawGeneData;
        /// <summary>
        ///   gene header ,used to check 
        /// </summary>
        public uint First => this[0];
        /// <summary>
        ///   gene tail ,used to check 
        /// </summary>
        public uint Last => this[Length - 1];
        /// <summary>
        /// get random 
        /// NOTE:If you want to more strict random (in science,industry etc. ),just insert it in rawGeneData like fitness in.
        /// </summary>
        public Random random  { get { return new Random((uint)rawGeneData); }}

        public GeneDynamic( uint* GenePtr, int Length)
        {
            rawGeneData = GenePtr;
            this.Length = Length;
        }
         public uint this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if (index<0||index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                return rawGeneData[index+1];
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if (index < 0 || index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                rawGeneData[index+1] = value;
            }
        }
        public uint* GetUnsafePtr()
        {
            return rawGeneData;
        }
        public uint* GetSnippetGenePtr()
        {
            return rawGeneData + 1;
        }
        public bool Equals(GeneDynamic other)
        {
            bool result = false;
            if (Length==other.Length)
            {
                result = true;
                for (int i = 0; i < Length; i++)
                {
                    if (other[i] != this[i])
                    {
                        result = false;
                        break;

                    }
                }
            }
            return result;
        }

        public string ToString(string format, IFormatProvider formatProvider)
        {
            return $"Length ={Length}, Fitness ={fitness},First={First},Last={Last}";
        }

        /// <summary>
        ///  Get raw Data size ,used to copy memory
        /// </summary>
        /// <param name="geneLength"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetSize(int geneLength)
        {
            return  GetRawLength (geneLength) * UINT_SIZE;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetRawLength(int geneLength)
        {
            return (geneLength + 1) ;
        }
    }

    /// <summary>
    /// Unfixed size Gene,a little bit more difficult ,but useful as the fixed
    /// ulong rawData(maybe fasty?)
    /// </summary>
    public unsafe struct GeneDynamic1 : System.IEquatable<GeneDynamic1>, IFormattable
    {
        /// <summary>
        /// Rawdata ptr
        /// </summary>
        private ulong* rawGeneData;
        /// <summary>
        /// gene snippet count,mostly it used to make safey check;
        /// </summary>
        public readonly int Length;
        /// <summary>
        ///  fitness for gene
        /// </summary>
        public float fitness
        {
            get { return *(float*)rawGeneData; }
            set { *(float*)rawGeneData = value; }
        }
        public Random random
        {
            get { return new Random(*(uint*)(rawGeneData+1)+(uint)rawGeneData); }
            set { *(uint*)(rawGeneData + 1) = value.state; }
        }

        /// <summary>
        ///   gene header ,used to check 
        /// </summary>
        public ulong First => this[0];
        /// <summary>
        ///   gene tail ,used to check 
        /// </summary>
        public ulong Last => this[Length - 1];
        /// <summary>
        /// get random 
        /// NOTE:If you want to more strict random (in science,industry etc. ),just insert it in rawGeneData like fitness in.
        /// </summary>

        public GeneDynamic1(ulong* GenePtr, int Length)
        {
            rawGeneData = GenePtr;
            this.Length = Length;
        }
        public ulong this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if (index < 0 || index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                return rawGeneData[index + 1];
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if (index < 0 || index >= Length)
                    throw new System.ArgumentException($"index must be between[0...{Length}]");
#endif
                rawGeneData[index + 1] = value;
            }
        }
        public ulong* GetUnsafePtr()
        {
            return rawGeneData;
        }
        public ulong* GetSnippetGenePtr()
        {
            return rawGeneData+1;
        }

        public bool Equals(GeneDynamic1 other)
        {
            bool result = false;
            if (Length == other.Length)
            {
                result = UnsafeUtility.MemCmp(other.GetSnippetGenePtr(), GetSnippetGenePtr(), Length * ULONG_SIZE)==0;
            }
            return result;
        }

        public string ToString(string format, IFormatProvider formatProvider)
        {
            return $"Length ={Length}, Fitness ={fitness},First={First},Last={Last}";
        }

        /// <summary>
        ///  Get raw Data size ,used to copy memory
        /// </summary>
        /// <param name="geneLength"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetSize(int geneLength)
        {
            return GetRawLength(geneLength) * ULONG_SIZE;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetRawLength(int geneLength)
        {
            return (geneLength + 1);
        }
    }
    //OYM:Safey but slow
    /*    public unsafe struct Gene_Safe : System.IEquatable<Gene_Safe>, IComparable<Gene_Safe>, IFormattable
        {
           public float fitness;
            public uint First => a0;
            public uint Last => d1;
            public const int Length = 32;

            //OYM:Add more behind this if you want more gene....
            uint a0;
            uint a1;
            uint a2;
            uint a3;
            uint a4;
            uint a5;
            uint a6;
            uint a7;
            uint a8;
            uint a9;
            uint b0;
            uint b1;
            uint b2;
            uint b3;
            uint b4;
            uint b5;
            uint b6;
            uint b7;
            uint b8;
            uint b9;
            uint c0;
            uint c1;
            uint c2;
            uint c3;
            uint c4;
            uint c5;
            uint c6;
            uint c7;
            uint c8;
            uint c9;
            uint d0;
            uint d1;

            unsafe public uint this[int index]
            {
                get
                {
    #if ENABLE_UNITY_COLLECTIONS_CHECKS
                    if ((uint)index >= Length)
                        throw new System.ArgumentException($"index must be between[0...{Length}]");
    #endif
                    fixed (uint* array = &a0) { return array[index]; }
                }
                set
                {
    #if ENABLE_UNITY_COLLECTIONS_CHECKS
                    if ((uint)index >= Length)
                        throw new System.ArgumentException($"index must be between[0...{Length}]");
    #endif
                    fixed (uint* array = &a0) { array[index] = value; }
                }
            }

            public uint* GetUnsafePtr()
            {
                fixed (uint* ptr = &a0) return ptr;
            }
            public bool Equals(Gene_Safe other)
            {
                return this.a0 == other.a0 &&
                              this.a1 == other.a1 &&
                              this.a2 == other.a2 &&
                              this.a3 == other.a3 &&
                              this.a4 == other.a4 &&
                              this.a5 == other.a5 &&
                              this.a6 == other.a6 &&
                              this.a7 == other.a7 &&
                              this.a8 == other.a8 &&
                              this.a9 == other.a9 &&
                              this.b0 == other.b0 &&
                              this.b1 == other.b1 &&
                              this.b2 == other.b2 &&
                              this.b3 == other.b3 &&
                              this.b4 == other.b4 &&
                              this.b5 == other.b5 &&
                              this.b6 == other.b6 &&
                              this.b7 == other.b7 &&
                              this.b8 == other.b8 &&
                              this.b9 == other.b9 &&
                              this.c0 == other.c0 &&
                              this.c1 == other.c1 &&
                              this.c2 == other.c2 &&
                              this.c3 == other.c3 &&
                              this.c4 == other.c4 &&
                              this.c5 == other.c5 &&
                              this.c6 == other.c6 &&
                              this.c7 == other.c7 &&
                              this.c8 == other.c8 &&
                              this.c9 == other.c9 &&
                              this.d0 == other.d0 &&
                              this.d1 == other.d1;
            }

            public string ToString(string format, IFormatProvider formatProvider)
            {
                return $"Length ={Length}, Fitness ={fitness},First={First},Last={Last}";
            }

            public int CompareTo(Gene_Safe other)
            {
                if (fitness > other.fitness)
                {
                    return 1;
                }
                else if (fitness < other.fitness)
                {
                    return -1;
                }
                else return 0;
            }
        }*/
}