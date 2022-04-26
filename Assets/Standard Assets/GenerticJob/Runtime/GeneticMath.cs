using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using Random = Unity.Mathematics.Random;
using System.Runtime.CompilerServices;

namespace GeneticJob
{
    public unsafe partial class GeneticMath
    {
        //constant
        public const int FLOAT_SIZE = 4;
        public const int UINT_SIZE = 4;
        public const int ULONG_SIZE = 8;
        
    }
    public unsafe partial class GeneticMath
    {
        //OYM: Transcription

        /// <summary>
        /// Precision:1/255
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ByteTofloat(byte target)
        {
            float result = target / (float)byte.MaxValue;
            return result;
        }
        /// <summary>
        /// Precision:1/65536
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float UshortTofloat(ushort target)
        {
            float result = target / (float)ushort.MaxValue;
            return result;
        }
        /// <summary>
        /// Precision:1/4,294,967,295
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        //OYM:base
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float UintTofloat(uint target)
        {
            float result = target / (float)uint.MaxValue;
            return result;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        //OYM:base
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Bool32 UintToBool32(uint target)
        {
            Bool32 bool32 = new Bool32();
            bool32.uintValue = target;
            return bool32;
        }

        /// <summary>
        /// Precision:at least 1/1023 (xy is 1/2047,z is 1/1023)
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 UintTofloat3(uint target)
        {
            float x = (target & 0x000007FFu) / (float)(0x000007FFu);
            float y = (target & 0x003FF800u) / (float)(0x003FF800u);
            float z = (target & 0xFFC00000u) / (float)(0xFFC00000u);
            return new float3(x, y, z);
        }
        /// <summary>
        /// Precision:1/255
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 UintTofloat4(uint target)
        {
            float x = (target & 0x000000FFu) / (float)(0x000000FFu);
            float y = (target & 0x0000FF00u) / (float)(0x0000FF00u);
            float z = (target & 0x00FF0000u) / (float)(0x00FF0000u);
            float w = (target & 0xFF000000u) / (float)(0xFF000000u);
            return new float4(x, y, z, w);
        }

        /// <summary>
        /// Precision:at least 1/2,097,151(z is 1/4,194,303)
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 UlongTofloat3(ulong target)
        {
            float x = (target & 0x0000_0000_001F_FFFFu) / (float)(0x0000_0000_001F_FFFFu);
            float y = (target & 0x0000_03FF_FFE0_0000u) / (float)(0x0000_03FF_FFE0_0000u);
            float z = (target & 0xFFFF_FC00_0000_0000u) / (float)(0xFFFF_FC00_0000_0000u);
            return new float3(x, y, z);
        }
        /// <summary>
        /// Precision:1/65,536
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 UlongTofloat4(ulong target)
        {
            float x = (target & 0x000000000000FFFFu) / (float)(0x000000000000FFFF);
            float y = (target & 0x00000000FFFF0000u) / (float)(0x00000000FFFF0000u);
            float z = (target & 0x0000FFFF00000000u) / (float)(0x0000FFFF00000000u);
            float w = (target & 0xFFFF000000000000u) / (float)(0xFFFF000000000000u);
            return new float4(x, y, z, w);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static byte FloatToByte(float target)
        {
#if UNITY_EDITOR
            if (target > 1 || target < 0)
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                byte result = (byte)((target) * byte.MaxValue);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ushort FloatToUShort(float target)
        {
#if UNITY_EDITOR
            if (target > 1 || target < 0)
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                ushort result = (ushort)((target) * ushort.MaxValue);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint FloatToUint(float target)
        {
#if UNITY_EDITOR
            if (target > 1 || target < 0)
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                uint result = (uint)((target) * uint.MaxValue);
                return result;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Bool32ToUint(Bool32 target)
        {
            return target.uintValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Float3ToUint(float3 target)
        {
#if UNITY_EDITOR
            if (math.any( target > 1 | target < 0))
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                uint result = (uint)(0x000007FFu * target.x )+ (uint)(0x003FF800u * target.y) + (uint)(0xFFC00000u * target.z);

                    //(uint)math.dot(target, new float3(0x000007FFu, 0x003FF800u, 0xFFC00000u));
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Float4ToUint(float4 target)
        {
#if UNITY_EDITOR
            if (math.any(target > 1 | target < 0))
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                uint result = (uint)(0x000000FFu * target.x) + (uint)(0x0000FF00u * target.y) + (uint)(0x00FF0000u * target.z) + (uint)(0xFF000000u * target.w);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong Float3ToUlong(float3 target)
        {
#if UNITY_EDITOR
            if (math.any(target > 1 | target < 0))
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                ulong result = (ulong)(0x0000_0000_001F_FFFFu * target.x) + (ulong)(0x0000_03FF_FFE0_0000u * target.y) + (ulong)(0xFFFF_FC00_0000_0000u * target.z);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong Float4ToUlong(float4 target)
        {
#if UNITY_EDITOR
            if (math.any(target > 1 | target < 0))
            {
                throw new ArgumentOutOfRangeException();
            }
            checked //OYM:会溢出吗？
#endif
            {
                ulong result = (ulong)(0x000000000000FFFFu * target.x) + (ulong)(0x00000000FFFF0000u * target.y) + (ulong)(0x0000FFFF00000000u * target.z) + (ulong)(0xFFFF000000000000u * target.w);
                return result;
            }
        }


    }


    public unsafe partial class GeneticMath
    {
        //OYM:Mutation

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Mutation_FilpBit(uint target,int index)
        {
            target ^= 1u << index;
            return target;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Mutation_FilpBitFast(uint target)
        {
            target ^= 0xAAAAAAAA + target;

            return target;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong Mutation_FilpBitFast(ulong target)
        {
            target ^= 0xAAAA_AAAA_AAAA_AAAA + target;

            return target;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Mutation_FilpUint(uint target)
        {
            target = ~target;
            return target;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Mutation_ReverseUint(uint target)
        {
            target = math.reversebits(target);
            return target;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Mutation_FastMutationUint(uint target)
        {
            target = target * 0x745ED837u + 0x816EFB5Du;
            return target;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Mutation_DisplacementUint(uint target,int value)
        {
            uint high = target << value;
            uint low = target >> value;
            target = high + low;
            return target;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Mutation_ReverseArray(uint* target, int targetLength, float mutation, Random random)
        {
            if (random.NextFloat() < mutation)
            {
                Reverse(target, 0, targetLength - 1);
            }
        }


        //OYM:https://blog.csdn.net/qq_26399665/article/details/79831490
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Mutation_ShuffleArray(uint* target, int targetLength)
        {
            Random random = Random.CreateFromIndex((uint)(target)); 
            for (int i = 0; i < targetLength - 1; i++)
            {
                int swapIndex = random.NextInt() % (targetLength - i);
                Swap(target + swapIndex, target + (targetLength - 1 - i));
            }
        }
        //OYM:https://blog.csdn.net/weixin_43251547/article/details/102992904
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Mutation_DisplacementArray(uint* target, int targetLength,int line)
        {
            if (line != 0 &&line<targetLength)
            {
                Reverse(target, 0, targetLength - 1);
                Reverse(target, 0, line - 1);
                Reverse(target, line, targetLength - 1);
            }
#if UNITY_EDITOR
            else
            {
                throw new InvalidOperationException("line must in [1,targetLength]");
            }
#endif
        }

    }

    public unsafe partial class GeneticMath
    {
        //OYM:Crossover

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint Crossover_CrossUintFast(uint targetA, uint targetB)
        {
            targetA = targetA & 0xAAAAAAAA;
            targetB = targetB & 0x55555555;
            return targetA | targetB;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Crossover_CycleCrossArray(uint* a, uint* b, uint* target, int length)
        {

            for (int i = 0; i < length / 2; i++)
            {
                target[i * 2] = a[i * 2];
                target[i * 2 + 1] = b[i * 2 + 1];

            }
            if (length % 2 == 1)
            {
                target[length - 1] = a[length - 1];
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Crossover_CycleCrossArray(ulong* a, ulong* b, ulong* target, int length, Random random)
        {

            for (int i = 0; i < length; i++)
            {
                bool bo = random.NextBool();
                    target[i] = bo ?
                    a[i]:
                    b[i];
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Crossover_OnePointCrossArray(uint* a, uint* b, uint* target, int crossPoint, int length)
        {
#if UNITY_EDITOR
            if (crossPoint >= length)
            {
                throw new InvalidOperationException();
            }
#endif
            for (int i = 0; i < length; i++)
            {
                if (i < crossPoint)
                {
                    target[i] = a[i];
                }
                else
                {
                    target[i] = b[i];
                }
            }

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Crossover_TowPointCrossArray(uint* a, uint* b, uint* target, int point1, int point2, int length)
        {
#if UNITY_EDITOR
            if (point1 >= length || point2 > length)
            {
                throw new InvalidOperationException();
            }
#endif
            int minPoint = math.min(point1, point2);
            int maxPoint = math.max(point1, point2);
            for (int i = 0; i < length; i++)
            {
                if (i < minPoint || i > maxPoint)
                {
                    target[i] = a[i];
                }
                else
                {
                    target[i] = b[i];
                }

            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Randomize_RandomGene(uint* GeneInneerPtr, Random randomsPtr, int geneSnippetCount)
        {
            for (int i = 0; i < geneSnippetCount; i++)
            {
                var RandomValue = randomsPtr.NextUInt();
                GeneInneerPtr[i] = RandomValue;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Randomize_RandomGene(ulong* GeneInneerPtr, Random random, int geneSnippetCount)
        {
            for (int i = 0; i < geneSnippetCount; i++)
            {
                ulong RandomValue = (ulong)random.NextUInt()*uint.MaxValue+ random.NextUInt();
                GeneInneerPtr[i] = RandomValue;
            }
        }
    }

    public unsafe partial class GeneticMath
    {
        //OYM:Other

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Swap(uint* a, uint* b)
        {
            uint temp = a[0];
            a[0] = b[0];
            b[0] = temp;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        public static void Reverse(uint* target, int start, int end)
        {
#if UNITY_EDITOR
            if (start < 0 || end < start)
            {
                throw new InvalidOperationException();
            }
#endif
            int targetLength = start - end + 1;
            for (int i = 0; i < targetLength / 2; i++)
            {
                int begin = start + i;
                int tail = end - i;
                Swap(target + begin, target + tail);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string UintToBinary(uint n)
        {
            if (n < 2) return n.ToString();

            var divisor = n / 2;
            var remainder = n % 2;

            return UintToBinary(divisor) + remainder;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 NomalizeTofloat3(float4 value, int norm, float3 MinValue, float3 MaxValue)
        {
#if (UNITY_EDITOR)
            {
                if (math.any(value > 1 | value < 0))
                {
                    throw new Exception("value out of range(0,1) :" + value);
                }
            }
#endif
            float3 result = MinValue;
            if (!math.all(value.xyz == 0)&& value.w!=0)
            {
                switch (norm)
                {
                    case 0:
                        result= math.lerp(MinValue, MaxValue, value.xyz);
                        break;
                    case 1:
                        float3 xyz = value.xyz / math.csum(value.xyz)* value.w;
                        result= math.lerp(MinValue, MaxValue, xyz) ;
                        break;
                    case 2:
                        float3 normal = value.xyz / math.length(value.xyz) * value.w;
                        result= math.lerp(MinValue, MaxValue, normal) ;
                        break;
                    default:
                        break;
                }
            }
            return result;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3 TranscritptToEuler(float3 value,float3 MinEuler, float3 MaxEuler)
        {
#if (UNITY_EDITOR)
            {
                if (math.any(value > 1 | value < 0))
                {
                    Debug.LogError("value out of range(0,1) :" + value);
                    return 0;
                }
            }
#endif
            return math.lerp(MaxEuler, MinEuler, value);
        }
    }
}
