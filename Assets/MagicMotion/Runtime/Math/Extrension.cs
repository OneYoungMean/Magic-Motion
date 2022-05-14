using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion.Extern
{
    public static class Extrension
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void ClearNativeArrayData<T>(NativeArray<T> data) where T : struct
        {
            UnsafeUtility.MemClear(data.GetUnsafePtr(), data.Length * UnsafeUtility.SizeOf<T>());
        }
    }

}
