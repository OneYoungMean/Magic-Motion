using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Collections;
using UnityEngine.Jobs;

namespace MagicMotion
{
    //OYM：第一阶段:安全的代码部分
    //OYM：第二阶段，不安全的代码片段
    //OYM：已经不安全的代码片段除外
    public class MagicMotionKernel 
    {
        private NativeArray<MMJointConstraintNative> constrainNativeArray;
        private NativeArray<MMJointNative> jointNativeArray;
        private NativeArray<MMMuscleNative> muscleNativeArray;
        private TransformAccessArray constraintTransformArray;
        private TransformAccessArray jointTransformArray;

        private MagicMotionJobsTable jobs1;
    }
}

