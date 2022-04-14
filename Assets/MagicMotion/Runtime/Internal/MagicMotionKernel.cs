using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Collections;
using UnityEngine.Jobs;

namespace MagicMotion
{
    //OYM����һ�׶�:��ȫ�Ĵ��벿��
    //OYM���ڶ��׶Σ�����ȫ�Ĵ���Ƭ��
    //OYM���Ѿ�����ȫ�Ĵ���Ƭ�γ���
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

