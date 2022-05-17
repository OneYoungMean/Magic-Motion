using MagicMotion.Extern;
using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using static MagicMotion.MagicMotionJobsTable;

namespace MagicMotion.Internal
{
    /// <summary>
    ///  Single Optimize
    /// </summary>
    internal unsafe struct LinerSearchGroup : IDisposable
    {
        #region  Field&Property

        /// <summary>
        /// Joint Data ,Contain localPos and localRot, minRange and maxRange etc.
        /// ParallelLength
        /// ReadOnly
        /// </summary>
        private NativeArray<JointData> jointDataNativeArray;
        /// <summary>
        /// joint relation muscle data,empty is -1
        /// musclesLength
        /// ReadOnly
        /// </summary>
        private NativeArray<int3> jointMuscleIndexNativeArray;

        /// <summary>
        ///  How many constraint affected by this joint
        /// </summary>
        private NativeArray<int> jointConstraintRelativeCountNativeArray;

        /// <summary>
        /// ConstraintData ,contatin positionConstraint lookAtConstraint's data. etc.
        /// ParallelLength
        /// ReadOnly
        /// </summary>
        private NativeArray<ConstraintData> constraintNativeArray;

        /// <summary>
        ///  parallelRelationData ,contain targetJointIndex, targetMuslceIndex,  change rontation joint's index .use to clac gradient.
        ///  parallelLength
        ///  Readonly
        /// </summary>
        private NativeArray<ParallelRelationData> parallelRelationDataNativeArray;


        //OYM：base data (readwrite)
        /// <summary>
        /// LBFGSSolver ,contain LBFGS's data
        /// 1
        /// ReadWrite
        /// </summary>
        private LBFGSSolver* LBFGSNative;
        /// <summary>
        /// Global data ,contain iterationCount, etc 
        /// 1
        /// ReadWrite
        /// </summary>
        private GlobalData* globalData;
        /// <summary>
        /// current loss value
        /// 1
        /// Read write
        /// </summary>
        private GroupLossData* currentGroupLoss;
        /// <summary>
        /// groupSettingData,include count and root transform
        /// </summary>
        private GroupSettingData* groupSettingData;

        /// <summary>
        /// muscle's Value
        /// muscleLength
        /// ReadWrite
        /// </summary>
        private NativeArray<float> muscleValueNativeArray;

        /// <summary>
        /// Dof3 value 
        /// jointLength
        /// ReadWrite
        /// </summary>
        private NativeArray<float3> Dof3NativeArray;
        /// <summary>
        /// Dof3 value to Quaternion Value 
        /// jointLength
        /// ReadWrite
        /// </summary>
        private NativeArray<quaternion> Dof3QuaternionNativeArray;
        /// <summary>
        /// Gradient's epsilion rotation
        /// parallelLength
        /// ReadWrite
        /// </summary>
        private NativeArray<quaternion> muscleGradientRotationArray;
        /// <summary>
        /// Joint  world transform data
        ///  jointLength
        ///  ReadWrite
        /// </summary>
        private NativeArray<RigidTransform> jointTransformNativeArray;
        /// <summary>
        /// joint lossData,contain stardard and gradient loss
        /// parallelLength
        /// ReadWrite
        /// </summary>
        private NativeArray<JointLoss> jointlossNativeArray;
        //OYM：LBFGS data(readwrite)
        /// <summary>
        /// gradient data,use to LBFGS optimie.
        /// muscleLength 
        /// ReadWrite
        /// </summary>
        private NativeArray<double> gradients;
        /// <summary>
        /// LBFGS's array data stroe
        /// ReadWrite
        /// </summary>
        private NativeArray<double> dataStore;

        //OYM：test data

        private NativeArray<double> lossRecorderNativeArray;
        /*        private NativeArray<double> gradientAllNativeArray;
                private NativeArray<float> muscleValueAllNativeArray;*/



        /// <summary>
        /// clac Dof3 rotaton
        /// </summary>
        private LinerSearchJob linerSearchJob;
        private NativeArray<double> muscleEpsilionNativeArray;

        /// <summary>
        /// Optimizer loss
        /// </summary>
        public double Loss => currentGroupLoss->loss;
        /// <summary>
        /// result as muscle
        /// </summary>
        public NativeArray<float> Muscles => muscleValueNativeArray;

        /// <summary>
        ///  result as dof3
        /// </summary>
        public NativeArray<float3> Dof3s => Dof3NativeArray;
        #endregion

        #region LocalFunc

        public LinerSearchGroup(
             NativeArray<GroupSettingData> groupSettingNativeArray,
             NativeArray<JointData> jointDataNativeArray,
             NativeArray<GroupLossData> lineSearcherLossNativeArray,
              /*             NativeArray<MusclesData> muscleDataNativeArray,*/
              NativeArray<ConstraintData> constraintNativeArray,
               NativeArray<ParallelRelationData> parallelRelationDataNativeArray,
               NativeArray<int> jointConstraintRelativeCountNativeArray,
NativeArray<int3> jointMusclesIndexNativeArray,
NativeArray<float> muscleValueNativeArray,
             List<IDisposable> disposeList
            )
        {
            #region DefineCreateFunc
            NativeArray<T> CreateNativeData<T>(int length, Allocator allocator = Allocator.Persistent) where T : struct
            {
                var nativeArr = new NativeArray<T>(length, allocator);
                disposeList.Add(nativeArr);
                return nativeArr;
            }
            #endregion

            #region SetValue

            this.jointMuscleIndexNativeArray = jointMusclesIndexNativeArray;
            this.jointDataNativeArray = jointDataNativeArray;
            this.constraintNativeArray = constraintNativeArray;
            this.parallelRelationDataNativeArray = parallelRelationDataNativeArray;
            this.muscleValueNativeArray = muscleValueNativeArray;
            this.jointConstraintRelativeCountNativeArray = jointConstraintRelativeCountNativeArray;
            #endregion

            #region CreateNativeArray
            this.LBFGSNative = (LBFGSSolver*)CreateNativeData<LBFGSSolver>(1).GetUnsafePtr();
            this.globalData = (GlobalData*)CreateNativeData<GlobalData>(1).GetUnsafePtr();
            this.currentGroupLoss = (GroupLossData*)lineSearcherLossNativeArray.GetUnsafePtr();
            this.groupSettingData = (GroupSettingData*)groupSettingNativeArray.GetUnsafePtr();

            this.Dof3NativeArray = CreateNativeData<float3>(groupSettingData->jointLength);
            this.Dof3QuaternionNativeArray = CreateNativeData<quaternion>(groupSettingData->jointLength);
            this.muscleGradientRotationArray = CreateNativeData<quaternion>(groupSettingData->muscleLength);
            this.jointlossNativeArray = CreateNativeData<JointLoss>(groupSettingData->jointLength);
            this.jointTransformNativeArray = CreateNativeData<RigidTransform>(groupSettingData->jointLength);
            this.gradients = CreateNativeData<double>(groupSettingData->muscleLength);
            this.dataStore = CreateNativeData<double>(L_BFGSStatic.GetDataStoreLength(groupSettingData->muscleLength));
            this.muscleEpsilionNativeArray = CreateNativeData<double>(groupSettingData->muscleLength, Allocator.Persistent);
            /*            this.gradientAllNativeArray = CreateNativeData<double>((iterationCount + 1) * muscleCount, Allocator.Persistent);
                        this.muscleValueAllNativeArray = CreateNativeData<float>((iterationCount + 1) * muscleCount, Allocator.Persistent);*/

            this.lossRecorderNativeArray = new NativeArray<double>(groupSettingData->LoopSum, Allocator.Persistent);//OYM：keep empty on initialize 
            #endregion

            #region CreateJob

            linerSearchJob = new LinerSearchJob
            {
                constraintDatas = (ConstraintData*)constraintNativeArray.GetUnsafeReadOnlyPtr(),
                jointMuscleIndexs = (int3*)jointMusclesIndexNativeArray.GetUnsafeReadOnlyPtr(),
                jointDatas = (JointData*)jointDataNativeArray.GetUnsafeReadOnlyPtr(),
                parallelRelationDatas = (ParallelRelationData*)parallelRelationDataNativeArray.GetUnsafeReadOnlyPtr(),
                jointConstraintRelativeCounts = (int*)jointConstraintRelativeCountNativeArray.GetUnsafeReadOnlyPtr(),

                muscleValues = (float*)muscleValueNativeArray.GetUnsafePtr(),
                Dof3s = (float3*)Dof3NativeArray.GetUnsafePtr(),
                muscleCurrentRotations = (quaternion*)Dof3QuaternionNativeArray.GetUnsafePtr(),
                muscleGradientRotations = (quaternion*)muscleGradientRotationArray.GetUnsafePtr(),
                jointTransformNatives = (RigidTransform*)jointTransformNativeArray.GetUnsafePtr(),

                LBFGSSolver = LBFGSNative,
                settingData = groupSettingData,

                gradients = (double*)gradients.GetUnsafePtr(),
                dataStore = (double*)dataStore.GetUnsafePtr(),
                lossesRecorder = (double*)lossRecorderNativeArray.GetUnsafePtr(),
                muscleEpsilions=(double*)muscleEpsilionNativeArray.GetUnsafePtr(),
                currentGroupLoss = currentGroupLoss,
                jointlosses = (JointLoss*)jointlossNativeArray.GetUnsafePtr(),

            };
            #endregion

            #region PostInitialize
            //OYM：Set LBFGSSolver
            var lBFGSSolver = LBFGSSolver.identity;
            lBFGSSolver.numberOfVariables = groupSettingData->muscleLength;
            LBFGSNative[0] = lBFGSSolver;

            #endregion
        }
        public JobHandle GetHandle(JobHandle handle)
        {
            Reset();
            return linerSearchJob.Schedule(groupSettingData->insideLoopCount, handle);
        }
        public void Run()
        {
            Reset();
            try
            {
                linerSearchJob.Run(groupSettingData->insideLoopCount);
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }

        }

        private void Reset()
        {
            //OYM：reset solver
            LBFGSNative->Reset();
            //OYM：reset root trasnform
        }

        /*        internal Keyframe[] GetmusclesKey(int index)
                {
                    Keyframe[] results = new Keyframe[groupSettingData->insideLoopCount* groupSettingData->outsideLoopCount];
                    for (int i = 0; i < groupSettingData->insideLoopCount * groupSettingData->outsideLoopCount; i++)
                    {
                        results[iterationCount - i] = new Keyframe(1 - i / (float)iterationCount, muscleValueAllNativeArray[index + i * muscleCount]);
                    }
                    return results;
                }
                internal Keyframe[] GetGradientsKey(int index)
                {
                    Keyframe[] results = new Keyframe[iterationCount + 1];
                    for (int i = 0; i < iterationCount + 1; i++)
                    {
                        results[iterationCount - i] = new Keyframe(1 - i / (float)iterationCount, (float)gradientAllNativeArray[index + i * muscleCount]);
                    }
                    return results;
                }*/
        internal Keyframe[] GetLossKey()
        {
            int loopSum = groupSettingData->insideLoopCount * groupSettingData->outsideLoopCount;
            Keyframe[] results = new Keyframe[loopSum];
            for (int i = 0; i < loopSum; i++)
            {
                results[i] = new Keyframe(i / (float)groupSettingData->insideLoopCount * groupSettingData->outsideLoopCount, math.log((float)lossRecorderNativeArray[i]));
            }
            return results;
        }

        public void Dispose()
        {
            lossRecorderNativeArray.Dispose();
        }
        #endregion
    }

}

/*        /// <summary>
        /// Transform musles to joint
        /// </summary>
        private MuscleToDof3Job muscleToDof3Job;*/
/*        /// <summary>
        /// clac Dof3 epsilion rotation
        /// </summary>
        private ClacDof3EpsilionJob clacDof3EpsilionJob;
        /// <summary>
        /// Build skelnion
        /// </summary>
        private BuildTransformJob buildTransformJob;*/

/*            muscleToDof3Job = new MuscleToDof3Job()
{
    Dof3s = (float3*)Dof3NativeArray.GetUnsafePtr(),
    muscleDatas = (MusclesData*)muscleDataNativeArray.GetUnsafeReadOnlyPtr(),
    muscleValues =(float*) muscleValueNativeArray.GetUnsafeReadOnlyPtr(),
};
clacDof3EpsilionJob = new ClacDof3EpsilionJob
{
    muscleCurrentRotation = (quaternion*)Dof3QuaternionNativeArray.GetUnsafeReadOnlyPtr(),
    jointTransformNatives = (RigidTransform*)jointTransformNativeArray.GetUnsafeReadOnlyPtr(),
    muscleGradientRotations =(quaternion*) muscleGradientRotationArray.GetUnsafePtr(),
    jointMuscleIndexs = (int3*)jointMusclesIndexNativeArray.GetUnsafeReadOnlyPtr(),
};

buildTransformJob = new BuildTransformJob()
{
    jointTransformNatives = (RigidTransform*)jointTransformNativeArray.GetUnsafePtr(),
    jointDatas = (JointData*)jointDataNativeArray.GetUnsafeReadOnlyPtr(),
    muscleCurrentRotation = (quaternion*)Dof3QuaternionNativeArray.GetUnsafePtr(),
};*/

/* muscleToDof3Job.Run(muscleCount);*/
/*                    buildTransformJob.Run(jointCount);
 clacDof3EpsilionJob.Run(jointCount);*/
/*caclulatelossJob = new CaclulatelossJob()
{
    constraintDatas = (ConstraintData*)constraintNativeArray.GetUnsafeReadOnlyPtr(),
    parallelRelationDatas = (parallelRelationData*)parallelRelationDataNativeArray.GetUnsafeReadOnlyPtr(),

    jointTransformNatives = (RigidTransform*)jointTransformNativeArray.GetUnsafeReadOnlyPtr(),
    Dof3s = (float3*)Dof3NativeArray.GetUnsafeReadOnlyPtr(),
    muscleGradientRotations = (quaternion*)muscleGradientRotationArray.GetUnsafeReadOnlyPtr(),
    jointlossNatives = (double*)jointlossNativeArray.GetUnsafePtr(),
};

mainControllerJob = new MainControllerJob()
{
    LBFGSSolver = LBFGSNative,
    globalData = globalData,

    gradients = (double*)gradients.GetUnsafePtr(),
    dataStore = (double*)dataStore.GetUnsafePtr(),
    losses = (double*)lossNativeArray.GetUnsafePtr(),
    jointlossNatives = (double*)jointlossNativeArray.GetUnsafePtr(),
    muscleValues = (float*)muscleValueNativeArray.GetUnsafePtr(),
    relationDatas = (parallelRelationData*)parallelRelationDataNativeArray.GetUnsafePtr(),
    constraintLength = constraintCount,
    parallelLength = parallelDataCount,
    muscleLength = muscleCount,
    jointLength = jointCount,

    muscleAlls = muscleValueAllNativeArray,
    gradientAlls = gradientAllNativeArray,
};*/

/*        /// <summary>
        /// clac loss value
        /// </summary>
        private CaclulatelossJob caclulatelossJob;
        /// <summary>
        /// CollectGradient and loss value;
        /// </summary>
        private MainControllerJob mainControllerJob;*/