using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Collections;
using UnityEngine.Jobs;
using Unity.Mathematics;
using System;
using static MagicMotion.MagicMotionJobsTable;

namespace MagicMotion
{
    //OYM：第一阶段:安全的代码部分
    //OYM：第二阶段，不安全的代码片段
    //OYM：已经不安全的代码片段除外
    public class MagicMotionKernel
    {

        #region  NativeArrayData
        public JobHandle MainHandle;

        //OYM：base data
        private NativeArray<MMJointNative> jointNativeArray;
        private NativeArray<MMMuscleNative> muscleNativeArray;
        private NativeArray<TransformToConstraintNative> transformToConstrainNativeArray;
        private NativeArray<MMJoinFitness> jointFitnessNativeArray;
        private NativeArray<MMJointConstraintNative> constraintNativeArray;

        private NativeArray<float3> Dof3NativeArray;
        private NativeArray<float> muscleValueNativeArray;
        private NativeArray<RigidTransform> jointTransformNativeArray;

        private TransformAccessArray jointTransformArray;
        private TransformAccessArray constraintTransformArray;


        private NativeArray<MMLBFGSNative> LBFGSNatives;
        private NativeArray<float> gradientStore;
        private NativeArray<float> gradients;
        private NativeArray<float> diagonal;

        private NativeArray<float> alpha;
        private NativeArray<float> steps;
        private NativeArray<float> delta;
        private NativeArray<float> rho;
        #endregion

        #region JobsData

        private TransformToConstraintJob getConstraintTransformJob;
        private RigHipPositionJob rigHipPositionJob;
        private MuscleToJointJob muscleToJointJob;
        private BuildTransformJob buildTransformJob;
        private CaclulateFitnessJob caclulateFitnessJob;
        private MainControllerJob mainControllerJob;
        private JointToTransformJob jointToTransformJob;

        #endregion

        #region DefaultData
        private MMJointNative[] joints;
        private MMMuscleNative[] muscles;
        private TransformToConstraintNative[] transformToConstraints;
        private MMJointConstraintNative[] constraints;

        private Transform[] constraintTransforms;
        private Transform[] jointsTransforms;

        private int jointCount;
        private int muscleCount;
        private bool isCreated;
        #endregion

        #region Accessor
        public bool IsCreated { get { return isCreated; } }

        private int parallelDataCount { get { return (muscleCount + 1) * jointCount; } }
        #endregion

        #region PublicFunc
        public void SetData(MMJointConstraintNative[] constraints, Transform[] constraintTransforms)
        {
            this.constraints = constraints;
            this.constraintTransforms = constraintTransforms;
            jointCount = constraints.Length;
        }
        public void SetData(MMJointNative[] joints, TransformToConstraintNative[] transformToConstraints, Transform[] jointsTransforms)
        {
            this.joints = joints;
            this.jointsTransforms = jointsTransforms;
            this.transformToConstraints = transformToConstraints;
        }
        public void SetSata(MMMuscleNative[] muscles)
        {
            this.muscles = muscles;
            muscleCount = muscles.Length;
        }
        public void BuildNativeData()
        {
            if (constraints.Length!= joints.Length)
            {
                throw new System.Exception("constraint length, is not equal joint length");
            }
            if (isCreated)
            {
                Dispose();
                isCreated = false;
            }
            BuildNativeDataInternal();
            BuildJobDataInternal();
            isCreated =true;
        }
        public void Reset()
        {
            if (!MainHandle.IsCompleted)
            {
                MainHandle.Complete();
            }
            InitializeMuscleJob initializeMuscleJob = new InitializeMuscleJob()
            {
                musclesNatives = muscleNativeArray,
                musclesValues = muscleValueNativeArray
            };
            MainHandle = initializeMuscleJob.Schedule(muscleCount, 32, MainHandle);
            MainHandle= muscleToJointJob.Schedule (muscleCount,32, MainHandle);
            MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);

        }
        public void Schedule(float deltatime,int iterationCount)
        {
            if (!MainHandle.IsCompleted)
            {
                return;
            }

            MainHandle = default(JobHandle);
            for (int i = 0; i < LBFGSNatives.Length; i++)
            {
                LBFGSNatives[i].Reset();
            }
            MainHandle = getConstraintTransformJob.ScheduleReadOnly(constraintTransformArray, 32,MainHandle);
            MainHandle = rigHipPositionJob.Schedule(muscleCount + 1, 4, MainHandle);
            for (int i = 0; i < iterationCount; i++)
            {
                MainHandle=muscleToJointJob.Schedule(muscleCount+1, 4, MainHandle);
                MainHandle = buildTransformJob.Schedule(muscleCount + 1, 4, MainHandle);
                MainHandle = caclulateFitnessJob.Schedule(parallelDataCount, 32, MainHandle);
                MainHandle = mainControllerJob.Schedule(1, 1, MainHandle);
            }
            MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);
        }
        public void Dispose()
        {
            if (!MainHandle.IsCompleted)
            {
                MainHandle.Complete();
            }

            jointNativeArray.Dispose();
            muscleNativeArray.Dispose();
            jointFitnessNativeArray.Dispose();
            constraintNativeArray.Dispose();

            Dof3NativeArray.Dispose();
            muscleValueNativeArray.Dispose();
            jointTransformNativeArray.Dispose();

            jointTransformArray.Dispose();
            constraintTransformArray.Dispose();

            LBFGSNatives.Dispose();
            gradients.Dispose();

            L_BFGSStatic.Disposed(diagonal, gradientStore, rho, alpha, steps, delta);
        }
        #endregion

        #region PrivateFunc
        private void BuildNativeDataInternal()
        {
            if (isCreated)
            {
                throw new System.Exception("Disposed NativeArray before you create it");
            }
            constraintNativeArray = new NativeArray<MMJointConstraintNative>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < muscleCount + 1; i++)
            {
                NativeArray<MMJointConstraintNative>.Copy(constraints, 0, constraintNativeArray, i * jointCount, jointCount);
            }

            jointNativeArray = new NativeArray<MMJointNative>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < muscleCount + 1; i++)
            {
                NativeArray<MMJointNative>.Copy(joints, 0, jointNativeArray, i * jointCount, jointCount);
            }

            jointTransformNativeArray = new NativeArray<RigidTransform>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                jointTransformNativeArray[i] = RigidTransform.identity;
            }

            muscleNativeArray = new NativeArray<MMMuscleNative>(muscles, Allocator.Persistent);

            transformToConstrainNativeArray = new NativeArray<TransformToConstraintNative>(transformToConstraints, Allocator.Persistent);

            constraintTransformArray = new TransformAccessArray(constraintTransforms);

            jointTransformArray = new TransformAccessArray(jointsTransforms);

            jointFitnessNativeArray = new NativeArray<MMJoinFitness>(parallelDataCount, Allocator.Persistent);

            Dof3NativeArray = new NativeArray<float3>(parallelDataCount, Allocator.Persistent);

            gradients = new NativeArray<float>(muscleCount, Allocator.Persistent);

            LBFGSNatives = new NativeArray<MMLBFGSNative>(1, Allocator.Persistent);
            LBFGSNatives[0] = MMLBFGSNative.identity;

            L_BFGSStatic.CreateWorkVector(muscleCount, out diagonal, out gradientStore, out rho, out alpha, out steps, out delta);
        }
        private void BuildJobDataInternal()
        {
            getConstraintTransformJob = new TransformToConstraintJob()
            {
                constraintNatives = constraintNativeArray,
                muscleLength = muscleCount,
                jointLength = jointCount
            };
            rigHipPositionJob = new RigHipPositionJob()
            {
                jointNatives = jointNativeArray,
                constraintNatives = constraintNativeArray,
                jointTransformNatives = jointTransformNativeArray,
                jointLength = jointCount,
                loopCount = 10,
            };
            muscleToJointJob = new MuscleToJointJob()
            {
                Dof3s = Dof3NativeArray,
                musclesNatives = muscleNativeArray,
                musclesValues = muscleValueNativeArray,
                jointCount = jointCount,
                muscleCount = muscleCount,
            };
             buildTransformJob = new BuildTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray,
                jointNatives = jointNativeArray,
                Dof3s = Dof3NativeArray,
                jointLength = jointCount,
            };
             caclulateFitnessJob = new CaclulateFitnessJob()
            {
                constraintNatives = constraintNativeArray,
                jointFitnessNatives = jointFitnessNativeArray,
                Dof3s = Dof3NativeArray,
            };

             mainControllerJob = new MainControllerJob()
            {
                jointFitnessNatives = jointFitnessNativeArray,
                Dof3s = Dof3NativeArray,
                muscleValueNatives = muscleValueNativeArray,
                gradients = gradients,
                LBFGSNatives = LBFGSNatives,
                diagonal = diagonal,
                gradientStore = gradientStore,
                rho = rho,
                alpha = alpha,
                steps = steps,
                delta = delta,
                muscleLength = muscleCount,
                jointLength = jointCount,
                fitness = 0
            };

             jointToTransformJob = new JointToTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray
            };
        }

        #endregion
    }
}

