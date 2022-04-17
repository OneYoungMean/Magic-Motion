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
using Unity.Collections.LowLevel.Unsafe;

namespace MagicMotion
{
    //OYM：第一阶段:安全的代码部分
    //OYM：第二阶段，不安全的代码片段
    //OYM：已经不安全的代码片段除外
    public unsafe class MagicMotionKernel
    {
        public const int iteration =128;
        #region  NativeArrayData
        public JobHandle MainHandle;

        //OYM：base data
        private NativeArray<MMJointNative> jointNativeArray;
        private NativeArray<MMMuscleNative> muscleNativeArray;
        private NativeArray<TransformToConstraintNative> transformToConstrainNativeArray;
        private NativeArray<MMJoinloss> jointlossNativeArray;

        private NativeArray<MMConstraintNative> constraintNativeArray;


        private NativeArray<float3> Dof3NativeArray;
        private NativeArray<float> muscleValueNativeArray;
        private NativeArray<RigidTransform> jointTransformNativeArray;

        private TransformAccessArray jointTransformArray;
        private TransformAccessArray constraintTransformArray;


        private NativeArray<MMLBFGSSolver> LBFGSNatives;
        private NativeArray<MMGlobalData> globalDataNativeArray;

        private NativeArray<float> gradientStore;
        private NativeArray<float> gradients;
        private NativeArray<float> diagonal;

        private NativeArray<float> alpha;
        private NativeArray<float> steps;
        private NativeArray<float> delta;
        private NativeArray<float> rho;

        private NativeArray<float> losses;
        #endregion

        #region JobsData

        private TransformToConstraintJob getConstraintTransformJob;
        private RigHipPositionJob rigHipPositionJob;
        private MuscleToJointJob muscleToJointJob;
        private BuildTransformJob buildTransformJob;
        private CaclulatelossJob caclulatelossJob;
        private MainControllerJob mainControllerJob;
        private JointToTransformJob jointToTransformJob;

        #endregion

        #region DefaultData
        private MMJointNative[] joints;
        private MMMuscleNative[] muscles;
        private TransformToConstraintNative[] transformToConstraints;
        private MMConstraintNative[] constraints;

        private Transform[] constraintTransforms;
        private Transform[] jointsTransforms;

        private int jointCount;
        private int muscleCount;
        private bool isCreated;
        private BroydenFletcherGoldfarbShanno LBFGSSolver;
        #endregion

        #region Accessor
        public bool IsCreated { get { return isCreated; } }

        private int parallelDataCount { get { return (muscleCount + 1) * jointCount; } }
        #endregion

        #region PublicFunc
        public void SetConstraintData(MMConstraintNative[] constraints, TransformToConstraintNative[] transformToConstraints, Transform[] constraintTransforms)
        {
            this.constraints = constraints;
            this.constraintTransforms = constraintTransforms;
            this.transformToConstraints = transformToConstraints;
        }
        public void SetJointData(MMJointNative[] joints, Transform[] jointsTransforms)
        {
            this.joints = joints;
            this.jointsTransforms = jointsTransforms;
            jointCount = joints.Length;
        }
        public void SetMuscleSata(MMMuscleNative[] muscles)
        {
            this.muscles = muscles;
            muscleCount = muscles.Length;
        }
        public void Initialize()
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
            BuildMainThreadTest();
            Reset();
            isCreated =true;
        }

        private void BuildMainThreadTest()
        {
            LBFGSSolver = new BroydenFletcherGoldfarbShanno(muscleCount, x => GetLoss(x), y => GetGradient(y));
        }

        private float GetLoss(NativeArray<float> solution)
        {
            UnsafeUtility.MemCpy(muscleValueNativeArray.GetUnsafePtr(), solution.GetUnsafePtr(), solution.Length * UnsafeUtility.SizeOf<float>());
            muscleToJointJob.Run(muscleCount + 1);
            buildTransformJob.Run(muscleCount + 1);
            caclulatelossJob.Run(parallelDataCount);
            mainControllerJob.Run(1);
            return losses[0];
        }
        private NativeArray<float> GetGradient(NativeArray<float> solution) 
        {
            return gradients;
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
/*            InitializeJointJob initializeJointJob = new InitializeJointJob()
            {
                jointTransformNatives = jointTransformNativeArray,
                muscleLength =muscleCount,
                jointLength=jointCount
            };*/
            initializeMuscleJob.Run(muscleCount);
            muscleToJointJob.Run(muscleCount);
            buildTransformJob.Run(muscleCount);
/*            MainHandle = initializeMuscleJob.Schedule(muscleCount, 32, MainHandle);
            MainHandle= muscleToJointJob.Schedule (muscleCount,32, MainHandle);
            MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);*/

        }
        public void Update(float deltatime)
        {
            if (!MainHandle.IsCompleted)
            {
                return;
            }

            MainHandle = default(JobHandle);
            for (int i = 0; i < LBFGSNatives.Length; i++)
            {
                var solver = LBFGSNatives[i];
                solver.Reset();
                solver.numberOfVariables = muscleCount;
                LBFGSNatives[i] = solver;

                var globalData = globalDataNativeArray[i];
                globalData.leastLoopCount = iteration;
                globalDataNativeArray[i] = globalData;

            }
            if (false)
            {
                getConstraintTransformJob.RunReadOnly(constraintTransformArray);
                for (int i = 0; i < iteration; i++)
                {
                    muscleToJointJob.Run(muscleCount + 1);
                    buildTransformJob.Run(muscleCount + 1);
                    caclulatelossJob.Run(parallelDataCount);
                    mainControllerJob.Run(1);
                }
                jointToTransformJob.Schedule(jointTransformArray).Complete();
            }
            else
            {
                Debug.Log(losses[iteration-1]+" ~ "+ losses[0]);

            MainHandle = getConstraintTransformJob.ScheduleReadOnly(constraintTransformArray, 32,MainHandle);
            for (int i = 0; i < iteration; i++)
            {
                MainHandle=muscleToJointJob.Schedule(muscleCount+1, 8, MainHandle);
                MainHandle = buildTransformJob.Schedule(muscleCount + 1, 8, MainHandle);
                MainHandle = caclulatelossJob.Schedule(parallelDataCount, 32, MainHandle);
                MainHandle = mainControllerJob.Schedule(1, 1, MainHandle);
            }
            MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);
            }


        }
        public void Dispose()
        {
            if (!MainHandle.IsCompleted)
            {
                MainHandle.Complete();
            }

            jointNativeArray.Dispose();
            muscleNativeArray.Dispose();
            jointlossNativeArray.Dispose();
            constraintNativeArray.Dispose();

            Dof3NativeArray.Dispose();
            muscleValueNativeArray.Dispose();
            jointTransformNativeArray.Dispose();

            jointTransformArray.Dispose();
            constraintTransformArray.Dispose();

            LBFGSNatives.Dispose();
            globalDataNativeArray.Dispose();
            gradients.Dispose();
            losses.Dispose();

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
            constraintNativeArray = new NativeArray<MMConstraintNative>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < muscleCount + 1; i++)
            {
                NativeArray<MMConstraintNative>.Copy(constraints, 0, constraintNativeArray, i * jointCount, jointCount);
            }

            jointNativeArray = new NativeArray<MMJointNative>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < muscleCount + 1; i++)
            {
                NativeArray<MMJointNative>.Copy(joints, 0, jointNativeArray, i * jointCount, jointCount);
            }

            jointTransformNativeArray = new NativeArray<RigidTransform>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                RigidTransform rigid;
                if (i % jointCount == 0)
                {
                    rigid.pos = jointsTransforms[0].position;
                    rigid.rot = jointsTransforms[0].rotation;
                }
                else
                {
                    rigid = RigidTransform.identity;
                }
                jointTransformNativeArray[i] = rigid;
            }

            muscleNativeArray = new NativeArray<MMMuscleNative>(muscles, Allocator.Persistent);

            muscleValueNativeArray=new NativeArray<float>(muscles.Length, Allocator.Persistent);

            transformToConstrainNativeArray = new NativeArray<TransformToConstraintNative>(transformToConstraints, Allocator.Persistent);

            constraintTransformArray = new TransformAccessArray(constraintTransforms);

            jointTransformArray = new TransformAccessArray(jointsTransforms);

            jointlossNativeArray = new NativeArray<MMJoinloss>(parallelDataCount, Allocator.Persistent);

            Dof3NativeArray = new NativeArray<float3>(parallelDataCount, Allocator.Persistent);

            gradients = new NativeArray<float>(muscleCount, Allocator.Persistent);

            LBFGSNatives = new NativeArray<MMLBFGSSolver>(1, Allocator.Persistent);

            globalDataNativeArray = new NativeArray<MMGlobalData>(1, Allocator.Persistent);

            LBFGSNatives[0] = MMLBFGSSolver.identity;

            L_BFGSStatic.CreateWorkVector(muscleCount, out diagonal, out gradientStore, out rho, out alpha, out steps, out delta);

            losses=new NativeArray<float>(iteration, Allocator.Persistent);
        }
        private void BuildJobDataInternal()
        {
            getConstraintTransformJob = new TransformToConstraintJob()
            {
                transformToConstrainNatives= transformToConstrainNativeArray,
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
                globalDataNative= globalDataNativeArray,
                Dof3s = Dof3NativeArray,
                musclesNatives = muscleNativeArray,
                musclesValues = muscleValueNativeArray,
                jointCount = jointCount,
                muscleCount = muscleCount,
            };
             buildTransformJob = new BuildTransformJob()
            {
                 globalDataNative = globalDataNativeArray,
                 jointTransformNatives = jointTransformNativeArray,
                jointNatives = jointNativeArray,
                Dof3s = Dof3NativeArray,
                jointLength = jointCount,
            };
             caclulatelossJob = new CaclulatelossJob()
            {
                 globalDataNative = globalDataNativeArray,
                 constraintNatives = constraintNativeArray,
                jointlossNatives = jointlossNativeArray,
                jointTransformNatives= jointTransformNativeArray,
                Dof3s = Dof3NativeArray,
            };

             mainControllerJob = new MainControllerJob()
            {
                 globalDataNative = globalDataNativeArray,
                 jointlossNatives = jointlossNativeArray,
                muscleValueNatives = muscleValueNativeArray,
                gradients = gradients,
                LBFGSSolvers = LBFGSNatives,
                diagonal = diagonal,
                gradientStore = gradientStore,
                 losses= losses,
                rho = rho,
                alpha = alpha,
                steps = steps,
                delta = delta,
                constraintLength =constraintTransforms.Length,
                muscleLength = muscleCount,
                jointLength = jointCount,
                loss = 0
            };

            jointToTransformJob = new JointToTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray.Slice(0, jointCount),
                constraintNatives=constraintNativeArray,
            };
        }

        #endregion
    }
}

