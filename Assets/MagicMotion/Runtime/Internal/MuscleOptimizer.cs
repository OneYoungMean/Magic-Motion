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
    internal  struct MuscleOptimizer
    {
        #region  Field&Property

        /// <summary>
        /// Joint Data ,Contain localPos and localRot, minRange and maxRange etc.
        /// ParallelLength
        /// ReadOnly
        /// </summary>
        private NativeArray<JointData> jointDataNativeArray;
        /// <summary>
        /// muscle data,contain targetJointIndex and target dof
        /// musclesLength
        /// ReadOnly
        /// </summary>
        private NativeArray<MuscleData> muscleDataNativeArray;
        /// <summary>
        /// ConstraintData ,contatin positionConstraint lookAtConstraint's data. etc.
        /// ParallelLength
        /// ReadOnly
        /// </summary>
        private NativeArray<ConstraintData> constraintNativeArray;

        /// <summary>
        ///  jointRelationData ,contain targetJointIndex, targetMuslceIndex,  change rontation joint's index .use to clac gradient.
        ///  parallelLength
        ///  Readonly
        /// </summary>
        private NativeArray<JointRelationData> jointRelationDataNativeArray;
        /// <summary>
        /// how many constraint influence by this muscle
        /// </summary>
        private NativeArray<int> muscleRelativeCountNativeArray;

        //OYM：base data (readwrite)
        /// <summary>
        /// LBFGSSolver ,contain LBFGS's data
        /// 1
        /// ReadWrite
        /// </summary>
        private NativeArray<LBFGSSolver> LBFGSNatives;
        /// <summary>
        /// Global data ,contain iterationCount, etc 
        /// 1
        /// ReadWrite
        /// </summary>
        private NativeArray<GlobalData> globalDataNativeArray;

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
        private NativeArray<JoinLoss> jointlossNativeArray;
        //OYM：LBFGS data(readwrite)
        /// <summary>
        /// gradient data,use to LBFGS optimie.
        /// muscleLength 
        /// ReadWrite
        /// </summary>
        private NativeArray<double> gradients;
        /// <summary>
        /// LBFGS's gradientStore
        /// muscleLength 
        /// ReadWrite
        /// </summary>
        private NativeArray<double> gradientStore;
        /// <summary>
        /// LBFGS's diagonal data ,is equal for search muscle 
        /// muscleLength
        /// ReadWrite
        /// </summary>
        private NativeArray<double> diagonal;
        /// <summary>
        /// alpha Data
        /// Controll by LBFGSStatic
        /// ReadWrite
        /// </summary>
        private NativeArray<double> alpha;
        /// <summary>
        /// steps Data
        /// Controll by LBFGSStatic
        /// ReadWrite
        /// </summary>
        private NativeArray<double> steps;
        /// <summary>
        /// delta Data
        /// Controll by LBFGSStatic
        /// ReadWrite
        /// </summary>
        private NativeArray<double> delta;
        /// <summary>
        /// rho Data
        /// Controll by LBFGSStatic
        /// ReadWrite
        /// </summary>
        private NativeArray<double> rho;
        //OYM：test data

        private NativeArray<double> lossNativeArray;
        private NativeArray<double> gradientAllNativeArray;
        private NativeArray<float> muscleValueAllNativeArray;


        /// <summary>
        /// Transform musles to joint
        /// </summary>
        private MuscleToDof3Job muscleToDof3Job;
        /// <summary>
        /// clac Dof3 rotaton
        /// </summary>
        private Dof3ToRotationJob dof3ToRotationJob;
        /// <summary>
        /// clac Dof3 epsilion rotation
        /// </summary>
        private ClacDof3EpsilionJob clacDof3EpsilionJob;
        /// <summary>
        /// Build skelnion
        /// </summary>
        private BuildTransformJob buildTransformJob;
        /// <summary>
        /// clac loss value
        /// </summary>
        private CaclulatelossJob caclulatelossJob;
        /// <summary>
        /// CollectGradient and loss value;
        /// </summary>
        private MainControllerJob mainControllerJob;
        /// <summary>
        /// iterationCount;
        /// </summary>
        private int iterationCount;
        /// <summary>
        /// parallelDataCount,is base for joint relation 
        /// </summary>
        private readonly int parallelDataCount;
        /// <summary>
        /// joint count 
        /// </summary>
        private readonly int jointCount;
        /// <summary>
        /// muscle count
        /// </summary>
        private readonly int muscleCount;
        /// <summary>
        /// Optimizer loss
        /// </summary>
        public float Loss =>(float)LBFGSNatives[0].loss;
        /// <summary>
        /// result as muscle
        /// </summary>
        public NativeArray<float> Muscles =>muscleValueNativeArray;

        /// <summary>
        ///  result as dof3
        /// </summary>
        public NativeArray<float3> Dof3s => Dof3NativeArray;
        #endregion

        #region LocalFunc

        public MuscleOptimizer(
             int parallelDataCount, int jointCount, int muscleCount,int iterationCount,NativeArray<int> muscleRelativeCountNativeArray,
             NativeArray<JointData> jointDataNativeArray,
             NativeArray<MuscleData> muscleDataNativeArray,
              NativeArray<ConstraintData> constraintNativeArray,
               NativeArray<JointRelationData> jointRelationDataNativeArray,
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

            this.iterationCount = 0;
            this.jointCount = jointCount;
            this.muscleCount = muscleCount;
            this.parallelDataCount = parallelDataCount;
            this.iterationCount = iterationCount;

            this.jointDataNativeArray = jointDataNativeArray;
            this.muscleDataNativeArray = muscleDataNativeArray;
            this.constraintNativeArray = constraintNativeArray;
            this.jointRelationDataNativeArray = jointRelationDataNativeArray;
            this.muscleRelativeCountNativeArray = muscleRelativeCountNativeArray;
            this.muscleValueNativeArray = muscleValueNativeArray;
            #endregion

            #region CreateNativeArray
            this.LBFGSNatives = CreateNativeData<LBFGSSolver>(1);
            this.globalDataNativeArray = CreateNativeData<GlobalData>(1);
            this.Dof3NativeArray = CreateNativeData<float3>(jointCount);
            this.Dof3QuaternionNativeArray = CreateNativeData<quaternion>(jointCount);
            this.muscleGradientRotationArray = CreateNativeData<quaternion>(muscleCount);
            this.jointlossNativeArray = CreateNativeData<JoinLoss>(parallelDataCount);
            this.jointTransformNativeArray = CreateNativeData<RigidTransform>(jointCount);
            this.gradients = CreateNativeData<double>(muscleCount);
            this.gradientStore = CreateNativeData<double>(muscleCount);
            this.diagonal = CreateNativeData<double>(muscleCount, Allocator.Persistent);
            this.rho = CreateNativeData<double>(L_BFGSStatic.CORRECTION, Allocator.Persistent);    
            this.alpha = CreateNativeData<double>(L_BFGSStatic.CORRECTION, Allocator.Persistent);            
            this.steps = CreateNativeData<double>(muscleCount * L_BFGSStatic.CORRECTION, Allocator.Persistent);        
            this.delta = CreateNativeData<double>(muscleCount * L_BFGSStatic.CORRECTION, Allocator.Persistent);

            this.lossNativeArray = CreateNativeData<double>(iterationCount + 1, Allocator.Persistent);
            this.gradientAllNativeArray = CreateNativeData<double>((iterationCount + 1) * muscleCount, Allocator.Persistent);
            this.muscleValueAllNativeArray = CreateNativeData<float>((iterationCount + 1) * muscleCount, Allocator.Persistent);


            #endregion

            #region CreateJob

            muscleToDof3Job = new MuscleToDof3Job()
            {
                Dof3s = Dof3NativeArray,
                muscleDatas = muscleDataNativeArray,
                muscleValues = muscleValueNativeArray,
            };

            dof3ToRotationJob = new Dof3ToRotationJob
            {
                Dof3s = Dof3NativeArray,
                jointDatas = jointDataNativeArray,
                Dof3Quaternions = Dof3QuaternionNativeArray,
            };

            clacDof3EpsilionJob = new ClacDof3EpsilionJob
            {
                Dof3Quaternions = Dof3QuaternionNativeArray,
                Dof3s = Dof3NativeArray,
                jointDatas = jointDataNativeArray,
                jointTransformNatives = jointTransformNativeArray,
                muscleDatas = muscleDataNativeArray,
                muscleGradientRotations = muscleGradientRotationArray,
                globalDatas = globalDataNativeArray,
            };

            buildTransformJob = new BuildTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray,
                jointDatas = jointDataNativeArray.Slice(0, jointCount),
                Dof3Quaternions = Dof3QuaternionNativeArray,
            };

            caclulatelossJob = new CaclulatelossJob()
            {
                constraintNatives = constraintNativeArray,
                jointRelationDatas = jointRelationDataNativeArray,
                jointTransformNatives = jointTransformNativeArray,
                Dof3s = Dof3NativeArray,
                muscleGradientRotations = muscleGradientRotationArray,
                jointlossNatives = jointlossNativeArray,
            };

            mainControllerJob = new MainControllerJob()
            {
                LBFGSSolvers = LBFGSNatives,
                globalDataNative = globalDataNativeArray,

                gradients = gradients,
                diagonal = diagonal,
                gradientStore = gradientStore,
                rho = rho,
                alpha = alpha,
                steps = steps,
                delta = delta,
                losses = lossNativeArray,
                jointlossNatives = jointlossNativeArray,
                muscleValues = muscleValueNativeArray,
                relationDatas = jointRelationDataNativeArray,
                jointRelativedCounts=muscleRelativeCountNativeArray,
/*                muscleAlls = muscleValueAllNativeArray,
                gradientAlls = gradientAllNativeArray,*/
                parallelLength = parallelDataCount,
                muscleLength = muscleCount,
                jointLength = jointCount,
            };
            #endregion

            #region PostInitialize
            //OYM：Set LBFGSSolver
            var lBFGSSolver = LBFGSSolver.identity;
            lBFGSSolver.numberOfVariables = muscleCount;
            LBFGSNatives[0] = lBFGSSolver;

            #endregion
        }

        public void Run(Vector3 rootPosition, Quaternion rootRotation)
        {
            try
            {
                Reset(rootPosition, rootRotation);
                for (int j = 0; j < iterationCount + 1; j++)
                {
                    muscleToDof3Job.Run(muscleCount);
                    dof3ToRotationJob.Run(jointCount);
                    buildTransformJob.Run(jointCount);
                    clacDof3EpsilionJob.Run(muscleCount);
                    caclulatelossJob.Run(parallelDataCount);
                    mainControllerJob.Run();
                }
            }
            catch (Exception e)
            {

                Debug.LogError(e);
            }

        }
        
        
        private void Reset(Vector3 rootPosition, Quaternion rootRotation)
        {
            //OYM：reset globalData
            var globalData = globalDataNativeArray[0];
            globalData.leastLoopCount = iterationCount;
            globalDataNativeArray[0] = globalData;
            //OYM：reset solver
            var solver = LBFGSNatives[0];
            solver.state = LBFGSState.Initialize;
            LBFGSNatives[0] = solver;
            //OYM：reset root trasnform

            buildTransformJob.rootPosition = rootPosition;
            buildTransformJob.rootRotation = rootRotation; 
        }

        internal Keyframe[] GetmusclesKey(int index)
        {
            Keyframe[] results = new Keyframe[iterationCount + 1];
            for (int i = 0; i < iterationCount + 1; i++)
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
        }
        internal Keyframe[] GetLossKey()
        {
            Keyframe[] results = new Keyframe[iterationCount + 1];
            for (int i = 0; i < iterationCount + 1; i++)
            {
                results[iterationCount - i] = new Keyframe(1 - i / (float)iterationCount,math.log( (float) lossNativeArray[i]));
            }
            return results;
        }
        #endregion
    }

}