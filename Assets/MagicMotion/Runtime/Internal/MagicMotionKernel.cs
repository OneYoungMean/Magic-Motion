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
        private NativeArray<JointData> jointDataNativeArray;
        private NativeArray<MMMuscleData> muscleDataArray;
        private NativeArray<quaternion> muscleGradientRotationArray;
        private NativeArray<int> muscleRelativeCountArray;
        private NativeArray<TransformToConstraintNative> transformToConstrainNativeArray;
        private NativeArray<MMJoinloss> jointlossNativeArray;
        private NativeArray<JointRelationData> jointRelationDataNativeArray;
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
        /*        private MuscleToJointJob muscleToJointJob;*/
        private BuildTransformJob buildTransformJob;
        private ScheduleConstraintDataJob scheduleConstraintDataJob;
        private CaclulatelossJob caclulatelossJob;
        private MainControllerJob mainControllerJob;
        private JointToTransformJob jointToTransformJob;

        #endregion

        #region DefaultData
        private JointData[] joints;
        private MMMuscleData[] muscles;
        private TransformToConstraintNative[] transformToConstraints;
        private MMConstraintNative[] constraints;

        private Transform[] constraintTransforms;
        private Transform[] jointsTransforms;

        private int jointCount;
        private bool[][] jointRelationMap;
        private int muscleCount;
        private bool isCreated;
        private BroydenFletcherGoldfarbShanno LBFGSSolver;
        private JointRelationData[] jointMapDatas;
        #endregion

        #region Accessor
        public bool IsCreated { get { return isCreated; } }

        private int parallelDataCount;
        private int[] muscleRelativedCounts;
        #endregion

        #region PublicFunc
        public void SetConstraintData(MMConstraintNative[] constraints, TransformToConstraintNative[] transformToConstraints, Transform[] constraintTransforms)
        {
            this.constraints = constraints;
            this.constraintTransforms = constraintTransforms;
            this.transformToConstraints = transformToConstraints;
        }
        public void SetJointData(JointData[] joints, Transform[] jointsTransforms)
        {
            this.joints = joints;
            this.jointsTransforms = jointsTransforms;
            jointCount = joints.Length;



        }
        public void SetMuscleSata(MMMuscleData[] muscles)
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
            BuildRelationData();
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
            mainControllerJob.Run(1);
            caclulatelossJob.Run(parallelDataCount);
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
                muscleDatas = muscleDataArray,
                musclesValues = muscleValueNativeArray
            };
/*            InitializeJointJob initializeJointJob = new InitializeJointJob()
            {
                jointTransformNatives = jointTransformNativeArray,
                muscleLength =muscleCount,
                jointLength=jointCount
            };*/
            initializeMuscleJob.Run(muscleCount);
/*            muscleToJointJob.Run(muscleCount);
            buildTransformJob.Run(muscleCount);*/

            /*            MainHandle = initializeMuscleJob.Schedule(muscleCount, 32, MainHandle);
                        MainHandle= muscleToJointJob.Schedule (muscleCount,32, MainHandle);
                        MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);*/

        }
        public void Update(float deltatime)
        {
            if (!isCreated||!MainHandle.IsCompleted)
            {
                return;
            }

            MainHandle = default(JobHandle);
            for (int i = 0; i < LBFGSNatives.Length; i++)
            {
                var globalData = globalDataNativeArray[i];
                globalData.leastLoopCount = iteration-1;
                globalData.isInitialize = false;
                globalDataNativeArray[i] = globalData;
            }
            Debug.Log(losses[iteration - 2] + " ~ " + losses[0]);
            if (true)
            {
                getConstraintTransformJob.RunReadOnly(constraintTransformArray);
                scheduleConstraintDataJob.Run(parallelDataCount);
                /*                for (int i = 0; i < iteration; i++)
                                {
                                    mainControllerJob.Run(1);
                                    caclulatelossJob.Run(parallelDataCount);
                                }*/
                mainControllerJob.Run(1);
                caclulatelossJob.Run(parallelDataCount);
                LBFGSSolver.Minimize(muscleValueNativeArray);
                jointToTransformJob.Schedule(jointTransformArray).Complete();
            }
            else
            {

            MainHandle = getConstraintTransformJob.ScheduleReadOnly(constraintTransformArray, 32,MainHandle);
            for (int i = 0; i < iteration; i++)
            {
/*                MainHandle=muscleToJointJob.Schedule(muscleCount+1, 8, MainHandle);
                MainHandle = buildTransformJob.Schedule(muscleCount + 1,8, MainHandle);*/
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

            jointDataNativeArray.Dispose();
            muscleDataArray.Dispose();
            jointlossNativeArray.Dispose();
            constraintNativeArray.Dispose();

            Dof3NativeArray.Dispose();
            muscleValueNativeArray.Dispose();
            jointTransformNativeArray.Dispose();

            jointTransformArray.Dispose();
            constraintTransformArray.Dispose();
            transformToConstrainNativeArray.Dispose();

            LBFGSNatives.Dispose();
            globalDataNativeArray.Dispose();
            gradients.Dispose();
            losses.Dispose();

            muscleGradientRotationArray.Dispose();
            jointRelationDataNativeArray.Dispose();
            muscleRelativeCountArray.Dispose();

            if (LBFGSSolver!=null)
            {
                LBFGSSolver.Dispose();
            }

            L_BFGSStatic.Disposed(diagonal, gradientStore, rho, alpha, steps, delta);
        }
        #endregion

        #region PrivateFunc

        private void BuildRelationData()
        {
            List<JointRelationData> jointMapDataTemp = new List<JointRelationData>();

            jointRelationMap = new bool[jointCount][];
            for (int i = 0; i < jointCount; i++)
            {
                jointRelationMap[i] = new bool[jointCount];
            }

            for (int i = 0; i < jointCount; i++)
            {
                int parentIndex =i;
                while (parentIndex != -1)
                {
                    jointRelationMap[parentIndex][i] = true;
                    parentIndex = joints[parentIndex].parentIndex;
                }
            }
            for (int i = 0; i < jointCount; i++)
            {
                jointMapDataTemp.Add(new JointRelationData()
                {
                    jointIndex=i,
                    relatedJointIndex = -1,
                    relatedMuscleIndex = -1,
                });
            }
            muscleRelativedCounts=new int[muscleCount];
            for (int i = 0; i < muscles.Length; i++)
            {
                int relativeCount = 0;
                int jointIndex = muscles[i].jointIndex;
                for (int j = 0; j < jointCount; j++)
                {
                    if (jointRelationMap[jointIndex][j])
                    {
                        jointMapDataTemp.Add(new JointRelationData()
                        {
                            jointIndex = j,
                            relatedJointIndex = jointIndex,
                            relatedMuscleIndex = i,
                        });
                        relativeCount++;
                    }
      
                }
                muscleRelativedCounts[i]=relativeCount;
            }

            jointMapDatas = jointMapDataTemp.ToArray();
            parallelDataCount = jointMapDatas.Length;
        }
        private void BuildNativeDataInternal()
        {
            if (isCreated)
            {
                throw new System.Exception("Disposed NativeArray before you create it");
            }
            jointRelationDataNativeArray = new NativeArray<JointRelationData>(jointMapDatas, Allocator.Persistent);
            constraintNativeArray = new NativeArray<MMConstraintNative>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                constraintNativeArray[i]=constraints[jointMapDatas[i].jointIndex];
            }

            jointDataNativeArray = new NativeArray<JointData>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                jointDataNativeArray[i] = joints[jointMapDatas[i].jointIndex];
            }

            jointTransformNativeArray = new NativeArray<RigidTransform>(jointCount, Allocator.Persistent);
            for (int i = 0; i < jointCount; i++)
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

            muscleDataArray = new NativeArray<MMMuscleData>(muscles, Allocator.Persistent);
            muscleValueNativeArray = new NativeArray<float>(muscles.Length, Allocator.Persistent);
            muscleGradientRotationArray =new NativeArray<quaternion>(muscles.Length, Allocator.Persistent);
            muscleRelativeCountArray =new NativeArray<int>(muscleRelativedCounts, Allocator.Persistent);

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
            scheduleConstraintDataJob = new ScheduleConstraintDataJob()
            {
                jointTransforms = jointTransformNativeArray,
                jointRelationDatas = jointRelationDataNativeArray,
                Dof3s = Dof3NativeArray,
                constraints = constraintNativeArray
            };
            /*
           rigHipPositionJob = new RigHipPositionJob()
           {
               jointDatas = jointDataArray,
               constraintNatives = constraintNativeArray,
               jointTransformNatives = jointTransformNativeArray,
               jointLength = jointCount,
               loopCount = 10,
           };

           muscleToJointJob = new MuscleToJointJob()
           {
               globalDataNative= globalDataNativeArray,
               Dof3s = Dof3NativeArray,
               muscleDatas = muscleDataArray,
               musclesValues = muscleValueNativeArray,
               jointCount = jointCount,
               muscleCount = muscleCount,
           };
                        */
            buildTransformJob = new BuildTransformJob()
           {
                jointTransformNatives = jointTransformNativeArray,
               jointDatas = jointDataNativeArray,
               Dof3s = Dof3NativeArray,
               jointLength = jointCount,
           };

            caclulatelossJob = new CaclulatelossJob()
           {
                constraintNatives = constraintNativeArray,
                jointRelationDatas = jointRelationDataNativeArray,
               jointTransformNatives= jointTransformNativeArray,
               Dof3s = Dof3NativeArray,
                muscleGradientRotations=muscleGradientRotationArray,
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
                losses = losses,
                jointDatas= jointDataNativeArray,
                jointTransformNatives=jointTransformNativeArray,
                jointlossNatives = jointlossNativeArray,
                Dof3s= Dof3NativeArray,
                muscleDatas=muscleDataArray,
                muscleValues= muscleValueNativeArray,
                muscleGradientRotations = muscleGradientRotationArray,
                muscleRelativeCounts=  muscleRelativeCountArray,
                relationDatas=jointRelationDataNativeArray,
                parallelLength =parallelDataCount,
                constraintLength =constraintTransforms.Length,
                muscleLength = muscleCount,
                jointLength = jointCount,
            };

            jointToTransformJob = new JointToTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray.Slice(0, jointCount),
                Dof3s  =Dof3NativeArray.Slice(0, jointCount),
                constraintNatives=constraintNativeArray,
                jointLength=jointCount,
                muscleLength=muscleCount,
            };
        }

        #endregion
    }
}

