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
using System.Threading;
using System.Threading.Tasks;

namespace MagicMotion
{
    //OYM：第一阶段:安全的代码部分
    //OYM：第二阶段，不安全的代码片段
    //OYM：已经不安全的代码片段除外
    public  class MagicMotionKernel
    {
        public const int iteration =64;
        public static int threadCount = JobsUtility.JobWorkerCount;
        #region  NativeArrayData
        public JobHandle MainHandle;
        //OYM： Base data (read only)
        private NativeArray<JointData> jointDataNativeArray;
        private NativeArray<MuscleData> muscleDataNativeArray;
        private NativeArray<ConstraintData> constraintNativeArray;
        private NativeArray<LBFGSSolver> LBFGSNatives;
        private NativeArray<GlobalData> globalDataNativeArray;
        //OYM：Relative data(readonly)
        private NativeArray<TransformToConstraintData> transformToConstrainNativeArray;
        private NativeArray<JointRelationData> jointRelationDataNativeArray;
        private NativeArray<int> muscleRelativeCountArray;
        //OYM：base data (readwrite)
        private NativeArray<float> muscleValueNativeArray;
        private NativeArray<float3> Dof3NativeArray;
        private NativeArray<quaternion> Dof3QuaternionNativeArray;
        private NativeArray<quaternion> muscleGradientRotationArray;
        private NativeArray<RigidTransform> jointTransformNativeArray;
        private NativeArray<JoinLoss> jointlossNativeArray;
        //OYM：LBFGS data(readwrite)
        private NativeArray<double> gradientStore;
        private NativeArray<double> gradients;
        private NativeArray<double> diagonal;
        private NativeArray<double> alpha;
        private NativeArray<double> steps;
        private NativeArray<double> delta;
        private NativeArray<double> rho;
        //OYM：transform data
        private TransformAccessArray jointTransformArray;
        private TransformAccessArray constraintTransformArray;
        //OYM：test data
        private NativeArray<double> lossNativeArray;
        public NativeArray<double> gradientAllNativeArray;
        public NativeArray<float> muscleValueAllNativeArray;
        #endregion

        #region JobsData

        private TransformToConstraintJob getConstraintTransformJob;
        //private RigHipPositionJob rigHipPositionJob;
        private BuildTransformJob buildTransformJob;
        private MuscleToDof3Job muscleToDof3Job;
        private Dof3ToRotationJob dof3ToRotationJob;
        private ClacDof3EpsilionJob clacDof3EpsilionJob;
        private ScheduleConstraintDataJob scheduleConstraintDataJob;
        private CaclulatelossJob caclulatelossJob;
        private MainControllerJob mainControllerJob;
        private JointToTransformJob jointToTransformJob;

        #endregion

        #region DefaultData
        private InitializeMuscleJob initializeMuscleJob;
        private JointData[] joints;
        private MuscleData[] muscles;
        private TransformToConstraintData[] transformToConstraints;
        private ConstraintData[] constraints;
        private JointRelationData[] jointMapDatas;

        private Transform[] constraintTransforms;
        private Transform[] jointsTransforms;
        private Task loopTask;
        private List<IDisposable> disposeList;

        private int[] muscleRelativedCounts;
        private float[] muscleValues;
        private bool[][] jointRelationMap;

        public int parallelDataCount;
        public int jointCount;
        public int muscleCount;

        private bool isFinish;
        private bool isCreated;

        #endregion

        #region Accessor
        public bool IsCreated { get { return isCreated; } }


        #endregion

        #region PublicFunc
        internal void SetConstraintData(ConstraintData[] constraints, TransformToConstraintData[] transformToConstraints, Transform[] constraintTransforms)
        {
            this.constraints = constraints;
            this.constraintTransforms = constraintTransforms;
            this.transformToConstraints = transformToConstraints;
        }
        internal void SetJointData(JointData[] joints, Transform[] jointsTransforms)
        {
            this.joints = joints;
            this.jointsTransforms = jointsTransforms;
            jointCount = joints.Length;

            



        }
        internal void SetMuscleSata(MuscleData[] muscles,float[] muscleValues = null)
        {
            this.muscles = muscles;
            muscleCount = muscles.Length;

            if (muscleValues == null)
            {
                this.muscleValues = new float[muscleCount];
            }
            else
            {
                this.muscleValues = muscleValues;
            }

        }
        public void Initialize()
        {
            ValueCheck();
            BuildRelationData();
            BuildNativeDataInternal();
            BuildJobDataInternal();
            Reset();
            isCreated =true;
        }

        public void Reset()
        {
            if (!MainHandle.IsCompleted)
            {
                MainHandle.Complete();
            }
             initializeMuscleJob = new InitializeMuscleJob()
            {
                muscleDatas = muscleDataNativeArray,
                musclesValues = muscleValueNativeArray
            };
            InitializeJointJob initializeJointJob = new InitializeJointJob()
            {
                jointTransformDatas = jointTransformNativeArray,
                muscleLength = muscleCount,
                jointLength = jointCount
            };
            initializeMuscleJob.Run(muscleCount);
            initializeJointJob.RunReadOnly(jointTransformArray);
            isFinish = true;
/*            muscleToJointJob.Run(muscleCount);
            buildTransformJob.Run(muscleCount);*/

            /*            MainHandle = initializeMuscleJob.Schedule(muscleCount, 32, MainHandle);
                        MainHandle= muscleToJointJob.Schedule (muscleCount,32, MainHandle);
                        MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);*/

        }
        public void Update(float deltatime)
        {
            if (!isCreated)
            {
                return;
            }
            if (loopTask!=null&&!loopTask.IsCompleted)
            {
                return;
            }
            isFinish = false;
            for (int i = 0; i < LBFGSNatives.Length; i++)
            {
                var globalData = globalDataNativeArray[i];
                globalData.leastLoopCount = iteration;
                globalData.isInitialize = false;
                globalDataNativeArray[i] = globalData;

                var solver = LBFGSNatives[0];
                solver.state = LBFGSState.Initialize;
                solver.numberOfVariables = muscleCount;
                LBFGSNatives[0]=solver;
            }

            if (true)
            {
                getConstraintTransformJob.RunReadOnly(constraintTransformArray);
                jointToTransformJob.Schedule(jointTransformArray).Complete();
               // loopTask = Task.Run(() =>
                {
                 //  initializeMuscleJob.Run(muscleCount);
                    scheduleConstraintDataJob.Run(parallelDataCount);
                    for (int i = 0; i < iteration+1; i++)
                    {
                        muscleToDof3Job.Run(muscleCount);
                        dof3ToRotationJob.Run(jointCount);
                        buildTransformJob.Run(jointCount);
                        clacDof3EpsilionJob.Run(muscleCount);
                        caclulatelossJob.Run(parallelDataCount);
                        mainControllerJob.Run();
                    }
                    double start = lossNativeArray[iteration];
                    double end = lossNativeArray[0];
                    Debug.Log(start + " ~ " + end);
                };
                
                isFinish = true;


            }
            else
            {
   /*             this.MainHandle = getConstraintTransformJob.ScheduleReadOnly(constraintTransformArray, 32, this.MainHandle);
                this.MainHandle= scheduleConstraintDataJob.Schedule(parallelDataCount, 1, this.MainHandle);
                for (int i = 0; i < iteration; i++)
                {
                    MainHandle = muscleToDof3Job.Schedule(muscleCount, muscleCount / threadCount, MainHandle);
                    MainHandle = dof3ToRotationJob.Schedule(jointCount, jointCount / threadCount, MainHandle);
                    MainHandle = buildTransformJob.Schedule(jointCount, MainHandle);
                    MainHandle = clacDof3EpsilionJob.Schedule(muscleCount, muscleCount / threadCount, MainHandle);
                    MainHandle = caclulatelossJob.Schedule(parallelDataCount, parallelDataCount / threadCount, MainHandle);
                    MainHandle = mainControllerJob.Schedule(MainHandle);
                }
                this.MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);*/
            }


        }
        public void Dispose()
        {
            if (!MainHandle.IsCompleted)
            {
                MainHandle.Complete();
            }
            for (int i = 0; i < disposeList.Count; i++)
            {
                disposeList[i].Dispose();
            }
            disposeList = null;

            L_BFGSStatic.Disposed(diagonal, gradientStore, rho, alpha, steps, delta);
        }
        #endregion

        #region PrivateFunc
        /// <summary>
        /// check value is vaild
        /// </summary>
        /// <exception cref="Exception"></exception>
        /// <exception cref="System.Exception"></exception>
        private void ValueCheck()
        {
            if (isCreated)
            {
                Dispose();
                isCreated = false;
            }
            if (disposeList!=null)
            {
                throw new Exception("You cannot Initialize it before call Dispose");
            }
            disposeList = new List<IDisposable>();

            if (constraints.Length != joints.Length)
            {
                throw new System.Exception("constraint length, is not equal joint length");
            }

        }
        /// <summary>
        /// Build relation data ,for the gradient claculate
        /// </summary>
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
            muscleRelativedCounts=new int[jointCount];
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
                        relativeCount+= constraints[j].GetVaildCount();
                    }
      
                }
                muscleRelativedCounts[jointIndex] =relativeCount;
            }

            jointMapDatas = jointMapDataTemp.ToArray();
            parallelDataCount = jointMapDatas.Length;
        }
        /// <summary>
        /// build all native data
        /// </summary>
        /// <exception cref="System.Exception"></exception>
        private void BuildNativeDataInternal()
        {
            if (isCreated)
            {
                throw new System.Exception("Disposed NativeArray before you create it");
            }
            jointRelationDataNativeArray = CreateNativeData(jointMapDatas, Allocator.Persistent);
            constraintNativeArray = CreateNativeData<ConstraintData>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                constraintNativeArray[i]=constraints[jointMapDatas[i].jointIndex];
            }

            jointDataNativeArray = CreateNativeData<JointData>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                jointDataNativeArray[i] = joints[jointMapDatas[i].jointIndex];
            }

            jointTransformNativeArray = CreateNativeData<RigidTransform>(jointCount, Allocator.Persistent);
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

            muscleDataNativeArray = CreateNativeData(muscles, Allocator.Persistent);
            muscleValueNativeArray = CreateNativeData(muscleValues, Allocator.Persistent);
            muscleGradientRotationArray = CreateNativeData<quaternion>(muscles.Length, Allocator.Persistent);
            muscleRelativeCountArray = CreateNativeData<int>(muscleRelativedCounts, Allocator.Persistent);

            transformToConstrainNativeArray = CreateNativeData(transformToConstraints, Allocator.Persistent);

            constraintTransformArray = CreateNativeData(constraintTransforms);

            jointTransformArray = CreateNativeData(jointsTransforms);

            jointlossNativeArray = CreateNativeData<JoinLoss>(parallelDataCount, Allocator.Persistent);

            Dof3NativeArray = CreateNativeData<float3>(jointCount, Allocator.Persistent);

            Dof3QuaternionNativeArray = CreateNativeData<quaternion>(jointCount, Allocator.Persistent);

            gradients = CreateNativeData<double>(muscleCount, Allocator.Persistent);

            LBFGSNatives = CreateNativeData<LBFGSSolver>(1, Allocator.Persistent);

            globalDataNativeArray = CreateNativeData<GlobalData>(1, Allocator.Persistent);



            LBFGSNatives[0] = MagicMotion.LBFGSSolver.identity;

            L_BFGSStatic.CreateWorkVector(muscleCount, out diagonal, out gradientStore, out rho, out alpha, out steps, out delta);

            lossNativeArray= CreateNativeData<double>(iteration+1, Allocator.Persistent);
            gradientAllNativeArray = CreateNativeData<double>((iteration + 1)*muscleCount, Allocator.Persistent);
            muscleValueAllNativeArray = CreateNativeData<float>((iteration + 1) * muscleCount, Allocator.Persistent);
        }
        /// <summary>
        /// Build all job data 
        /// </summary>
        private void BuildJobDataInternal()
        {
            getConstraintTransformJob = new TransformToConstraintJob()
            {
                transformToConstrainDatas= transformToConstrainNativeArray,
                constraintDatas = constraintNativeArray,
                muscleLength = muscleCount,
                jointLength = jointCount
            };
            scheduleConstraintDataJob = new ScheduleConstraintDataJob()
            {
                jointTransforms = jointTransformNativeArray,
                jointRelationDatas = jointRelationDataNativeArray,
                Dof3s = Dof3NativeArray,
                constraintDatas = constraintNativeArray
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
               Dof3Quaternions=Dof3QuaternionNativeArray,
           };
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
                losses = lossNativeArray,
                jointlossNatives = jointlossNativeArray,
                muscleValues= muscleValueNativeArray,
                relationDatas =jointRelationDataNativeArray,
                muscleRelativeCounts=muscleRelativeCountArray,
                muscleAlls= muscleValueAllNativeArray,
                gradientAlls = gradientAllNativeArray,
                parallelLength =parallelDataCount,
                constraintLength =constraintTransforms.Length,
                muscleLength = muscleCount,
                jointLength = jointCount,
            };

            jointToTransformJob = new JointToTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray.Slice(0, jointCount),
            };
        }

        private NativeArray<T> CreateNativeData<T>(T[] arr, Allocator allocator =Allocator.Persistent) where  T: struct
        {
            var nativeArr = new NativeArray<T>(arr, allocator);
            disposeList.Add(nativeArr);
            return nativeArr;
        }
        private NativeArray<T> CreateNativeData<T>(int length, Allocator allocator = Allocator.Persistent) where T : struct
        {
            var nativeArr = new NativeArray<T>(length, allocator);
            disposeList.Add(nativeArr);
            return nativeArr;
        }
        private TransformAccessArray CreateNativeData(Transform[] transforms)
        {
            var nativeArr = new TransformAccessArray(transforms);
            disposeList.Add(nativeArr);
            return nativeArr;
        }
        #endregion

        public Keyframe[] GetmusclesKey(int index)
        {

            Keyframe[] muscles = new Keyframe[iteration + 1];
            for (int i = 0; i < iteration+1; i++)
            {
                muscles[iteration - i]=new Keyframe(1 - i /(float)iteration, muscleValueAllNativeArray[index+i * muscleCount]);
            }
            return muscles;
        }
        public Keyframe[] GetGradientsKey(int index)
        {
            Keyframe[] muscles = new Keyframe[iteration + 1];
            for (int i = 0; i < iteration + 1; i++)
            {
                muscles[iteration - i] = new Keyframe(1 - i / (float)iteration, (float)gradientAllNativeArray[index + i * muscleCount]);
            }
            return muscles;

        }
        public Keyframe[] GetLossKey()
        {
            Keyframe[] muscles = new Keyframe[iteration + 1];
            for (int i = 0; i < iteration + 1; i++)
            {
                muscles[iteration-i] = new Keyframe(1-i / (float)iteration, math.log10((float)lossNativeArray[i]));
            }
            return muscles;

        }
    }
}

