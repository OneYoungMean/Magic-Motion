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
    using Internal;
    public enum SearchLevel
    {
        Single = 1,
        Double = 2,
        Quater = 4,
        Oct = 8,
        Hex = 16,
    }
    //OYM：第一阶段:安全的代码部分
    //OYM：第二阶段，不安全的代码片段
    //OYM：已经不安全的代码片段除外
    public class MagicMotionKernel
    {

        public const int iterationCount = 32;
        public const int BatchCount = 4;
        public static int threadCount = JobsUtility.JobWorkerCount;
        #region  NativeArrayData
        //OYM： Base data (read only)
        private NativeArray<JointData> jointDataNativeArray;//OYM：所有jointData的存放位置,长度为parallelLength
        private NativeArray<MuscleData> muscleDataNativeArray;
        private NativeArray<ConstraintData> constraintDataNativeArray;

        //OYM：Relative data(readonly)
        private NativeArray<TransformToConstraintData> transformToConstrainNativeArray;
        private NativeArray<JointRelationData> jointRelationDataNativeArray;
        private NativeArray<int> muscleRelativeCountNativeArray;
        //OYM： muslceData
        private NativeArray<float>[] muscleValueNativeArrays;

        //OYM：transform data
        private NativeArray<Vector3> jointPositionNativeArray;
        private NativeArray<Vector3> constraintPositionNativeArray;
        private NativeArray<Quaternion> jointRotationNativeArray;
        private NativeArray<Quaternion> constraintRotationNativeArray;
        #endregion

        #region JobsData
        private TransformToConstraintJob getConstraintTransformJob;
        //private RigHipPositionJob rigHipPositionJob;
        private BuildConstraintDataJob buildConstraintDataJob;

        private MuscleOptimizer[] optimizes;

        #endregion

        #region DefaultData

        private InitializeMuscleJob initializeMuscleJob;
        private JointData[] joints;
        private MuscleData[] muscles;
        private TransformToConstraintData[] transformToConstraints;
        private ConstraintData[] constraints;
        private JointRelationData[] jointMapDatas;

        private Task waitTask;
        private Task[] loopTasks;
        private Action[] loopActions;
        private List<IDisposable> disposeList;


        private int[] jointRelativedCounts;
        private float[] muscleValues;
        private bool[][] jointRelationMap;

        public int parallelDataCount;
        public int jointCount;
        public int muscleCount;

        private volatile float bestLoss;
        private volatile int bestOptimizerIndex;

        private bool isCreated;

        private Action<float[]> SetMuscleCall;
        private int searchLevel;


        public bool IsCreated { get { return isCreated; } }

        #endregion

        #region PublicFunc
        public MagicMotionKernel(SearchLevel searchLevel = SearchLevel.Double)
        {
            this.searchLevel = (int)searchLevel;
        }
        internal void SetConstraintData(ConstraintData[] constraints, TransformToConstraintData[] transformToConstraints)
        {
            this.constraints = constraints;
            this.transformToConstraints = transformToConstraints;
        }
        internal void SetJointData(JointData[] joints)
        {
            this.joints = joints;
            jointCount = joints.Length;
        }
        internal void SetMuscleSata(MuscleData[] muscles, float[] muscleValues = null)
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

        internal void SetOutDataFunc(Action<float[]> SetMuscleCall)
        {
            this.SetMuscleCall = SetMuscleCall;
        }
        public void Initialize()
        {
            ValueCheck();
            BuildRelationData();
            BuildNativeDataInternal();
            BuildJobDataInternal();
            isCreated = true;
        }

        public  void Optimize(float deltatime, Vector3[] jointPositions, Quaternion[] jointRotations, Vector3[] constraintPositions, Quaternion[] constraintRotations)
        {
            //OYM：未来要异步的部分,尽量不要涉及mono的内容
            if (!isCreated)
            {
                return;
            }
            if (waitTask != null && !waitTask.IsCompleted)
            {
                return;
            }
            waitTask = Task.Run(() =>
              {
                  #region Transform Data To NativeArray
                  jointPositionNativeArray.CopyFrom(jointPositions);
                  jointRotationNativeArray.CopyFrom(jointRotations);
                  constraintPositionNativeArray.CopyFrom(constraintPositions);
                  constraintRotationNativeArray.CopyFrom(constraintRotations);
                  #endregion

                  #region Build Question
                  buildConstraintDataJob.Dof3s = optimizes[bestOptimizerIndex].Dof3s;
                  getConstraintTransformJob.Run(transformToConstrainNativeArray.Length);
                  buildConstraintDataJob.Run(parallelDataCount);

                  #endregion

                  #region Optimize

                  for (int i = 0; i < searchLevel; i++)
                  {
                      for (int ii = 0; ii < muscleCount; ii++)
                      {
                          float refValue = muscleValueNativeArrays[0][ii];
                          muscleValueNativeArrays[i][ii] = GetSearchValue(i, refValue);
                      }
                  }

                  bestLoss = float.MaxValue;
                  
                  for (int i = 0; i < searchLevel; i++)
                  {
                      int end = (i + 1) % BatchCount;
                      loopTasks[end] = Task.Run((loopActions[i]));
                      if (end == 0||i==searchLevel-1)
                      {
                          Task.WaitAll(loopTasks);
                      }
                  }


                  #endregion

                  #region Output muscle Value
                  if (muscleValueNativeArrays[0].IsCreated)
                  {
                      if (bestOptimizerIndex != 0)
                      {
                          muscleValueNativeArrays[0].CopyFrom(muscleValueNativeArrays[bestOptimizerIndex]);
                      }
                      SetMuscleCall(muscleValueNativeArrays[0].ToArray());
                  }

            #endregion
        }
            );
        }
        public async void Dispose()
        {
            if (waitTask!=null&&!waitTask.IsCompleted)
            {
                await waitTask;
            }

            for (int i = 0; i < disposeList.Count; i++)
            {
                disposeList[i].Dispose();
            }
            disposeList = null;
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
            if (disposeList != null)
            {
                throw new Exception("You cannot Initialize it before call Dispose");
            }
            disposeList = new List<IDisposable>();

            if (constraints.Length != joints.Length)
            {
                throw new System.Exception("constraint length, is not equal joint length");
            }
            if (SetMuscleCall == null)
            {
                throw new System.Exception("SetMuscleCall is empty");
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
                int parentIndex = i;
                while (parentIndex != -1)
                {
                    jointRelationMap[parentIndex][i] = true;
                    parentIndex = joints[parentIndex].parentIndex;
                }
            }
            jointRelativedCounts = new int[jointCount];
            int allConstraintCount = 0;
            for (int i = 0; i < jointCount; i++)
            {
                jointMapDataTemp.Add(new JointRelationData()
                {
                    jointIndex = i,
                    relatedJointIndex = -1,
                    relatedMuscleIndex = -1,
                });
                allConstraintCount += constraints[i].GetVaildCount();
            }
            jointRelativedCounts[0] = allConstraintCount;


            for (int i = 0; i < muscles.Length; i++)
            {
                int relativeCount = 0;
                int jointIndex = muscles[i].jointIndex;
                for (int ii = 0; ii < jointCount; ii++)
                {
                    if (jointRelationMap[jointIndex][ii])
                    {
                        jointMapDataTemp.Add(new JointRelationData()
                        {
                            jointIndex = ii,
                            relatedJointIndex = jointIndex,
                            relatedMuscleIndex = i,
                        });
                        relativeCount += constraints[ii].GetVaildCount();
                    }

                }
                jointRelativedCounts[jointIndex] = relativeCount;
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

            constraintDataNativeArray = CreateNativeData<ConstraintData>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                constraintDataNativeArray[i] = constraints[jointMapDatas[i].jointIndex];
            }

            jointDataNativeArray = CreateNativeData<JointData>(parallelDataCount, Allocator.Persistent);
            for (int i = 0; i < parallelDataCount; i++)
            {
                jointDataNativeArray[i] = joints[jointMapDatas[i].jointIndex];
            }

            transformToConstrainNativeArray = CreateNativeData(transformToConstraints, Allocator.Persistent);
            muscleDataNativeArray = CreateNativeData(muscles, Allocator.Persistent);

            muscleRelativeCountNativeArray = CreateNativeData<int>(jointRelativedCounts, Allocator.Persistent);

            jointPositionNativeArray = CreateNativeData<Vector3>(jointCount);
            jointRotationNativeArray = CreateNativeData<Quaternion>(jointCount);

            constraintPositionNativeArray = CreateNativeData<Vector3>(transformToConstraints.Length);
            constraintRotationNativeArray = CreateNativeData<Quaternion>(transformToConstraints.Length);
            muscleValueNativeArrays = new NativeArray<float>[(int)searchLevel];

            for (int i = 0; i < searchLevel; i++)
            {
                muscleValueNativeArrays[i] = CreateNativeData<float>(muscleCount);
            }
        }
        /// <summary>
        /// Build all job data 
        /// </summary>
        private void BuildJobDataInternal()
        {
            getConstraintTransformJob = new TransformToConstraintJob()
            {
                transformToConstrainDatas = transformToConstrainNativeArray,
                constraintPositions = constraintPositionNativeArray,
                constraintRotations = constraintRotationNativeArray,
                constraintDatas = constraintDataNativeArray,
            };

            buildConstraintDataJob = new BuildConstraintDataJob()
            {
                jointPositions = jointPositionNativeArray,
                jointRotations = jointRotationNativeArray,
                jointRelationDatas = jointRelationDataNativeArray,
                constraintDatas = constraintDataNativeArray,
            };

            optimizes = new MuscleOptimizer[searchLevel];
            loopActions = new Action[searchLevel];

            for (int i = 0; i < searchLevel; i++)
            {
                optimizes[i] = new MuscleOptimizer
                (
                parallelDataCount, jointCount, muscleCount, iterationCount,
                muscleRelativeCountNativeArray,
                jointDataNativeArray,
                muscleDataNativeArray,
                constraintDataNativeArray,
                jointRelationDataNativeArray,
                muscleValueNativeArrays[i],
                disposeList
                );

                int temp = i;
                loopActions[i] = () => Opimize(temp);
            }

            loopTasks = new Task[BatchCount];
            for (int i = 0; i < loopTasks.Length; i++)
            {
                loopTasks[i] = Task.CompletedTask;
            }
        }
        private void Opimize(int index)
        {
            optimizes[index].Run();
            if (optimizes[index].Loss < bestLoss)
            {
                bestLoss = optimizes[index].Loss;
                bestOptimizerIndex = index;
            }
        }
        private NativeArray<T> CreateNativeData<T>(T[] arr, Allocator allocator = Allocator.Persistent) where T : struct
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

        private static float GetSearchValue(int index, float refValue)
        {
            switch (index)
            {
                case 0:
                    break;
                case 1:
                    refValue = 0;
                    break;
                case 2:
                    refValue = -1;
                    break;
                case 3:
                    refValue = 1;
                    break;
                case 4:
                    refValue = math.lerp(refValue, -1, 0.01f);
                    break;
                case 5:
                    refValue = math.lerp(refValue, 1, 0.01f);
                    break;
                case 6:
                    refValue = math.lerp(refValue, -1, 0.1f);
                    break;
                case 7:
                    refValue = math.lerp(refValue, 1, 0.1f);
                    break;
                case >= 8 and < 16:
                    refValue =math.clamp(refValue* index/16f,-1,1);
                    break;
                case >= 16 and < 32:
                    refValue = math.lerp(-1, 1, (index-16 )/ 16f);
                    break;
                default:
                    break;
            }
            return refValue;
        }

        #endregion

        /*        public Keyframe[] GetmusclesKey(int index)
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
                        muscles[iteration-i] = new Keyframe(1-i / (float)iteration, (float)lossNativeArray[i]);
                    }
                    return muscles;

                }*/
    }
}

/*          for (int i = 0; i < muscleCount; i++)
            {
                searchPoint[2][i]=(searchPoint[0][i]+1)/2;
                searchPoint[3][i]= (searchPoint[0][i] - 1) / 2;
            }
            if (true)
            {
*//*                getConstraintTransformJob.RunReadOnly(constraintTransformArray);*//*
                //  jointToTransformJob.Schedule(jointTransformArray).Complete();
                
                
                 //Task.Run(() =>
                {
                    //  initializeMuscleJob.Run(muscleCount);

                }
                //);
                
                isFinish = true;


            }
            else
            {
   *//*             this.MainHandle = getConstraintTransformJob.ScheduleReadOnly(constraintTransformArray, 32, this.MainHandle);
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
                this.MainHandle = jointToTransformJob.Schedule(jointTransformArray, MainHandle);*//*
            }

*/
/*          
muscleValueNativeArray = CreateNativeData(muscleValues, Allocator.Persistent);
muscleGradientRotationArray = CreateNativeData<quaternion>(muscles.Length, Allocator.Persistent);

constraintTransformArray = CreateNativeData(constraintTransforms);

jointTransformArray = CreateNativeData(jointsTransforms);

jointlossNativeArray = CreateNativeData<JoinLoss>(parallelDataCount, Allocator.Persistent);

Dof3NativeArray = CreateNativeData<float3>(jointCount, Allocator.Persistent);

Dof3QuaternionNativeArray = CreateNativeData<quaternion>(jointCount, Allocator.Persistent);

gradients = CreateNativeData<double>(muscleCount, Allocator.Persistent);

LBFGSNatives = CreateNativeData<LBFGSSolver>(1, Allocator.Persistent);

globalDataNativeArray = CreateNativeData<GlobalData>(1, Allocator.Persistent);

LBFGSNatives[0] = MagicMotion.LBFGSSolver.identity;

diagonal = CreateNativeData<double>(muscleCount, Allocator.Persistent);
gradientStore = CreateNativeData<double>(muscleCount, Allocator.Persistent);
rho = CreateNativeData<double>(L_BFGSStatic. CORRECTION, Allocator.Persistent);                  // Stores the scalars rho.
alpha = CreateNativeData<double>(L_BFGSStatic.CORRECTION, Allocator.Persistent);               // Stores the alphas in computation of H*g.
steps = CreateNativeData<double>(muscleCount * L_BFGSStatic.CORRECTION, Allocator.Persistent);          // Stores the last M search steps.
delta = CreateNativeData<double>(muscleCount * L_BFGSStatic.CORRECTION, Allocator.Persistent);

lossNativeArray = CreateNativeData<double>(iteration+1, Allocator.Persistent);
gradientAllNativeArray = CreateNativeData<double>((iteration + 1)*muscleCount, Allocator.Persistent);
muscleValueAllNativeArray = CreateNativeData<float>((iteration + 1) * muscleCount, Allocator.Persistent);*/
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
/*            buildTransformJob = new BuildTransformJob()
           {
                jointTransformNatives = jointTransformNativeArray,
               jointDatas = jointDataNativeArray.Slice(0,jointCount),
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
                muscleAlls= muscleValueAllNativeArray,
                gradientAlls = gradientAllNativeArray,
                parallelLength =parallelDataCount,
                constraintLength =constraintTransforms.Length,
                muscleLength = muscleCount,
                jointLength = jointCount,
            };*/

/*            jointToTransformJob = new JointToTransformJob()
            {
                jointTransformNatives = jointTransformNativeArray.Slice(0, jointCount),
            };*/
/*        
        private TransformAccessArray jointTransformArray;
        private TransformAccessArray constraintTransformArray;
*/

//OYM：base data (readwrite)
/*        private NativeArray<LBFGSSolver> LBFGSNatives;
        private NativeArray<GlobalData> globalDataNativeArray;
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

        //OYM：test data
        private NativeArray<double> lossNativeArray; 
        public NativeArray<double> gradientAllNativeArray;
        public NativeArray<float> muscleValueAllNativeArray;*/
/*        /// <summary>
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
private MainControllerJob mainControllerJob;*/