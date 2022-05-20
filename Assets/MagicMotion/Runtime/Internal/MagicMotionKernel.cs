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
    internal class MagicMotionKernel
    {
        #region  NativeArrayData
        //OYM： Base data (read only)
    /*        [Obsolete]
            private NativeArray<MusclesData> muscleDataNativeArray;*/
        private NativeArray<JointData> jointDataNativeArray;//OYM：所有jointData的存放位置,长度为parallelLength
        private NativeArray<ConstraintData> constraintDataNativeArray;
        private NativeArray<ParallelRelationData> parallelRelationDataNativeArray;
        private NativeArray<int3> jointMusclesIndexNativeArray;
        private NativeArray<GroupSettingData> groupSettingDataNativeArray;
        private NativeArray<int> JointConstraintRelativeCountNativeArray;
        //OYM： share data (read write)
        private NativeArray<GroupLossData> groupLossNativeArray;
        private NativeArray<float> muscleValueNativeArray;
        private NativeArray<float>[] muscleValueSubNativeArrays;

        #endregion

        #region JobsData
        //private RigHipPositionJob rigHipPositionJob;
        private CopyConstraintDataJob copyConstraintDataJob;
        private SortingFactoryJob sortingFactoryJob;
        private LinerSearchGroup[] optimizes;
        #endregion

        #region DefaultData
        private JointData[] joints;
        private MusclesData[] muscles;
        private ConstraintData[] constraints;
        private ParallelRelationData[] jointMapDatas;
        private JobHandle mainHandle;

        private Action<float[]> SetMuscleCall;
        private List<IDisposable> shareDataDisposeList;
        private List<IDisposable> groupDataDisposeList;

        private int[] jointRelativedCounts;
        private bool[][] parallelRelationMap;

        private int parallelDataCount;
        private int jointCount;
        private int muscleCount;

        private int bestOptimizerIndex;
        private float bestOptimizerLoss;

        private bool isInitialize;
        public bool isInMainThread;

        private GroupSettingData SettingData { get => groupSettingDataNativeArray[0]; set =>  groupSettingDataNativeArray[0]= value; } 
        public bool IsCreated { get { return isInitialize; } }

        public int BestOptimizerIndex => bestOptimizerIndex;
        public float Loss => bestOptimizerLoss;
        #endregion

        #region PublicFunc
        public MagicMotionKernel()
        {
        }

        internal void SetData(ConstraintData[] constraints, MusclesData[] muscles, JointData[] joints, Action<float[]> SetMuscleCall)
        {
            this.constraints = constraints;
            this.joints = joints;
            this.muscles = muscles;
            this.SetMuscleCall = SetMuscleCall;

            muscleCount = muscles.Length;
            jointCount = joints.Length;
        }
        public void Initialize()
        {
            ValueCheck();
            BuildRelationData();
            BuildNativeDataInternal();
            BuildJobDataInternal();
            isInitialize = true;
        }
        public void Optimize(float deltatime, float3 worldPosition, quaternion worldRotation,float convergence=1, int innerLoopCount=64,int outsideLoopCount =1,int linerSearchGroupCount=4)
        {
            if (!isInitialize)
            {
                return;
            }

            mainHandle.Complete();

            #region Output last frame muscle Value
            //OYM：用不用JobTransform呢....
            #endregion

            #region Build Question
            GroupSettingData settingData = SettingData;
            bool isRebuildGroup = (settingData.insideLoopCount != innerLoopCount + 1 || settingData.outsideLoopCount != outsideLoopCount|| settingData.linerSearchGroupCount != linerSearchGroupCount);

            var rootTransform = new RigidTransform(worldRotation, worldPosition);
            settingData.rootTransform = rootTransform;
            settingData.insideLoopCount= innerLoopCount+1;
            settingData.outsideLoopCount= outsideLoopCount;
            settingData.outsideLoopIndex = 0; 
            settingData.loopConvergence = math.pow(10, convergence*5 / (settingData.insideLoopCount-1));
            settingData.linerSearchGroupCount = linerSearchGroupCount;
            SettingData = settingData;

            if (isRebuildGroup)
            {
                ReBuildGroupData();
            }

            bestOptimizerIndex = groupLossNativeArray[0].index;
            bestOptimizerLoss = (float)groupLossNativeArray[0].loss;
            SetMuscleCall(muscleValueSubNativeArrays[0].ToArray());

            NativeArray<ConstraintData>.Copy(constraints, constraintDataNativeArray, constraints.Length);

            mainHandle = new JobHandle();
            #endregion

            #region Optimize
            if (isInMainThread)
            {
                copyConstraintDataJob.Schedule(parallelDataCount, mainHandle).Complete();
                for (int i = 0; i < outsideLoopCount; i++)
                {
                    for (int ii = 0; ii < settingData.linerSearchGroupCount; ii++)
                    {
                        optimizes[ii].Run();
                    }
                   sortingFactoryJob.Schedule(mainHandle).Complete();
                }
            }
            else
            {
                mainHandle = copyConstraintDataJob.Schedule(parallelDataCount, mainHandle);

                NativeArray<JobHandle> tempHandles = new NativeArray<JobHandle>(settingData.linerSearchGroupCount, Allocator.Temp);
                for (int i = 0; i < outsideLoopCount; i++)
                {
                    for (int ii = 0; ii < settingData.linerSearchGroupCount; ii++)
                    {
                        tempHandles[ii] = optimizes[ii].GetHandle(mainHandle);
                    }
                    mainHandle = JobHandle.CombineDependencies(tempHandles);
                    mainHandle = sortingFactoryJob.Schedule(mainHandle);
                }

                tempHandles.Dispose();
            }

            #endregion
        }
        public  void Dispose()
        {
            mainHandle.Complete();
            if (groupDataDisposeList!=null)
            {
                DisposeGroupData();
            }

            for (int i = 0; i < shareDataDisposeList?.Count; i++)
            {
                shareDataDisposeList[i].Dispose();
            }
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
            if (isInitialize)
            {
                Dispose();
                isInitialize = false;
            }
            if (groupDataDisposeList != null)
            {
                throw new Exception("You cannot Initialize it before call Dispose");
            }
            groupDataDisposeList = new List<IDisposable>();

            if (constraints.Length != joints.Length)
            {
                throw new System.Exception("constraint length, is not equal joint length");
            }
            if (SetMuscleCall == null)
            {
                throw new System.Exception("SetMuscleCall is empty");
            }
            if (groupDataDisposeList==null)
            {
                groupDataDisposeList = new List<IDisposable>();
            }
            if (shareDataDisposeList==null)
            {
                shareDataDisposeList = new List<IDisposable>();
            }
        }
        /// <summary>
        /// Build relation data ,for the gradient claculate
        /// </summary>
        private void BuildRelationData()
        {
            List<ParallelRelationData> jointMapDataTemp = new List<ParallelRelationData>();

            parallelRelationMap = new bool[jointCount][];
            for (int i = 0; i < jointCount; i++)
            {
                parallelRelationMap[i] = new bool[jointCount];
            }

            for (int i = 0; i < jointCount; i++)
            {
                int parentIndex = i;
                while (parentIndex != -1)
                {
                    parallelRelationMap[parentIndex][i] = true;
                    parentIndex = joints[parentIndex].parentIndex;
                }
            }
            jointRelativedCounts = new int[jointCount];
            int allConstraintCount = 0;
            for (int i = 0; i < jointCount; i++)
            {
                jointMapDataTemp.Add(new ParallelRelationData()
                {
                    jointIndex = i,
                    relatedJointIndex = -1,
                    relatedMuscleIndex = -1,
                    parentJointIndex = joints[i].parentIndex,
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
                    if (parallelRelationMap[jointIndex][ii])
                    {
                        jointMapDataTemp.Add(new ParallelRelationData()
                        {
                            jointIndex = ii,
                            relatedJointIndex = jointIndex,
                            relatedMuscleIndex = i,
                            parentJointIndex=joints[ii].parentIndex,
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
            if (isInitialize)
            {
                throw new System.Exception("Disposed NativeArray before you create it");
            }

            parallelRelationDataNativeArray = CreateNativeData(jointMapDatas, Allocator.Persistent);

            constraintDataNativeArray = CreateNativeData<ConstraintData>(parallelDataCount, Allocator.Persistent);

            jointDataNativeArray = CreateNativeData<JointData>(joints, Allocator.Persistent);

            JointConstraintRelativeCountNativeArray = CreateNativeData(jointRelativedCounts, Allocator.Persistent);

            groupSettingDataNativeArray = CreateNativeData<GroupSettingData>(1, Allocator.Persistent);
            {
                groupSettingDataNativeArray[0] = new GroupSettingData(jointRelativedCounts[0], parallelDataCount, muscleCount, jointCount);
            }

            jointMusclesIndexNativeArray = CreateNativeData<int3>(jointCount);
            for (int i = 0; i < jointCount; i++)
            {
                jointMusclesIndexNativeArray[i] = new int3(-1);
            }
            for (int i = 0; i < muscleCount; i++)
            {
                var muscle =muscles[i];
                int3 data = jointMusclesIndexNativeArray[muscle.jointIndex];
                data[muscle.dof] = i;
                jointMusclesIndexNativeArray[muscle.jointIndex] = data;
            }
        }
        /// <summary>
        /// Build all job data 
        /// </summary>
        private void BuildJobDataInternal()
        {
            //OYM：啊怎么才能把这个干掉
            unsafe
            {
                copyConstraintDataJob = new CopyConstraintDataJob()
                {
                    parallelRelationDatas =(ParallelRelationData*) parallelRelationDataNativeArray.GetUnsafeReadOnlyPtr(),
                    constraintDatas =(ConstraintData*) constraintDataNativeArray.GetUnsafePtr(),
                    jointLength = jointCount,
                };
            }
        }

        private void ReBuildGroupData()
        {
            NativeArray<T> CreateNativeData<T>(int length) where T:struct
                {
                    var nativeArr = new NativeArray<T>(length, Allocator.Persistent);
                    shareDataDisposeList.Add(nativeArr);
                    return nativeArr;
                }
            DisposeGroupData();

            groupLossNativeArray = CreateNativeData<GroupLossData>(SettingData.linerSearchGroupCount);

            for (int i = 0; i < SettingData.linerSearchGroupCount; i++)
            {
                groupLossNativeArray[i] = new GroupLossData()
                {
                    loss = 0,
                    index = i,
                };
            }

            muscleValueNativeArray = CreateNativeData<float>(SettingData.muscleLength * SettingData.linerSearchGroupCount);

            muscleValueSubNativeArrays = new NativeArray<float>[SettingData.linerSearchGroupCount];

            for (int i = 0; i < SettingData. linerSearchGroupCount; i++)
            {
                muscleValueSubNativeArrays[i] = muscleValueNativeArray.GetSubArray(i * muscleCount, muscleCount);
            }
            optimizes = new LinerSearchGroup[SettingData.linerSearchGroupCount];
            for (int i = 0; i < SettingData.linerSearchGroupCount; i++)
            {
                optimizes[i] = new LinerSearchGroup
                (
                groupSettingDataNativeArray,
                jointDataNativeArray,
                groupLossNativeArray.GetSubArray(i, 1),
                constraintDataNativeArray,
                parallelRelationDataNativeArray,
                JointConstraintRelativeCountNativeArray,
                jointMusclesIndexNativeArray,
                muscleValueSubNativeArrays[i],
                groupDataDisposeList
                );
                groupDataDisposeList.Add(optimizes[i]);
            }
            sortingFactoryJob = new SortingFactoryJob()
            {
                musclesValue = muscleValueNativeArray,
                groupLoss = groupLossNativeArray,
                muscleLength = muscleCount,
                groupLength = SettingData.linerSearchGroupCount,
                settingData = groupSettingDataNativeArray
            };
        }

        private void DisposeGroupData()
        {
            for (int i = 0; i < groupDataDisposeList.Count; i++)
            {
                groupDataDisposeList[i].Dispose();
            }
            groupDataDisposeList.Clear();
        }

        private NativeArray<T> CreateNativeData<T>(T[] arr, Allocator allocator = Allocator.Persistent) where T : struct
        {
            var nativeArr = new NativeArray<T>(arr, allocator);
            shareDataDisposeList.Add(nativeArr);
            return nativeArr;
        }
        private NativeArray<T> CreateNativeData<T>(int length, Allocator allocator = Allocator.Persistent) where T : struct
        {
            var nativeArr = new NativeArray<T>(length, allocator);
            shareDataDisposeList.Add(nativeArr);
            return nativeArr;
        }
        private TransformAccessArray CreateNativeData(Transform[] transforms)
        {
            var nativeArr = new TransformAccessArray(transforms);
            shareDataDisposeList.Add(nativeArr);
            return nativeArr;
        }

        #endregion

/*        public Keyframe[] GetmusclesKey(int index)
        {
            return optimizes[BestOptimizerIndex].GetmusclesKey(index);

        }
        public Keyframe[] GetGradientsKey(int index)
        {
            return optimizes[BestOptimizerIndex].GetGradientsKey(index);
        }*/
        public Keyframe[] GetLossKey()
        {
            var result= optimizes[BestOptimizerIndex].GetLossKey();
            Debug.Log(result[0].value-result[result.Length-1].value);
            return result;
        }
    }
}
#region note
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
                    MainHandle = muscleToTransformJob.Schedule(jointCount, jointCount / threadCount, MainHandle);
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
               muscleCurrentRotation=Dof3QuaternionNativeArray,
           };
            muscleToDof3Job = new MuscleToDof3Job()
            {
                Dof3s = Dof3NativeArray,
                muscleDatas = muscleDataNativeArray,
                muscleValues = muscleValueNativeArray,
            };

            muscleToTransformJob = new muscleToTransformJob
            {
                Dof3s = Dof3NativeArray,
                jointDatas = jointDataNativeArray,
                muscleCurrentRotation = Dof3QuaternionNativeArray,
            };

            clacDof3EpsilionJob = new ClacDof3EpsilionJob
            {
                muscleCurrentRotation = Dof3QuaternionNativeArray,
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
                parallelRelationDatas = parallelRelationDataNativeArray,
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
                relationDatas =parallelRelationDataNativeArray,
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
private muscleToTransformJob muscleToTransformJob;
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
#endregion