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
        [Obsolete]
        private NativeArray<MusclesData> muscleDataNativeArray;
        private NativeArray<JointData> jointDataNativeArray;//OYM：所有jointData的存放位置,长度为parallelLength
        private NativeArray<ConstraintData> constraintDataNativeArray;
        private NativeArray<JointRelationData> jointRelationDataNativeArray;
        private NativeArray<int3> jointMusclesIndexNativeArray;
        private NativeArray<RigidTransform> rootTransforms;
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
        private JointRelationData[] jointMapDatas;
        private JobHandle mainHandle;

        private Action<float[]> SetMuscleCall;
        private List<IDisposable> disposeList;

        private int[] jointRelativedCounts;
        private bool[][] jointRelationMap;

        private int parallelDataCount;
        private int jointCount;
        private int muscleCount;
        private int linerSearchGroupCount;

        private int bestOptimizerIndex;
        private float bestOptimizerLoss;

        public int insideIteration;
        public int outsideIteration;
        private bool isInitialize;
        private bool isInMainThread;

        public bool IsCreated { get { return isInitialize; } }

        public int BestOptimizerIndex => bestOptimizerIndex;
        public float Loss => bestOptimizerLoss;
        #endregion

        #region PublicFunc
        public MagicMotionKernel(SearchLevel searchLevel = SearchLevel.Double,int insideIteration=16,int outsideIteration=1)
        {
            this.linerSearchGroupCount = (int)searchLevel;
            this.insideIteration = insideIteration;
            this.outsideIteration = outsideIteration;
        }
        internal void SetConstraintData(ConstraintData[] constraints)
        {
            this.constraints = constraints;
        }
        internal void SetJointData(JointData[] joints)
        {
            this.joints = joints;
            jointCount = joints.Length;
        }
        internal void SetMuscleSata(MusclesData[] muscles)
        {
            this.muscles = muscles;
            muscleCount = muscles.Length;
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
            isInitialize = true;
        }
        public void Optimize(float deltatime, float3 worldPosition, quaternion worldRotation)
        {
            if (!isInitialize)
            {
                return;
            }

            if (!mainHandle.IsCompleted)
            {
                mainHandle.Complete();
                return;
            }
            mainHandle.Complete();
            mainHandle = new JobHandle();

            bestOptimizerIndex = groupLossNativeArray[0].index;
            bestOptimizerLoss = (float)groupLossNativeArray[0].loss;

            #region Output last frame muscle Value
            //OYM：用不用JobTransform呢....
            SetMuscleCall(muscleValueSubNativeArrays[0].ToArray());
            #endregion

            #region Build Question
            var rootTransform = new RigidTransform(worldRotation, worldPosition);
            rootTransforms[0] = rootTransform;

            NativeArray<ConstraintData>.Copy(constraints, constraintDataNativeArray, constraints.Length);

            mainHandle = new JobHandle();
            mainHandle =copyConstraintDataJob.Schedule(parallelDataCount, mainHandle);
            #endregion

            #region Optimize
            if (isInMainThread)
            {
                copyConstraintDataJob.Schedule(parallelDataCount, mainHandle).Complete();
                for (int i = 0; i < outsideIteration; i++)
                {
                    for (int ii = 0; ii < linerSearchGroupCount; ii++)
                    {
                        optimizes[ii].Run();
                    }
                   sortingFactoryJob.Schedule(mainHandle).Complete();
                }
            }
            else
            {
                mainHandle = copyConstraintDataJob.Schedule(parallelDataCount, mainHandle);

                NativeArray<JobHandle> tempHandles = new NativeArray<JobHandle>(linerSearchGroupCount, Allocator.Temp);
                for (int i = 0; i < outsideIteration; i++)
                {
                    for (int ii = 0; ii < linerSearchGroupCount; ii++)
                    {
                        tempHandles[ii] = optimizes[ii].GetHandle(mainHandle);
                    }
                    mainHandle = JobHandle.CombineDependencies(tempHandles);
                    mainHandle = sortingFactoryJob.Schedule(mainHandle);
                }
                mainHandle.Complete();
                tempHandles.Dispose();
            }

            #endregion
        }
        public  void Dispose()
        {
            if (!mainHandle.IsCompleted)
            {
                mainHandle.Complete();
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
            if (isInitialize)
            {
                Dispose();
                isInitialize = false;
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
            if (isInitialize)
            {
                throw new System.Exception("Disposed NativeArray before you create it");
            }

            jointRelationDataNativeArray = CreateNativeData(jointMapDatas, Allocator.Persistent);

            constraintDataNativeArray = CreateNativeData<ConstraintData>(parallelDataCount, Allocator.Persistent);

            jointDataNativeArray = CreateNativeData<JointData>(joints, Allocator.Persistent);

            rootTransforms = CreateNativeData<RigidTransform>(1, Allocator.Persistent);

            groupLossNativeArray = CreateNativeData<GroupLossData>(linerSearchGroupCount);

            for (int i = 0; i < linerSearchGroupCount; i++)
            {
                groupLossNativeArray[i] = new GroupLossData()
                {
                    loss = 0,
                    index = i,
                };
            }

            muscleValueNativeArray = CreateNativeData<float>(muscleCount* linerSearchGroupCount); 

            muscleValueSubNativeArrays = new NativeArray<float>[(int)linerSearchGroupCount];

            for (int i = 0; i < linerSearchGroupCount; i++)
            {
                muscleValueSubNativeArrays[i] = muscleValueNativeArray.GetSubArray(i * muscleCount, muscleCount);
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
            copyConstraintDataJob = new CopyConstraintDataJob()
            {
                jointRelationDatas = jointRelationDataNativeArray,
                constraintDatas = constraintDataNativeArray,
                jointLength = jointCount,
            };
            sortingFactoryJob = new SortingFactoryJob()
            {
                musclesValue = muscleValueNativeArray,
                groupLoss = groupLossNativeArray,
                muscleLength = muscleCount,
                groupLength=linerSearchGroupCount,
            };

            optimizes = new LinerSearchGroup[linerSearchGroupCount];
            for (int i = 0; i < linerSearchGroupCount; i++)
            {
                optimizes[i] = new LinerSearchGroup
                (
                parallelDataCount, jointCount, muscleCount, insideIteration,
                jointRelativedCounts[0],
                rootTransforms,
                jointDataNativeArray,
                groupLossNativeArray.GetSubArray(i, 1),
                constraintDataNativeArray,
                jointRelationDataNativeArray,
                jointMusclesIndexNativeArray,
                muscleValueSubNativeArrays[i],
                disposeList
                );
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

        #endregion

        public Keyframe[] GetmusclesKey(int index)
        {
            return optimizes[BestOptimizerIndex].GetmusclesKey(index);

        }
        public Keyframe[] GetGradientsKey(int index)
        {
            return optimizes[BestOptimizerIndex].GetGradientsKey(index);
        }
        public Keyframe[] GetLossKey()
        {
            return optimizes[BestOptimizerIndex].GetLossKey();
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