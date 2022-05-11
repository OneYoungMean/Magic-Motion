using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;

public unsafe class AsyncReadNativeArrayTest : MonoBehaviour
{
    // Start is called before the first frame update
    public int testCount = 100_0000;
    public int threadCount = 8;
    public bool isSwitch;
    public NativeArray<float> testData1;
    private TestJob testJob;
    private TestJob[] testJobTask;
    private int loopCount;

    // Update is called once per frame
    private void Start()
    {
        testData1 = new NativeArray<float>(testCount+1, Allocator.Persistent);

        testJobTask = new TestJob[threadCount];
        loopCount = testData1.Length / threadCount;
        for (int i = 0; i < threadCount; i++)
        {
            testJobTask[i] = new TestJob()
            {
                testData = testData1.Slice(loopCount * i, loopCount),
            };
        }
        testJob = new TestJob()
        {
            testData = testData1,
        };
    
    }       
    void Update()
    {
        Task [] tasks = new Task[threadCount];
        if (isSwitch)
        {
            for (int i = 0; i < threadCount; i++)
            {
                int temp = i;
                tasks[temp] = Task.Run(() =>
                {
                    testJobTask[temp].Run(loopCount);
                });
            }
            Task.WaitAll(tasks);
        }
        else
        {
           Task.Run(()=> { testJob.Schedule(testCount, 128).Complete(); });
        }
    }
    private void OnDestroy()
    {
        testData1.Dispose();
    }
    [BurstCompile]
    public struct TestJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction]
        public  NativeSlice<float> testData;
        public void Execute(int index)
        {
            testData[index] =math.cos(testData[index]);
        }
    }
}
