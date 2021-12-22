using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using Unity.Collections.LowLevel.Unsafe;
namespace GeneticJob
{
    public struct MotionDataStruct
    {
        public float dataType;
        public float realTimeSenceStarUp;
        public float x;
        public float y;
        public float z;
        public float Visibility;
        public float Presence;

    }

    public interface IGetDecodeData
    {
        public MotionDataStruct GetPackStruct();
    }
    public unsafe class DataRecoder : MonoBehaviour
    {
        // Start is called before the first frame update
        public static DataRecoder Instance { get; private set; }
        private string savePath = "D:/MotionData3.bin";
        FileStream fileStream;
        ConcurrentQueue<MotionDataStruct> WriteDataQueue;
        static readonly int size = UnsafeUtility.SizeOf(typeof(MotionDataStruct));
        float currentTime;
        void Awake()
        {
            Instance = this;
            WriteDataQueue = new ConcurrentQueue<MotionDataStruct>();
            fileStream = new FileStream(savePath, FileMode.CreateNew);
            DontDestroyOnLoad(this);
        }
        private void Update()
        {
            currentTime = Time.time;
        }
        private void OnDestroy()
        {
            if (WriteDataQueue.Count != 0)
            {
                MotionDataStruct[] copyFrom = WriteDataQueue.ToArray();
                byte[] copyTo = new byte[size * copyFrom.Length];
                float[] copyCheck = new float[copyFrom.Length * 7];
                MotionDataStruct* copyFromPtr = (MotionDataStruct*)UnsafeUtility.PinGCArrayAndGetDataAddress(copyFrom, out ulong INgc);
                byte* copyToPtr = (byte*)UnsafeUtility.PinGCArrayAndGetDataAddress(copyTo, out ulong OUTgc);
                {
                    UnsafeUtility.MemCpy(copyToPtr, copyFromPtr, copyTo.Length);
                }

                fileStream.Write(copyTo, 0, copyTo.Length);
                fileStream.Flush();
                fileStream.Close();
                UnsafeUtility.ReleaseGCObject(INgc);
                UnsafeUtility.ReleaseGCObject(OUTgc);

            }

        }


        static internal void WriteToStream(IEnumerable< IGetDecodeData> poseLandmarks, float dataType)
        {
            Instance.WriteToStreamInternal(poseLandmarks, dataType);
        }

        private void WriteToStreamInternal(IEnumerable<IGetDecodeData> poseLandmarks, float dataType)
        {
            if (poseLandmarks == null)
            {
                return;
            }
            List<MotionDataStruct> dataList = new List<MotionDataStruct>();

            foreach (var item in poseLandmarks)
            {
                WriteDataQueue.Enqueue(item.GetPackStruct());
            }
        }
    }
}