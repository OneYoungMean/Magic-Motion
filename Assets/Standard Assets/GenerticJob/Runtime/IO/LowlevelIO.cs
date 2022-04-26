using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.IO.LowLevel.Unsafe;
using Unity.Jobs;

namespace GeneticJob
{
    public unsafe static class LowlevelIO
    {
        public static IORawDataBuffer GetFileLoadRequest(string path)
        {
            return GetFileLoadRequests(path)[0];
        }

        public static IORawDataBuffer[] GetFileLoadRequests(params string[] paths)
        {
            int fileCount = paths.Length;
            ReadCommand[] readCommands = new ReadCommand[fileCount];
            IORawDataBuffer[] fileLoadRequests = new IORawDataBuffer[fileCount];
            ReadCommand* readCommandsPtr = (ReadCommand*)UnsafeUtility.PinGCArrayAndGetDataAddress(readCommands, out ulong handleGC); //like fixed()
            {
                for (int i = 0; i < paths.Length; i++)
                {
                    var info = new FileInfo(paths[i]);
                    if (!info.Exists)
                    {
                        Debug.LogError("Cannot Find the File :" + paths[i]);
                        continue;
                    }

                    readCommandsPtr[i].Offset = 0;
                    readCommandsPtr[i].Size = info.Length;
                    readCommandsPtr[i].Buffer = UnsafeUtility.Malloc(info.Length, 16, Allocator.Persistent);

                    fileLoadRequests[i] = new IORawDataBuffer(readCommandsPtr[i].Size, readCommandsPtr[i].Buffer, AsyncReadManager.Read(info.FullName, readCommandsPtr + i, 1));
                }
            }
            UnsafeUtility.ReleaseGCObject(handleGC);

            return fileLoadRequests;
        }

        public static byte[] CopyToRawData<T>(T[] copyFrom) where T : struct
        {
            int size = UnsafeUtility.SizeOf<T>();
            byte[] copyTo = new byte[size * copyFrom.LongLength];

            byte* copyFromPtr = (byte*)UnsafeUtility.PinGCArrayAndGetDataAddress(copyFrom, out ulong INgc);
            byte* copyToPtr = (byte*)UnsafeUtility.PinGCArrayAndGetDataAddress(copyTo, out ulong OUTgc);

            {
                UnsafeUtility.MemCpy(copyToPtr, copyFromPtr, copyTo.Length);
            }
            UnsafeUtility.ReleaseGCObject(INgc);
            UnsafeUtility.ReleaseGCObject(OUTgc);

            return copyTo;
        }
    }

    public unsafe struct IORawDataBuffer : IDisposable
    {
        public readonly long Length;
        private readonly void* rawData;
        public readonly ReadHandle readHandle;

        public IORawDataBuffer(long Length, void* rawData, ReadHandle readHandle)
        {
            this.Length = Length;
            this.rawData = rawData;
            this.readHandle = readHandle;
        }

        public void* GetUnsafePtr()
        {
            return rawData;
        }

        public JobHandle GetJobHandle()
        {
            return readHandle.JobHandle;
        }
        public int SafeyCheck()
        {
            if (!readHandle.IsValid()|| readHandle.Status == ReadStatus.Failed)
            {
                return -1;

            }
            else if (readHandle.Status != ReadStatus.Complete)
            {
                return 0;
            }
            else
            {
                return 1;
            }

        }

        public  NativeArray<T> AsNativeArray<T>(Allocator allocator = Allocator.Persistent) where T : struct
        {
            if (SafeyCheck()<=0)
            {
                Debug.LogError("operation error");
                return default(NativeArray<T>);
            }

            int size = UnsafeUtility.SizeOf<T>();

            if (Length % size != 0)
            {
                throw new InvalidOperationException("Cannot against byte");
            }
            int newLength = 0;
            checked //OYM:·ÀÖ¹Òç³ö
            {
                newLength = (int)(Length / size);
            }
            var result = new NativeArray<T>(newLength, allocator);
            
            void* resultPtr = result.GetUnsafePtr();
            UnsafeUtility.MemCpy(resultPtr, rawData, Length);

            return result;
        }

        public T[] AsArray<T>() where T : struct
        {
            if (SafeyCheck() <= 0)
            {
                Debug.LogError("operation error");
                return null;
            }

            int size = UnsafeUtility.SizeOf<T>();
            if (Length % size != 0)
            {
                throw new InvalidOperationException("Cannot against byte");
            }

            long newLength = Length / size;
            T[] resultArr = new T[newLength];
            void* resulArrPtr = UnsafeUtility.PinGCArrayAndGetDataAddress(resultArr, out ulong handleGC); //like fixed()
            {
                UnsafeUtility.MemCpy(resulArrPtr, rawData, Length);
            }
            UnsafeUtility.ReleaseGCObject(handleGC);
            return resultArr;
        }

        public void Dispose()
        {
            readHandle.Dispose();
            UnsafeUtility.Free(rawData, Allocator.Persistent);
        }
    }
}
