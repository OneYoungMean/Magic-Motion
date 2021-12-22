using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using System.IO;
using Unity.Collections.LowLevel.Unsafe;
using Unity.IO.LowLevel.Unsafe;
using System.Collections;

namespace GeneticJob
{
    public unsafe class DataEncoder: MonoBehaviour
    {
        public string MotionBinaryFile;
        IORawDataBuffer dataBuffer;
        NativeArray<MotionDataStruct> motionDataStructs;
        void Awake()
        {
            dataBuffer = LowlevelIO.GetFileLoadRequest(MotionBinaryFile);

            StartCoroutine(WaitAssetLoad());
        }

        private IEnumerator WaitAssetLoad()
        {
            if (dataBuffer.SafeyCheck()<0)
            {
                yield break;
            }
            else if (dataBuffer.SafeyCheck() == 0)
            {
                int index = 0;
                int indexCount = 500;
                yield return new WaitUntil(() => index++ > indexCount || dataBuffer.SafeyCheck()>0);
            }


            if (dataBuffer.SafeyCheck()<1)
            {
                Debug.LogError("Load time too much long , please check your asset" );
                yield break;
            }
            else
            {
                motionDataStructs = dataBuffer.AsNativeArray<MotionDataStruct>();
                dataBuffer.Dispose();
            }

        }

        private void OnDestroy()
        {
            motionDataStructs.Dispose();
        }
    }


}