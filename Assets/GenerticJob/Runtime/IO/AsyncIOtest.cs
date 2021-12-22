using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

using Unity.Collections;
using Unity.IO.LowLevel.Unsafe;
using Unity.Collections.LowLevel.Unsafe;
using System;

public class AsyncIOtest : MonoBehaviour
{
    // Start is called before the first frame update
    FileInfo[] info;
    DirectoryInfo di;
    private ReadHandle[] readHandles;
    NativeArray<ReadCommand> cmds;
    public LoadType loadType;
    public Texture2D[] textures;
    bool isDispose;
    public enum LoadType
    {
        jpg,
        png
    }

    System.Diagnostics.Stopwatch sw;
    unsafe void Start()
    {
        List<long> list = new List<long>();
        DirectoryInfo di = new DirectoryInfo(Application.dataPath);
        info = di.GetFiles("*."+ loadType.ToString(), SearchOption.AllDirectories);
        textures = new Texture2D[info.Length];
        cmds = new NativeArray<ReadCommand>(info.Length, Allocator.Persistent);
        readHandles = new ReadHandle[info.Length];
        var pReadCommand = (ReadCommand*)cmds.GetUnsafePtr();

        for (int i = 0; i < info.Length; i++)
        {
            string filePath = info[i].FullName;
            ReadCommand cmd;
            cmd.Offset = 0;
            cmd.Size = info[i].Length;
            cmd.Buffer = (byte*)UnsafeUtility.Malloc(cmd.Size, 16, Allocator.Persistent);
            cmds[i] = cmd;

            readHandles[i] = AsyncReadManager.Read(filePath, pReadCommand + i, 1);

        }
    }

    // Update is called once per frame
    unsafe void Update()
    {
        for (int i = 0; i < cmds.Length; i++)
        {
            if (textures[i] == null  && readHandles[i].Status != ReadStatus.InProgress)
            {
                //Debug.LogFormat(info[i].Name + " Read {0} ", readHandles[i].Status == ReadStatus.Complete ? "Successful" : "Failed");

                byte[] ramTex = new byte[cmds[i].Size];
                byte* pByte = (byte*)cmds[i].Buffer;

                fixed (byte* pRamTex = &ramTex[0])
                {
                    Unity.Collections.LowLevel.Unsafe.UnsafeUtility.MemCpy(pRamTex, pByte, cmds[i].Size);//OYM：这个方法比for快两个数量级
                }

                textures[i] = new Texture2D(2, 2);
                textures[i].LoadImage(ramTex);//OYM：这个方法是真的慢
                textures[i].name = info[i].Name;
                UnsafeUtility.Free(cmds[i].Buffer, Allocator.Persistent);
            }
        }

    }
    private void OnDestroy()
    {
        cmds.Dispose();
    }

}
