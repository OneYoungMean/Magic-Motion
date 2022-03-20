using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public class Test : MonoBehaviour
{
    [BurstCompatible]
    public struct ClacMove : IJobParallelFor
    {
        public NativeArray<Vector3> oldVertexPosition;
        public NativeArray<Vector3> currentVertexPosition;
        public NativeArray<float4> vertexWeight;
        public NativeArray<Vector3> resultMove;

        public void Execute(int index)
        {
            for (int i = 0; i < 4; i++)
            {
                resultMove[i] += (currentVertexPosition[index] - oldVertexPosition[index]) * vertexWeight[index][i];
            }
        }
    }

    // Start is called before the first frame update
    public SkinnedMeshRenderer meshRenderer;
    public Transform targetBone0, targetBone1, targetBone2, targetBone3;
    public Vector3 resultPositon0, resultPositon1, resultPositon2, resultPositon3;

    private NativeArray<Vector3> oldVertexPosition;
    private NativeArray<Vector3> currentVertexPosition;
    private NativeArray<float4> vertexWeight;
    private NativeArray<Vector3> resultMove;
    private ClacMove clacMove;

    void Start()
    {
        //OYM����¼��ʼλ��
        resultPositon0 = targetBone0!=null? targetBone0.position:Vector3.zero;
        resultPositon1= targetBone1!=null? targetBone1.position:Vector3.zero;
        resultPositon2 = targetBone2!=null? targetBone2.position:Vector3.zero;
        resultPositon3 = targetBone3!=null? targetBone3.position:Vector3.zero;

        var originPos = meshRenderer.sharedMesh.vertices;
        var boneweights = meshRenderer.sharedMesh.boneWeights;
        var allBone = meshRenderer.bones;

        int[] targetBoneIndex = new int[4];//OYM��������ֻ��¼��4��transform,������4����Ȼ��Ҳ���Ի��ĸ��࣬��Ҫ�ο���һ�º���
        for (int i = 0; i < targetBoneIndex.Length; i++)
        {
            targetBoneIndex[i] = -1;//OYM����ʼ��
        }
        //OYM����¼�±�
        for (int i = 0; i < allBone.Length; i++)
        {
            if (allBone[i]==targetBone0)
            {
                targetBoneIndex[0] = i;
            }
            if (allBone[i] == targetBone1)
            {
                targetBoneIndex[1] = i;
            }
            if (allBone[i] == targetBone2)
            {
                targetBoneIndex[2] = i;
            }
            if (allBone[i] == targetBone3)
            {
                targetBoneIndex[3] = i;
            }
        }

        oldVertexPosition = new NativeArray<Vector3>(originPos.Length, Allocator.Persistent);
        currentVertexPosition=new NativeArray<Vector3>(meshRenderer.sharedMesh.vertices, Allocator.Persistent);
        vertexWeight = new NativeArray<float4>(originPos.Length, Allocator.Persistent);//OYM������ÿ����һ��transform��Ҫ�޸�һ��float4��4�������û������float5���߸��࣬������float4x2����float4x4����

        resultMove = new NativeArray<Vector3>(4, Allocator.Persistent);//OYM��������ֻ�õ���4��transform������ֻ����4��
        //OYM����¼�±��õ���Ȩ��
        for (int i = 0; i < boneweights.Length; i++)
        {
            BoneWeight weight = boneweights[i];
            vertexWeight[i] = ClacVertexWeight(targetBoneIndex, weight);
        }
        clacMove = new ClacMove()
        {
            oldVertexPosition = oldVertexPosition,
            currentVertexPosition = currentVertexPosition,
            vertexWeight = vertexWeight,
            resultMove = resultMove
        };
    }

    // Update is called once per frame
    void Update()
    {
        oldVertexPosition.CopyFrom(currentVertexPosition);
        currentVertexPosition.CopyFrom(meshRenderer.sharedMesh.vertices);
        //OYM��������һ֡λ�ƴ�����Ӱ��
        clacMove.Schedule(64,oldVertexPosition.Length).Complete();
        resultPositon0 += resultMove[0];
        resultPositon1+=resultMove[1];
        resultPositon2 += resultMove[2];
        resultPositon3 += resultMove[3];

    }
    private void OnDestroy()
    {
        oldVertexPosition.Dispose();
        currentVertexPosition.Dispose();
        vertexWeight.Dispose();
        resultMove.Dispose();
    }

    private void OnDrawGizmos()
    {
        Gizmos.color= Color.red;
        Gizmos.DrawSphere(resultPositon0, 0.05f);
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(resultPositon1, 0.05f);
        Gizmos.color = Color.gray;
        Gizmos.DrawSphere(resultPositon2, 0.05f);
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(resultPositon3, 0.05f);
    }


    public static float4 ClacVertexWeight(int[] index, BoneWeight weight)
    {
        float4 boneWeightValue = float4.zero; //OYM�����ϸ�Ļ�����ᱨ������float4x2��Ȼ����΢��һ��API����
        for (int i = 0; i < index.Length; i++)
        {
            int targetBoneIndex = index[i];
            if (weight.boneIndex0 == targetBoneIndex)
            {
                boneWeightValue[i] = weight.weight0;
                break;
            }
            if (weight.boneIndex1 == targetBoneIndex)
            {
                boneWeightValue[i] = weight.weight1;
                break;
            }
            if (weight.boneIndex2 == targetBoneIndex)
            {
                boneWeightValue[i] = weight.weight2;
                break;
            }
            if (weight.boneIndex3 == targetBoneIndex)
            {
                boneWeightValue[i] = weight.weight3;
                break;
            }
        }
        return boneWeightValue;
    }
}
