using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SA;
using static SA.FullBodyIK;
public class SAFBIKMathTest : MonoBehaviour
{
    // Start is called before the first frame update
   public Transform targetA;
   public  Transform targetB;
    void Start()
    {
        GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.transform.parent = transform;
        box.transform.localPosition = Vector3.zero;
        box.transform.localRotation = Quaternion.identity;


    }

    // Update is called once per frame
    void Update()
    {
        Vector3 dirA = transform.position - targetA.position;
        Vector3 dirB = transform.position - targetB.position;

        SAFBIKComputeBasisFromXZLockZ(out Matrix3x3 basis, dirA.normalized, dirB.normalized);
        SAFBIKMatGetRot(out Quaternion rotation, ref basis);
        transform.rotation = rotation;
    }
}
