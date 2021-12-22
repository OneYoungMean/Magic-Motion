using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetLocalRotation : MonoBehaviour
{
    public Vector3 eulerAngle;
    public Quaternion initialLocalRot;
    public void Start()
    {
        initialLocalRot = transform.localRotation;
    }
    private void Update()
    {

        transform.localRotation = initialLocalRot * Quaternion.LookRotation(Vector3.forward, eulerAngle);
    }
}
