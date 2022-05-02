using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;

namespace MagicMotion.Mono
{
    public class MMConstraintsController : MonoBehaviour
    {
        public const float LOOKAT_LENGTH = 0.1f;
        public bool isInitialize;
        public List<MMConstraint> constraints;

        private ConstraintData[] constraintNatives;
        private Transform[] constraintTransform;
        public void Initialize(MMJoint[] joints)
        {
            if(!isInitialize)
            {
            }
            isInitialize = true;
        }

        internal ConstraintData[] GetNativeDatas()
        {
            constraintNatives = new ConstraintData[constraints.Count];
/*            for (int i = 0; i < constraintNatives.Length; i++)
            {
                constraintNatives[i]=constraints[i].GetNativeData();
            }*/
            return constraintNatives;
        }
        public Transform[] GetTransforms()
        {
            constraintTransform = new Transform[constraints.Count];
            for (int i = 0; i < constraintTransform.Length; i++)
            {
                constraintTransform[i] = constraints[i].transform;
            }
            return constraintTransform;
        }
    }

}
