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

        private MMJointConstraintNative[] constraintNatives;
        private Transform[] constraintTransform;
        public void Initialize(MMJoint[] joints)
        {
            if(!isInitialize)
            {
                constraints=new List<MMConstraint>();

                for (int i = 0; i < joints.Length; i++)
                {
                    var joint = joints[i];
                    GameObject positionIK = new GameObject("IK_" + joints[i].name + "_Position");
                    positionIK.transform.parent = transform;
                    positionIK.transform.position = joint.transform.position;
                    MMConstraint positionConstriant = positionIK.AddComponent<MMPositionConstraint>();
                    positionConstriant.Initialize();

                    joint.constraints[(int)positionConstriant.GetConstraintType()] =positionConstriant;
                    constraints.Add(positionConstriant);
                }

                for (int i = 0; i < joints.Length; i++)
                {
                    var joint = joints[i];
                    GameObject lookAtIK = new GameObject("IK_" + joints[i].name + "_Lookat");
                    lookAtIK.transform.parent = transform;
                    lookAtIK.transform.position = joint.transform.position+LOOKAT_LENGTH* joint.transform.forward;
                    MMConstraint lookatConstriant = lookAtIK.AddComponent<MMLookConstraint>();
                    lookatConstriant.Initialize();

                    joint.constraints[(int)lookatConstriant.GetConstraintType()] = lookatConstriant;
                    constraints.Add(lookatConstriant);
                }
            }
            isInitialize = true;
        }

        public void Regular()
        {
            for (int i = 0; i < constraints.Count; i++)
            {
                if (constraints[i]==null)
                {
                    constraints.RemoveAt(i);
                    i--;
                }
                constraints[i].index = i;
            }
        }

        public MMJointConstraintNative[] GetNativeDatas()
        {
            constraintNatives = new MMJointConstraintNative[constraints.Count];
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
