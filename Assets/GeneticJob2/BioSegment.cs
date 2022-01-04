using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BIOIK2
{
    public class BioSegment : MonoBehaviour
    {
        public Bioik2 Character;//OYM:��ɫ����
        public Transform Transform;//OYM:����transform
        public BioSegment Parent;//OYM:���ڵ�
        public BioSegment[] Childs = new BioSegment[0];//OYM:��
        public BioJoint Joint = null;
        public BioObjective[] Objectives = new BioObjective[0];//OYM:���ص�BioObject
                                                               // Start is called before the first frame update
        public BioSegment Create(Bioik2 character)
        {
            Character = character;
            Transform = transform;
            hideFlags = HideFlags.HideInInspector;
            return this;
        }

        public Vector3 GetAnchoredPosition()
        {
            return Joint == null ? Transform.position : Joint.GetAnchorInWorldSpace();
        }
    }

}
