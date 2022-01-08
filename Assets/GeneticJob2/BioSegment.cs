using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BIOIK2
{
    public class BioSegment : MonoBehaviour
    {
        public Bioik2 Character;//OYM:��ɫ����
        public BioSegment parent;//OYM:���ڵ�
        public List< BioSegment> Childs = new List<BioSegment>();//OYM:��
        public BioJoint Joint = null;
        public BioObjective[] Objectives = new BioObjective[0];//OYM:���ص�BioObject
                                                               // Start is called before the first frame update
        public BioSegment Create(Bioik2 character)
        {
            Character = character;
            hideFlags = HideFlags.HideInInspector;
            return this;
        }

        public Vector3 GetAnchoredPosition()
        {
            return Joint == null ? transform.position : Joint.GetAnchorInWorldSpace();
        }


        public void RenewRelation()
        {
            parent = null;
            Childs.Clear();
            if (transform!=Character.transform)
            {
                parent = Character.FindSegment(transform.parent);
                parent.AddChild(this);
            }
        }

        private void AddChild(BioSegment bioSegment)
        {
            Childs.Add(bioSegment);
        }
    }

}
