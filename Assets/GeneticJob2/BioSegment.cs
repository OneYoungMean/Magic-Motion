using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BIOIK2
{
    public class BioSegment : MonoBehaviour
    {
        public BIOIK2 character;//OYM:角色本身
        public BioSegment parent;//OYM:父节点
        public List< BioSegment> childs = new List<BioSegment>();//OYM:段
        public BioJoint joint = null;
        public BioObjective[] objectives = new BioObjective[0];//OYM:挂载的BioObject
                                                               // Start is called before the first frame update
        public BioSegment Create(BIOIK2 character)
        {
            this.character = character;
            hideFlags = HideFlags.HideInInspector;
            joint=GetComponent<BioJoint>();
            objectives = GetComponents<BioObjective>();
            for (int i = 0; i < objectives.Length; i++)
            {
                objectives[i].Create(this); 
            }
            if (joint!=null)
            {
                joint.Initialize(this);
            }
            return this;
        }

        public Vector3 GetAnchoredPosition()
        {
            return joint == null ? transform.position : joint.GetAnchorInWorldSpace();
        }


        public void RenewRelation()
        {
            parent = null;
            childs.Clear();
            if (transform!=character.transform)
            {
                parent = character.FindSegment(transform.parent);
                parent.AddChild(this);
            }
        }

        private void AddChild(BioSegment bioSegment)
        {
            childs.Add(bioSegment);
        }
    }

}
