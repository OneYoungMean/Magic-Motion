using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BIOIK2
{
    public class BioSegment : MonoBehaviour
    {
        public Bioik2 Character;//OYM:角色本身
        public Transform Transform;//OYM:自身transform
        public BioSegment Parent;//OYM:父节点
        public BioSegment[] Childs = new BioSegment[0];//OYM:段
        public BioJoint Joint = null;
        public BioObjective[] Objectives = new BioObjective[0];//OYM:挂载的BioObject
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
