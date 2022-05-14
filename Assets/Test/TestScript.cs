using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MagicMotion.Mono;
public class TestScript : MonoBehaviour
{
    public class MediaPipeData
    {
        public HumanBodyBones humanBody;
       public float weight;
       public Vector3 position;

        public MediaPipeData(HumanBodyBones humanBody, float weight, Vector3 position)
        {
            this.humanBody = humanBody;
                this.weight = weight;
            this.position = position;
        }
    }
    // Start is called before the first frame update
    public MMHumanGenerator humanGenerator;
    Dictionary<HumanBodyBones, MediaPipeData> mediaPipeDatas =new Dictionary<HumanBodyBones, MediaPipeData>()
        {
        { HumanBodyBones.LeftUpperLeg, new MediaPipeData(HumanBodyBones.LeftUpperLeg,1,new Vector3 (0,1,0)) },
       { HumanBodyBones.LeftUpperArm, new MediaPipeData(HumanBodyBones.LeftUpperArm,1,new Vector3 (0,1,0)) },
        };
    public bool isInitialize;
    
    void Start()
    {
        if (humanGenerator==null)
        {
            return;
        }
        isInitialize= true;

    }

    // Update is called once per frame
    void Update()
    {
        if (!isInitialize)
        {
            return;
        }
        for (int i = 0; i <(int) HumanBodyBones.LastBone; i++)
        {
            HumanBodyBones humanBody = (HumanBodyBones)i;
            MMConstraint constraint = humanGenerator.GetConstraintTarget(humanBody, MagicMotion.MMConstraintType.Position);
            if (constraint != null)
            {
                MMPositionConstraint positionConstraint = constraint as MMPositionConstraint;
                if (positionConstraint!=null)
                {
                    if (mediaPipeDatas.TryGetValue(humanBody, out MediaPipeData mediaPipeData))
                    {
                        positionConstraint.TargetTransform.position = mediaPipeData.position;
                        positionConstraint.weight3 = Vector3.one * mediaPipeData.weight;
                    }
                    else
                    {
                        positionConstraint.weight3 = Vector3.zero;
                    }
                }
            }
        }
    }
}
