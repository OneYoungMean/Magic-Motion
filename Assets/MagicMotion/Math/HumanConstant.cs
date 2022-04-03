using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MagicMotion
{
    public static class HumanConstant
    {
        //OYM£ºHip
        public static readonly float3 HipLimit = new float3(40, 40, 40);
        //OYM£ºBody
        public static readonly float3 BodyUpperLimit = new float3(40, 40, 40);
        public static readonly float3 BodyLowerLimit = new float3(-40, -40, -40);
        
        public static readonly float3x2 SpineLimit =new float3x2( BodyUpperLimit, BodyLowerLimit);
        public static readonly float3x2 ChestLimit = new float3x2(BodyUpperLimit, BodyLowerLimit);
        public static readonly float3x2 UpperChestLimit = new float3x2(BodyUpperLimit, BodyLowerLimit);
        //OYM£ºHead
        public static readonly float3 HeadUpperLimit = new float3(40, 40, 40);
        public static readonly float3 HeadLowerLimit = new float3(-40, -40, -40);

        public static readonly float3x2 NeckLimit = new float3x2(HeadUpperLimit, HeadLowerLimit);
        public static readonly float3x2 HeadLimit = new float3x2(HeadUpperLimit, HeadLowerLimit);
        public static readonly float3x2 LeftEyeLimit = new float3x2(
            new float3(-15,-20,0),
            new float3(10, 20, 0)
            );
        public static readonly float3x2 RightEyeLimit = new float3x2(
            new float3(-15, -20, 0),
            new float3(10, 20, 0)
            );


    }

}
