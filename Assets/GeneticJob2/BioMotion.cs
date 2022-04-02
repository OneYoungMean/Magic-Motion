using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using static Unity.Mathematics.math;
namespace BioIK2
{
    public enum MotionType
    {
        Fast,//OYM:Instantaneous
        Slow,//OYM:Realistic
    }
    public class BioMotion : MonoBehaviour
    {
        public BioJoint joint;
        public float3 isEnableValue=1;
        public float3 constraintValue = 1;//OYM:Լ��?Լ��ɶ?
        [SerializeField]
        private float3 lowerLimit=-15;
        [SerializeField]
        private float3 upperLimit=15;
        public float3 targetValue;
        public float3 currentValue;
        public float3 currentError;
        public float3 currentVelocity;
        public float3 currentAcceleration;
        public float3x3 axisMat;
        public float maximumVelocity => (joint.jointType == BioJointType.Rotational ? Mathf.Rad2Deg :1f)*joint.segment.character.maximumVelocity;
        public float maximumAcceleration => (joint.jointType == BioJointType.Rotational ? Mathf.Rad2Deg : 1f) * joint.segment.character.maximumAcceleration;

        public const float SPEEDUP = 1;
        public const float SLOWDOWN = 1;

        public void Create(BioJoint joint)
        {
            this.joint = joint;
            Transform target = joint.transform;
            axisMat = new float3x3(target.right, target.up, target.forward);
        }

        public float3 ProcessMotion(MotionType motionType)
        {
            if (Application.isPlaying&&motionType==MotionType.Slow)
            {
                UpdateRealistic(Time.deltaTime, maximumVelocity, maximumAcceleration,
                    ref currentValue, ref currentError, ref currentVelocity, ref currentAcceleration);
            }
            else
            {
                UpdateInstantaneous(targetValue,
                    out currentValue, out currentError, out currentVelocity, out currentAcceleration);
            }

            return currentValue*(float3)isEnableValue;
        }

        public void SetLowerLimit(float3 value)
        {
            lowerLimit = min(0, value);
        }

        public float3 GetLowerLimit()
        {
            float3 constraintMul = (1- constraintValue) * float.MinValue + 1;//OYM��false =minvalue ,true =1


            float3 result =0;
            if (joint.jointType == BioJointType.Rotational)
            {
                result = radians( lowerLimit);
            }
            else
            {
                result = lowerLimit;
            }
            result *= constraintMul;
            return result;
        }

        public void SetUpperLimit(float3 value)
        {
            upperLimit = max(0.0f, value);
        }

        public float3 GetUpperLimit()
        {
            float3 constraintMul = (1 - constraintValue) * float.MaxValue + 1;//OYM��false =maxvalue ,true =1
            float3 result = 0;
            if  (joint.jointType == BioJointType.Rotational)
            {
                result = radians(upperLimit);
            }
            else
            {
                result = upperLimit;
            }
            result *= constraintValue;
            return result;
        }

        public void SetTargetValue(float3 value, bool normalised = false)
        {
            float3 constraintMul = (1-constraintValue) * float.MaxValue +1;

            value = math.clamp(value, GetLowerLimit() * constraintValue, GetUpperLimit() * constraintValue);

            targetValue = value;
        }

        public float3 GetTargetValue(bool normalized = false)
        {
            return targetValue;
        }
        private static void UpdateInstantaneous(float3 targetValue, out float3 currentValue, out float3 currentError, out float3 currentVelocity, out float3 currentAcceleration)
        {
            currentValue = targetValue;
            currentError = 0;
            currentVelocity = 0;
            currentAcceleration = 0;
        }

        private static void UpdateRealistic(
            float deltaTime,float3 maxVelocity,float3 maxAcceleration,
            ref float3 currentValue, ref float3 currentError, ref float3 currentVelocity, ref float3 currentAcceleration)
        {
            if (deltaTime==0)
            {
                return;
            }

            float3 stoppingDistance =
                abs((currentVelocity*currentVelocity) / (2 * maxAcceleration*SLOWDOWN ) )+
               abs(currentAcceleration) / 2 * deltaTime * deltaTime +
                abs(currentVelocity) * deltaTime;

            currentAcceleration = select(
                    select(
                        -sign(currentVelocity) * min(
                                min(abs(currentVelocity) / deltaTime, maxAcceleration), 
                                abs((currentVelocity * currentVelocity) / (currentError * 2))),
                          -sign(currentVelocity) * min(abs(currentVelocity) / deltaTime, maxAcceleration),
                        currentError == 0),
                    sign(currentError) * min(abs(currentError) / deltaTime, maxAcceleration * SPEEDUP), 
                abs(currentError) > stoppingDistance);

            currentVelocity += currentAcceleration * deltaTime;
            currentVelocity = clamp(currentVelocity, -maxVelocity, maxVelocity);
            currentValue += currentVelocity * deltaTime;
        }
    }

}
