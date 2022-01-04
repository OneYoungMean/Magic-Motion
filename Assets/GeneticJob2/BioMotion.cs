using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using static Unity.Mathematics.math;
namespace BIOIK2
{
    public enum MotionType
    {
        Fast,//OYM:Instantaneous
        Slow,//OYM:Realistic
    }
    public class BioMotion : MonoBehaviour
    {
        public BioJoint joint;
        public bool3 isEnable;
        public bool3 constraint = true;//OYM:Ô¼Êø?Ô¼ÊøÉ¶?
        public float3 lowerLimit;
        public float3 upperLimit;
        public float3 targetValue;
        public float3 currentValue;
        public float3 currentError;
        public float3 currentVelocity;
        public float3 currentAcceleration;
        public float3x3 axisMat;
        public float maximumVelocity => (joint.jointType == BioJointType.Rotational ? Mathf.Rad2Deg :1f)*joint.segment.Character.maximumVelocity;
        public float maximumAcceleration => (joint.jointType == BioJointType.Rotational ? Mathf.Rad2Deg : 1f) * joint.segment.Character.maximumAcceleration;

        public const float SPEEDUP = 1;
        public const float SLOWDOWN = 1;

        public BioMotion(BioJoint joint)
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

            return currentValue*(float3)isEnable;
        }

        public void SetLowerLimit(float3 value)
        {
            lowerLimit = min(0, value);
        }

        public float3 GetLowerLimit(bool normalized = false)
        {
            float3 constraintValue = (float3)constraint * float.MaxValue + new float3(1,1,1);
            float3 result =0;
            if (normalized && joint.jointType == BioJointType.Rotational)
            {
                result = Mathf.Deg2Rad * lowerLimit;
            }
            else
            {
                result = lowerLimit;
            }
            result += constraintValue;
            return result;
        }

        public void SetTargetValue(float3 value, bool normalised = false)
        {
            float3 constraintValue = (float3)constraint * float.MaxValue + new float3(1, 1, 1);

            if (normalised && joint.jointType == BioJointType.Rotational)
            {
                value *= Mathf.Rad2Deg;
            }
            value = math.clamp(value, lowerLimit * constraintValue, upperLimit * constraintValue);

            targetValue = value;
        }

        public float3 GetTargetValue(bool normalized = false)
        {
            if (normalized&&joint.jointType==BioJointType.Rotational)
            {
                return Mathf.Deg2Rad * targetValue;
            }
            else
            {
                return targetValue;
            }
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
