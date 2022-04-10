using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace MagicMotion.UnityEditor
{
    using Mono;
    [CustomEditor(typeof(MagicMotionJointAndMuscleController))]
    public class MagicMotionJointAndMuscleControllerEditor : Editor
    {
        MagicMotionJointAndMuscleController controller;
        public void OnEnable()
        {
            controller = (MagicMotionJointAndMuscleController)target;
        }

        public override void OnInspectorGUI()
        {

            if (controller.isInitialize)
            {
                var muscles = controller.motionMuscles;
                for (int i = 0; i < muscles.Length; i++)
                {
                    muscles[i].value = EditorGUILayout.Slider(muscles[i].muscleName, muscles[i].value, -1, 1);
                }
                controller.UpdateMuscle();
                controller.UpdateMotion();
            }
            else
            {
                if (GUILayout.Button("Generate Joint and muscle"))
                {
                    controller.Initialize();
                }
            }
            serializedObject.ApplyModifiedProperties();
        }
    }
}
