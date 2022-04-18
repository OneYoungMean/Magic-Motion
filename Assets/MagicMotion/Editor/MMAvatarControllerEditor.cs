using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace MagicMotion.UnityEditor
{
    using Mono;
    [CustomEditor(typeof(MMJointController))]
    public class MMAvatarControllerEditor : Editor
    {
        MMJointController controller;
        public void OnEnable()
        {
            controller = (MMJointController)target;
        }

        public override void OnInspectorGUI()
        {
            if (!Application.isPlaying)
            {
                if (controller.isInitialize)
                {
                    var muscles = controller.motionMuscles;
                    for (int i = 0; i < muscles.Length; i++)
                    {
                        muscles[i].value = EditorGUILayout.Slider(muscles[i].muscleName, muscles[i].value, -1, 1);
                    }
                    controller.UpdateMotion();
                }
                else
                {
                    if (GUILayout.Button("Generate Joint and muscle"))
                    {
                        controller.Initialize();
                    }
                }
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
}
