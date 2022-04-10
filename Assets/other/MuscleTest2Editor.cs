using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MuscleTest2))]
public class MuscleTest2Editor : Editor
{
    MuscleTest2 muscleTest2;
    public void OnEnable()
    {
        muscleTest2=(MuscleTest2)target;
    }

    public override void OnInspectorGUI()
    {
        if (Application.isPlaying)
        {
            for (int i = 0; i < muscleTest2.currentPose.muscles.Length; i++)
            {
                muscleTest2.currentPose.muscles[i] = EditorGUILayout.Slider(i.ToString(), muscleTest2.currentPose.muscles[i], -1, 1);
            }
            serializedObject.ApplyModifiedProperties();
        }
    }
}
