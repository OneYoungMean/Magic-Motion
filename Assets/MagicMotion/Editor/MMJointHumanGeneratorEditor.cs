using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace MagicMotion.UnityEditor
{
    using Mono;
    [CustomEditor(typeof(MMHumanGenerator))]
    public class MMJointHumanGeneratorEditor : Editor
    {
        public MMHumanGenerator controller;
        public void OnEnable()
        {
            controller = (MMHumanGenerator)target;
        }

        public override void OnInspectorGUI()
        {
            if (!controller.isInitialize)
            {
                if (GUILayout.Button("Generate"))
                {
                    controller.Generate();
                }
            }
            base.OnInspectorGUI();
        }
    }
}