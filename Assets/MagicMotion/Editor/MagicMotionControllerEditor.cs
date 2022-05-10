using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace MagicMotion.UnityEditor
{
    using Mono;
    [CustomEditor(typeof(MagicMotionController))]
    public class MagicMotionControllerEditor : Editor
    {
        private MagicMotionController controller;
        private void OnEnable()
        {
            controller = (MagicMotionController)target; 
        }

/*        public override void OnInspectorGUI()
        {
            if (GUILayout.Button("Save"))
            {

                controller.UpdateCruve();
            }
            base.OnInspectorGUI();
        }*/
    }

}
