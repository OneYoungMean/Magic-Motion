using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MagicMotion
{
    public class MagicMotionController : MonoBehaviour
    {
        /// <summary>
        /// Define the input function
        /// </summary>
        public enum InputMode
        {
            FromHumanAnimator,//OYM: input from another humanAnimator
            FromTransform,//OYM: input from Transform
            FromAPI//OYM: input from API
        }
        /// <summary>
        /// input mode that you want 
        /// </summary>
        public InputMode inputMode;
        /// <summary>
        /// the input animator
        /// </summary>
        public Animator targetAnimator;

        [SerializeField]
        private Animator characterAnimator;
        // Start is called before the first frame update
        void Start()
        {
            InitializeTransForm();
        }

        private void InitializeTransForm()
        {

        }

        // Update is called once per frame
        void Update()
        {
            switch (inputMode)
            {
                case InputMode.FromHumanAnimator:
                    break;
                case InputMode.FromTransform:
                    break;
                case InputMode.FromAPI:
                    break;
                default:
                    break;
            }
        }
    }
}

