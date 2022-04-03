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
        #region  Field&Property

        /// <summary>
        /// input mode that you want 
        /// </summary>
        public InputMode inputMode;
        /// <summary>
        /// The input animator
        /// </summary>
        public Animator targetAnimator;
        
        /// <summary>
        /// The character animator whitch has been controlled
        /// </summary>
        [SerializeField]
        private Animator characterAnimator;
        #endregion

        #region UnityFunc
        // Start is called before the first frame update
        void Start()
        {
            InitializeTransForm();
        }
        #endregion

        #region LocalFunc
        private void InitializeTransForm()
        {

        }
        #endregion

        #region StaticFunc
        private static void InitializeTransform(Animator animator)
        {

        }
        #endregion






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

