using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace MagicMotion.Mono
{
    public class MagicMotionController : MonoBehaviour
    {
        /// <summary>
        /// Define the input function
        /// </summary>
        public enum InputMode
        {
            FromHumanAnimator,//OYM: input from another humanAnimator
            FromIK,//OYM: input from Transform
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
        /// <summary>
        /// To convert human animator as native data.
        /// </summary>
        private MMHumanGenerator jointController;
        /// <summary>
        /// is Initialize
        /// </summary>
        private bool isInitial;
        [SerializeField]
        private int editorIndex;

        #endregion

        #region UnityFunc
        // Start is called before the first frame update
        void Start()
        {

/*            bool isValueCheck = ValueCheck();
            if (!isValueCheck)
            {
                return;
            }
            Initialize();*/
        }
        #endregion

        #region LocalFunc
        /// <summary>
        /// the base value checker.
        /// </summary>
        /// <exception cref="NullReferenceException"></exception>
        /// <exception cref="InvalidOperationException"></exception>
        private bool ValueCheck()
        {
            //OYM£ºcharacter and muscleAndJointController
            if (characterAnimator == null)
            {
                characterAnimator = gameObject.GetComponent<Animator>();
                if (characterAnimator == null)
                {
                    throw new NullReferenceException("characterAnimator is empty");
                }
            }
            if (!characterAnimator.isHuman)
            {
                throw new InvalidOperationException("characterAnimator is not human");
            }
            if (jointController==null&&!gameObject.TryGetComponent<MMHumanGenerator>(out jointController))
            {
                jointController = gameObject.AddComponent<MMHumanGenerator>();
            }
            return ValueCheck(inputMode);
            
        }
        /// <summary>
        /// check value from different input mode
        /// </summary>
        /// <param name="inputMode"></param>
        /// <returns></returns>
        private bool ValueCheck(InputMode inputMode)
        {
            
            switch (inputMode)
            {
                case InputMode.FromHumanAnimator:
                    if (targetAnimator==null)
                    {
                        Debug.Log("targetAnimator is empty ,are you missing it?");
                        return false;
                    }
                    break;
                case InputMode.FromIK:
                    break;
                case InputMode.FromAPI:
                    break;
                default:
                    return false;
            }
            return true;
        }
        /// <summary>
        /// Initialize data
        /// </summary>
         private void Initialize()
        {
            jointController.Generate(characterAnimator);
            isInitial = true;
        }
        #endregion


        #region StaticFunc

        #endregion
    }
}

