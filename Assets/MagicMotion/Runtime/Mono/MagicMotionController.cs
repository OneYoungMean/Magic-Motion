using System;
using System.Collections;
using System.Collections.Generic;
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
        private MMAvatarController muscleAndJointController;
        /// <summary>
        /// All constraint manager
        /// </summary>
        private MMConstraintsController constraintsController;
        #endregion

        #region UnityFunc
        // Start is called before the first frame update
        void Start()
        {

            ValueCheck();
        }
        #endregion

        #region LocalFunc
        /// <summary>
        /// the base value checker.
        /// </summary>
        /// <exception cref="NullReferenceException"></exception>
        /// <exception cref="InvalidOperationException"></exception>
        private void ValueCheck()
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
            if (muscleAndJointController==null)
            {
                muscleAndJointController = gameObject.AddComponent<MMAvatarController>();
                muscleAndJointController.Initialize(characterAnimator);
            }

            ValueCheck(inputMode);
        }
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
                    if (constraintsController==null)
                    {
                        BuildConstraintController();
                    }
                    break;
                case InputMode.FromAPI:
                    break;
                default:
                    return false;
            }
            return true;
        }
         private void Initialize()
        {
            
        }

        private void BuildConstraintController()
        {
            var IKRoot = new GameObject(characterAnimator.name + " IKRoot");
            IKRoot.transform.parent = characterAnimator.transform;
            IKRoot.transform.localPosition = Vector3.zero;
            IKRoot.transform.rotation = Quaternion.identity;
            IKRoot.transform.localScale = Vector3.one;

            constraintsController = IKRoot.AddComponent<MMConstraintsController>();
            constraintsController.Initialize( muscleAndJointController.motionJoints);

        }
        #endregion

        #region StaticFunc

        #endregion






        // Update is called once per frame
        void Update()
        {
            switch (inputMode)
            {
                case InputMode.FromHumanAnimator:
                    break;
                case InputMode.FromIK:
                    break;
                case InputMode.FromAPI:
                    break;
                default:
                    break;
            }
        }
    }
}

