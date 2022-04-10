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
        /// Auto generate,all IK's point root; 
        /// </summary>
        private GameObject IKRoot;
        /// <summary>
        /// To convert human animator as native data.
        /// </summary>
        private MagicMotionJointAndMuscleController muscleAndJointController;
        #endregion

        #region UnityFunc
        // Start is called before the first frame update
        void Start()
        {

            InitializeTransform();
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
            if (muscleAndJointController=null)
            {
                muscleAndJointController = gameObject.AddComponent<MagicMotionJointAndMuscleController>();
                muscleAndJointController.Initialize(characterAnimator);
            }
        }
        private bool ValueCheck(InputMode inputMode)
        {
            return true;
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
                    if (IKRoot==null)
                    {


                    }
                    break;
                case InputMode.FromAPI:
                    break;
                default:
                    break;
            }
        }
            private void Initialize()
        {
            
        }

        private void InitializeTransform()
        {

        }

        private void BuildIKPoint()
        {
            IKRoot = new GameObject(characterAnimator.name + "IKRoot");
            IKRoot.transform.parent = characterAnimator.transform;
            IKRoot.transform.localPosition = Vector3.zero;
            IKRoot.transform.rotation = Quaternion.identity;
            IKRoot.transform.localScale = Vector3.one;

            var joints = muscleAndJointController.motionJoints;
            for (int i = 0; i < joints.Length; i++)
            {

            }
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

