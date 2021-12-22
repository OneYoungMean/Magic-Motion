using UnityEngine;

namespace GeneticJob
{
    public unsafe class GeneticMono : MonoBehaviour
    {
       public  AnimatorTree inputAnimatorTree;
        public AnimatorTree outputAnimatorTree;
        private GeneticJoint[] GeneticJointArray;
        private Transform[] GeneticJointTransformArray;

        public GeneticIKKernal geneticIKKernal;

        public bool isAsync;
        public void Start()
        {
            inputAnimatorTree.Initialize();

           GeneticJointArray = inputAnimatorTree.ToGeneticJointArray();
            GeneticJointTransformArray= inputAnimatorTree.ToTransformArray();

            geneticIKKernal = new GeneticIKKernal();
            geneticIKKernal.SetData(GeneticJointTransformArray, GeneticJointArray);
            geneticIKKernal.Schedule(isAsync, 32, 1);
        }

        public void Update()
        {
            
        }

        public void OnDestroy()
        {
            geneticIKKernal. Dispose();
        }
    }
}