using UnityEngine;

namespace BIOIK {

	[AddComponentMenu("")]
	public class BioSegment : MonoBehaviour {
		public BioIK Character;
		public Transform Transform;
		public BioSegment Parent;
		public BioSegment[] Childs = new BioSegment[0];
		public BioJoint Joint = null;
		public BioObjective[] Objectives = new BioObjective[0];

		void Awake() {

		}

		void Start() {

		}

		void OnDestroy() {

		}

		public BioSegment Create(BioIK character) {
			Character = character;
			Transform = transform;
			hideFlags = HideFlags.HideInInspector;
			return this;
		}

		public Vector3 GetAnchoredPosition() {
			return Joint == null ? Transform.position : Joint.GetAnchorInWorldSpace();
		}

		public void AddChild(BioSegment child) {
			System.Array.Resize(ref Childs, Childs.Length+1);
			Childs[Childs.Length-1] = child;
		}

        public void RenewRelations() //OYM:置零所有的child,并且寻找父节点
        { //OYM:设置一个新的relation
            Parent = null;
			System.Array.Resize(ref Childs, 0);
			if(Transform != Character.transform) {
                Parent = Character.FindSegment(Transform.parent); //OYM:寻找父节点
                Parent.AddChild(this);
			}
		}

		public BioJoint AddJoint() {
			if(Joint != null) {
				Debug.Log("The segment already has a joint.");
			} else {
				Joint = Utility.AddBioJoint(this);
				Character.Refresh();
			}
			return Joint;
		}

		public BioObjective AddObjective(ObjectiveType type) {
			BioObjective objective = Utility.AddObjective(this, type);
			if(objective == null) {
				Debug.Log("The objective could not be found.");
				return null;
			} else {
				System.Array.Resize(ref Objectives, Objectives.Length+1);
				Objectives[Objectives.Length-1] = objective;
				Character.Refresh();
				return Objectives[Objectives.Length-1];
			}
		}
	}

}