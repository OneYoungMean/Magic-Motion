using UnityEngine;

namespace BIOIK {

	[AddComponentMenu("")]
    public abstract class BioObjective : MonoBehaviour
    { //OYM:抽象类,包括各种工具
        public BioSegment Segment;
		public float Weight = 1.0f;

		void Awake() {

		}

		void Start() {

		}

		void OnEnable() {
			if(Segment != null) {
				Segment.Character.Refresh();
			}
		}

		void OnDisable() {
			if(Segment != null) {
				Segment.Character.Refresh();
			}
		}

		void OnDestroy() {

		}

		public BioObjective Create(BioSegment segment) {
			Segment = segment;
			hideFlags = HideFlags.HideInInspector;
			return this;
		}

		public void Remove() {
			for(int i=0; i<Segment.Objectives.Length; i++) {
				if(Segment.Objectives[i] == this) {
					for(int j=i; j<Segment.Objectives.Length-1; j++) {
						Segment.Objectives[j] = Segment.Objectives[j+1];
					}
					System.Array.Resize(ref Segment.Objectives, Segment.Objectives.Length-1);
					break;
				}
			}
			if(Segment != null) {
				if(Segment.Character != null) {
					Segment.Character.Refresh();
				}
			}
			Utility.Destroy(this);
		}

		public void Erase() {
			Utility.Destroy(this);
		}

		public void SetWeight(float weight) {
			if(weight < 0.0f) {
				Debug.Log("Weight must be at least zero.");
				Weight = 0.0f;
				return;
			}
			Weight = weight;
		}

		public float GetWeight() {
			return Weight;
		}

		public abstract ObjectiveType GetObjectiveType();
		public abstract void UpdateData();
		public abstract float ComputeLoss(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration);
		public abstract bool CheckConvergence(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration);
		public abstract float ComputeValue(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration);
	}

}