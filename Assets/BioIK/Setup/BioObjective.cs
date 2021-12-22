using UnityEngine;

namespace BIOIK {

	[AddComponentMenu("")]
    public abstract class BioObjective : MonoBehaviour
    { //OYM:抽象类,包括各种工具
        public BioSegment Segment;
		public double Weight = 1.0;

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

		public void SetWeight(double weight) {
			if(weight < 0.0) {
				Debug.Log("Weight must be at least zero.");
				Weight = 0.0;
				return;
			}
			Weight = weight;
		}

		public double GetWeight() {
			return Weight;
		}

		public abstract ObjectiveType GetObjectiveType();
		public abstract void UpdateData();
		public abstract double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration);
		public abstract bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration);
		public abstract double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration);
	}

}