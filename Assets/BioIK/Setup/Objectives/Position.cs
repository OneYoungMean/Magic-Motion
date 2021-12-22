using UnityEngine;

namespace BIOIK {

    //This objective aims to minimise the translational distance between the transform and the target.
    //OYM:找到距离当前点最近的状态
    [AddComponentMenu("")]
	public class Position : BioObjective {
		
		[SerializeField] public Transform Target;
		[SerializeField] public double TPX, TPY, TPZ;
		[SerializeField] public double MaximumError = 0.001;

		private double ChainLength;
		private double Rescaling;
		//private Vector3 Root;

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.Position;
		}
        //OYM:更新了chain长度，更新了target位置
        public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}
			ChainLength = 0.0;
			Transform[] chain = Segment.Character.Evolution.GetModel().FindObjectivePtr(this).Node.Chain;;
			for(int i=0; i<chain.Length-1; i++) {
				ChainLength += Vector3.Distance(chain[i].position, chain[i+1].position);
			}
			Rescaling = (Utility.PI * Utility.PI) / (ChainLength * ChainLength);
			//Root = chain[0].position;
			if(Target != null) {
				Vector3 position = Target.position;
				TPX = position.x;
				TPY = position.y;
				TPZ = position.z;
			}
		}

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			//Adaptive
			/*
			double loss = System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
			double s = System.Math.Sqrt(
				(node.Chain.Length+loss)
				*
				(System.Math.Sqrt((WPX-Root.x)*(WPX-Root.x) + (WPY-Root.y)*(WPY-Root.y) + (WPZ-Root.z)*(WPZ-Root.z))+loss)
				);
			loss = Utility.PI * loss / s;
			return Weight * loss * loss;
			*/

			//Fast and slightly adaptive
			return Weight * Rescaling * ((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
			
			//Fastest but not adaptive
			//return Weight * ((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ)) <= MaximumError;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
		}

		public void SetTargetTransform(Transform target) {
			Target = target;
			if(Target != null) {
				SetTargetPosition(Target.position);
			}
		}

		public Transform GetTargetTransform() {
			return Target;
		}

		public void SetTargetPosition(Vector3 position) {
			TPX = position.x;
			TPY = position.y;
			TPZ = position.z;
		}

		public Vector3 GetTargetPosition() {
			return new Vector3((float)TPX, (float)TPY, (float)TPZ);
		}
		
		public void SetMaximumError(double units) {
			MaximumError = units;
		}

		public double GetMaximumError() {
			return MaximumError;
		}

	}

}